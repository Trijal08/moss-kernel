#![no_std]
#![no_main]
#![feature(used_with_arg)]
#![feature(likely_unlikely)]
#![feature(box_as_ptr)]
#![allow(internal_features)]
#![cfg_attr(test, feature(core_intrinsics))]
#![feature(custom_test_frameworks)]
#![reexport_test_harness_main = "test_main"]
#![test_runner(crate::testing::test_runner)]

use alloc::{
    boxed::Box,
    string::{String, ToString},
    vec,
    vec::Vec,
};
use arch::{Arch, ArchImpl};
use core::panic::PanicInfo;
use drivers::{fdt_prober::get_fdt, fs::register_fs_drivers};
use fs::VFS;
use getargs::{Opt, Options};
use libkernel::{
    CpuOps, KernAddressSpace,
    fs::{
        BlockDevice, OpenFlags, attr::FilePermissions, blk::ramdisk::RamdiskBlkDev, path::Path,
        pathbuf::PathBuf,
    },
    memory::{
        address::{PA, VA},
	permissions::PtePermissions,
        proc_vm::address_space::VirtualMemory,
        region::PhysMemoryRegion,
	region::VirtMemoryRegion,
    },
};
use log::{error, warn};
use process::ctx::UserCtx;
use sched::{
    sched_init, spawn_kernel_work, syscall_ctx::ProcessCtx, uspc_ret::dispatch_userspace_task,
};

extern crate alloc;
extern crate moss_macros;

mod arch;
mod clock;
mod console;
mod drivers;
mod fs;
mod initramfs;
mod interrupts;
mod kernel;
mod memory;
mod net;
mod process;
mod sched;
mod sync;
#[cfg(test)]
pub mod testing;

#[panic_handler]
fn on_panic(info: &PanicInfo) -> ! {
    ArchImpl::disable_interrupts();

    let panic_msg = info.message();

    if let Some(location) = info.location() {
        error!(
            "Kernel panicked at {}:{}:{}: {}",
            location.file(),
            location.line(),
            location.column(),
            panic_msg
        );
    } else {
        error!("Kernel panicked at unknown location: {panic_msg}");
    }

    ArchImpl::power_off();
}

fn read_fdt_u32_or_u64(prop: &fdt_parser::Property<'_>) -> libkernel::error::Result<u64> {
    match prop.raw_value().len() {
        4 => Ok(prop.u32().into()),
        8 => Ok(prop.u64()),
        _ => Err(libkernel::error::KernelError::InvalidValue),
    }
}

fn initrd_region() -> Option<PhysMemoryRegion> {
    let dt = get_fdt();
    let chosen = dt.find_nodes("/chosen").next()?;
    let start_addr = chosen
        .find_property("linux,initrd-start")
        .map(|prop| read_fdt_u32_or_u64(&prop))
        .transpose()
        .unwrap_or_else(|e| panic!("Invalid linux,initrd-start property: {e}"))?;
    let end_addr = chosen
        .find_property("linux,initrd-end")
        .map(|prop| read_fdt_u32_or_u64(&prop))
        .transpose()
        .unwrap_or_else(|e| panic!("Invalid linux,initrd-end property: {e}"))?;

    Some(PhysMemoryRegion::from_start_end_address(
        PA::from_value(start_addr as _),
        PA::from_value(end_addr as _),
    ))
}

fn map_initrd_bytes(region: PhysMemoryRegion) -> libkernel::error::Result<&'static [u8]> {
    const INITRD_BASE: usize = 0xffff_9800_0000_0000;

    let aligned_start = PA::from_value(region.start_address().value() & !0xfffusize);
    let offset = region.start_address().value() - aligned_start.value();
    let aligned_size = (offset + region.size() + 0xfff) & !0xfffusize;
    let virt = VA::from_value(INITRD_BASE);

    ArchImpl::kern_address_space().lock_save_irq().map_normal(
        PhysMemoryRegion::new(aligned_start, aligned_size),
        VirtMemoryRegion::new(virt, aligned_size),
        PtePermissions::ro(false),
    )?;

    // SAFETY: The region has been mapped into the kernel address space above
    // and remains mapped for the rest of boot.
    Ok(unsafe {
        core::slice::from_raw_parts(virt.cast::<u8>().as_ptr().add(offset), region.size())
    })
}

async fn launch_init(mut ctx: ProcessCtx, mut opts: KOptions) {
    let init = opts
        .init
        .unwrap_or_else(|| panic!("No init specified in kernel command line"));

    let initrd_region = initrd_region();

    // Set time to rtc time if possible
    if let Some(rtc) = drivers::rtc::get_rtc()
        && let Some(time) = rtc.time()
    {
        clock::realtime::set_date(time);
    }

    let root_fs = opts
        .root_fs
        .unwrap_or_else(|| panic!("No root FS driver specified in kernel command line"));

    if root_fs == "initramfs" {
        let initrd = initrd_region.unwrap_or_else(|| panic!("No initrd present for initramfs"));
        let initrd_bytes =
            map_initrd_bytes(initrd).unwrap_or_else(|e| panic!("Failed to map initrd: {e}"));

        VFS.mount_root("tmpfs", None)
            .await
            .unwrap_or_else(|e| panic!("Failed to mount initramfs tmpfs root: {e}"));

        initramfs::populate_root(VFS.root_inode(), initrd_bytes)
            .await
            .unwrap_or_else(|e| panic!("Failed to unpack initramfs: {e}"));
    } else {
        let initrd_block_dev: Option<Box<dyn BlockDevice>> = initrd_region.map(|region| {
            Box::new(
                RamdiskBlkDev::new(
                    region,
                    VA::from_value(0xffff_9800_0000_0000),
                    &mut *ArchImpl::kern_address_space().lock_save_irq(),
                )
                .unwrap(),
            ) as Box<dyn BlockDevice>
        });

        VFS.mount_root(&root_fs, initrd_block_dev)
            .await
            .unwrap_or_else(|e| panic!("Failed to mount root FS: {e}"));
    }

    // Process all automounts.
    for (path, fs) in opts.automounts.iter() {
        let mount_point = VFS
            .resolve_path_absolute(path, VFS.root_inode())
            .await
            .unwrap_or_else(|e| panic!("Could not find automount path: {}. {e}", path.as_str()));

        VFS.mount(mount_point, fs, None)
            .await
            .unwrap_or_else(|e| panic!("Automount failed: {e}"));
    }

    let inode = VFS
        .resolve_path_absolute(&init, VFS.root_inode())
        .await
        .expect("Unable to find init");

    let task = ctx.shared().clone();

    // Ensure that the exec() call applies to init.
    assert!(task.process.tgid.is_init());

    // Now that the root fs has been mounted, set the real root inode as the
    // cwd and root.
    *task.cwd.lock_save_irq() = (VFS.root_inode(), PathBuf::from("/"));
    *task.root.lock_save_irq() = (VFS.root_inode(), PathBuf::from("/"));

    let console = VFS
        .open(
            Path::new("/dev/console"),
            OpenFlags::O_RDWR,
            VFS.root_inode(),
            FilePermissions::empty(),
            &task,
        )
        .await
        .expect("Could not open console for init process");

    {
        let mut fd_table = task.fd_table.lock_save_irq();

        // stdin, stdout, stderr
        fd_table
            .insert(console.clone())
            .expect("Could not clone FD");
        fd_table
            .insert(console.clone())
            .expect("Could not clone FD");
        fd_table
            .insert(console.clone())
            .expect("Could not clone FD");
    }

    #[cfg(test)]
    test_main();

    drop(task);

    let mut init_args = vec![init.as_str().to_string()];

    init_args.append(&mut opts.init_args);

    process::exec::kernel_exec(&mut ctx, init.as_path(), inode, init_args, vec![])
        .await
        .expect("Could not launch init process");
}

struct KOptions {
    init: Option<PathBuf>,
    root_fs: Option<String>,
    automounts: Vec<(PathBuf, String)>,
    init_args: Vec<String>,
}

fn parse_args(args: &str) -> KOptions {
    let mut kopts = KOptions {
        init: None,
        root_fs: None,
        automounts: Vec::new(),
        init_args: Vec::new(),
    };

    let mut opts = Options::new(args.split(" "));

    loop {
        match opts.next_opt() {
            Ok(Some(arg)) => match arg {
                Opt::Long("init") => kopts.init = Some(PathBuf::from(opts.value().unwrap())),
                Opt::Long("init-arg") => kopts.init_args.push(opts.value().unwrap().to_string()),
                Opt::Long("rootfs") => kopts.root_fs = Some(opts.value().unwrap().to_string()),
                Opt::Long("automount") => {
                    let string = opts.value().unwrap();
                    let mut split = string.split(",");
                    let path = split.next().unwrap();
                    let fs = split.next().unwrap();

                    kopts.automounts.push((PathBuf::from(path), fs.to_string()));
                }
                Opt::Long(x) => warn!("Unknown option {x}"),
                Opt::Short(x) => warn!("Unknown option {x}"),
            },
            Ok(None) => return kopts,
            Err(e) => error!("Could not parse option: {e}, ignoring."),
        }
    }
}

pub fn kmain(args: String, ctx_frame: *mut UserCtx) {
    sched_init();

    register_fs_drivers();

    let kopts = parse_args(&args);

    {
        // SAFETY: kmain is called prior to init being launched. Thefore, we
        // will be the only access to `ctx` at this point.
        let mut ctx = unsafe { ProcessCtx::from_current() };
        let ctx2 = unsafe { ctx.clone() };

        spawn_kernel_work(&mut ctx, launch_init(ctx2, kopts));
    }

    dispatch_userspace_task(ctx_frame);
}
