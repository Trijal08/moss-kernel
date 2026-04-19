use crate::drivers::fdt_prober::get_fdt;
use alloc::string::{String, ToString};

pub const MAX_FDT_SZ: usize = 2 * 1024 * 1024;

pub fn get_cmdline() -> Option<String> {
    let fdt = get_fdt();

    // Some(fdt.chosen()?.bootargs()?.to_string())
    Some("--rootfs=ext4fs --automount=/dev,devfs --automount=/tmp,tmpfs --automount=/proc,procfs --automount=/sys,sysfs --init=/bin/ash --init-arg=-i".to_string())
}
