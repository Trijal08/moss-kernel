use crate::{
    arch::ArchImpl,
    drivers::{
        DeviceDescriptor, Driver, DriverManager, fdt_prober::get_fdt, init::PlatformBus,
        probe::DeviceMatchType,
    },
    kernel_driver,
};
use aarch64_cpu::registers::{Readable, Writeable};
use alloc::{boxed::Box, sync::Arc};
use libkernel::{
    KernAddressSpace, VirtualMemory,
    error::{ProbeError, Result},
    memory::{
        address::{PA, VA},
        region::PhysMemoryRegion,
    },
};
use tock_registers::{register_structs, registers::ReadWrite};

const USI_MODE_NONE: u32 = 0;
const USI_MODE_UART: u32 = 1;
const USI_MODE_SPI: u32 = 2;
const USI_MODE_I2C: u32 = 3;
const USI_MODE_I2C1: u32 = 4;
const USI_MODE_I2C0_1: u32 = 5;
const USI_MODE_UART_I2C1: u32 = 6;

const USI_V1_SW_CONF_NONE: u32 = 0x0;
const USI_V1_SW_CONF_I2C0: u32 = 0x1;
const USI_V1_SW_CONF_I2C1: u32 = 0x2;
const USI_V1_SW_CONF_I2C0_1: u32 = 0x3;
const USI_V1_SW_CONF_SPI: u32 = 0x4;
const USI_V1_SW_CONF_UART: u32 = 0x8;
const USI_V1_SW_CONF_UART_I2C1: u32 = 0xa;
const USI_V1_SW_CONF_MASK: u32 = 0x0f;

const USI_V2_SW_CONF_NONE: u32 = 0x0;
const USI_V2_SW_CONF_UART: u32 = 1 << 0;
const USI_V2_SW_CONF_SPI: u32 = 1 << 1;
const USI_V2_SW_CONF_I2C: u32 = 1 << 2;
const USI_V2_SW_CONF_MASK: u32 = USI_V2_SW_CONF_UART | USI_V2_SW_CONF_SPI | USI_V2_SW_CONF_I2C;

const USI_CON_RESET: u32 = 1 << 0;
const USI_OPTION_CLKREQ_ON: u32 = 1 << 1;
const USI_OPTION_CLKSTOP_ON: u32 = 1 << 2;

register_structs! {
    ExynosUsiRegBank {
        (0x000 => _reserved0),
        (0x004 => con: ReadWrite<u32>),
        (0x008 => option: ReadWrite<u32>),
        (0x00c => @END),
    }
}

#[derive(Clone, Copy)]
enum UsiVersion {
    V1,
    V2,
}

#[derive(Clone, Copy)]
struct UsiVariant {
    version: UsiVersion,
    sw_conf_mask: u32,
    min_mode: u32,
    max_mode: u32,
}

const EXYNOS850_USI: UsiVariant = UsiVariant {
    version: UsiVersion::V2,
    sw_conf_mask: USI_V2_SW_CONF_MASK,
    min_mode: USI_MODE_NONE,
    max_mode: USI_MODE_I2C,
};

const EXYNOS8895_USI: UsiVariant = UsiVariant {
    version: UsiVersion::V1,
    sw_conf_mask: USI_V1_SW_CONF_MASK,
    min_mode: USI_MODE_NONE,
    max_mode: USI_MODE_UART_I2C1,
};

pub struct ExynosUsi {
    name: &'static str,
    _regs: Option<&'static mut ExynosUsiRegBank>,
    _sysreg: VA,
    _sw_conf_offset: usize,
    _mode: u32,
}

unsafe impl Send for ExynosUsi {}
unsafe impl Sync for ExynosUsi {}

impl Driver for ExynosUsi {
    fn name(&self) -> &'static str {
        self.name
    }
}

fn variant_mode_value(variant: UsiVariant, mode: u32) -> Option<u32> {
    match variant.version {
        UsiVersion::V1 => match mode {
            USI_MODE_NONE => Some(USI_V1_SW_CONF_NONE),
            USI_MODE_UART => Some(USI_V1_SW_CONF_UART),
            USI_MODE_SPI => Some(USI_V1_SW_CONF_SPI),
            USI_MODE_I2C => Some(USI_V1_SW_CONF_I2C0),
            USI_MODE_I2C1 => Some(USI_V1_SW_CONF_I2C1),
            USI_MODE_I2C0_1 => Some(USI_V1_SW_CONF_I2C0_1),
            USI_MODE_UART_I2C1 => Some(USI_V1_SW_CONF_UART_I2C1),
            _ => None,
        },
        UsiVersion::V2 => match mode {
            USI_MODE_NONE => Some(USI_V2_SW_CONF_NONE),
            USI_MODE_UART => Some(USI_V2_SW_CONF_UART),
            USI_MODE_SPI => Some(USI_V2_SW_CONF_SPI),
            USI_MODE_I2C => Some(USI_V2_SW_CONF_I2C),
            _ => None,
        },
    }
}

fn write_sysreg_bits(base: VA, offset: usize, mask: u32, value: u32) {
    let reg = base.as_ptr_mut().wrapping_add(offset).cast::<u32>();
    let mut current = unsafe { reg.read_volatile() };
    current &= !mask;
    current |= value & mask;
    unsafe { reg.write_volatile(current) };
}

fn resolve_sysreg(node: &fdt_parser::Node<'static>) -> Result<(VA, usize)> {
    let mut sysreg = node
        .find_property("samsung,sysreg")
        .ok_or(ProbeError::NoMatch)?
        .u32_list();

    let phandle = sysreg.next().ok_or(ProbeError::NoMatch)?;
    let sw_conf_offset = sysreg.next().ok_or(ProbeError::NoMatch)? as usize;

    let sysreg_node = get_fdt()
        .get_node_by_phandle(phandle.into())
        .ok_or(ProbeError::Deferred)?;

    let region = sysreg_node
        .reg()
        .ok_or(ProbeError::NoReg)?
        .next()
        .ok_or(ProbeError::NoReg)?;

    let size = region.size.ok_or(ProbeError::NoRegSize)?;

    let mem = ArchImpl::kern_address_space()
        .lock_save_irq()
        .map_mmio(PhysMemoryRegion::new(
            PA::from_value(region.address as usize),
            size,
        ))?;

    Ok((mem, sw_conf_offset))
}

fn probe_usi(
    _dm: &mut DriverManager,
    d: DeviceDescriptor,
    variant: UsiVariant,
) -> Result<Arc<dyn Driver>> {
    match d {
        DeviceDescriptor::Fdt(fdt_node, _) => {
            let mode = fdt_node
                .find_property("samsung,mode")
                .map(|prop| prop.u32())
                .ok_or(ProbeError::NoMatch)?;

            if mode < variant.min_mode || mode > variant.max_mode {
                return Err(ProbeError::NoMatch.into());
            }

            let sw_conf_value = variant_mode_value(variant, mode).ok_or(ProbeError::NoMatch)?;
            let (sysreg, sw_conf_offset) = resolve_sysreg(&fdt_node)?;

            write_sysreg_bits(sysreg, sw_conf_offset, variant.sw_conf_mask, sw_conf_value);

            let regs = match variant.version {
                UsiVersion::V1 => None,
                UsiVersion::V2 => {
                    let region = fdt_node
                        .reg()
                        .ok_or(ProbeError::NoReg)?
                        .next()
                        .ok_or(ProbeError::NoReg)?;

                    let size = region.size.ok_or(ProbeError::NoRegSize)?;

                    let mem = ArchImpl::kern_address_space().lock_save_irq().map_mmio(
                        PhysMemoryRegion::new(PA::from_value(region.address as usize), size),
                    )?;

                    let regs = unsafe { &mut *(mem.as_ptr_mut() as *mut ExynosUsiRegBank) };

                    let mut con = regs.con.get();
                    con &= !USI_CON_RESET;
                    regs.con.set(con);

                    if fdt_node.find_property("samsung,clkreq-on").is_some() {
                        let mut option = regs.option.get();
                        option &= !USI_OPTION_CLKSTOP_ON;
                        option |= USI_OPTION_CLKREQ_ON;
                        regs.option.set(option);
                    }

                    Some(regs)
                }
            };

            Ok(Arc::new(ExynosUsi {
                name: fdt_node.name,
                _regs: regs,
                _sysreg: sysreg,
                _sw_conf_offset: sw_conf_offset,
                _mode: mode,
            }))
        }
    }
}

fn exynos850_usi_probe(dm: &mut DriverManager, d: DeviceDescriptor) -> Result<Arc<dyn Driver>> {
    probe_usi(dm, d, EXYNOS850_USI)
}

fn exynos8895_usi_probe(dm: &mut DriverManager, d: DeviceDescriptor) -> Result<Arc<dyn Driver>> {
    probe_usi(dm, d, EXYNOS8895_USI)
}

pub fn exynos_usi_init(bus: &mut PlatformBus, _dm: &mut DriverManager) -> Result<()> {
    bus.register_platform_driver(
        DeviceMatchType::FdtCompatible("samsung,exynos850-usi"),
        Box::new(exynos850_usi_probe),
    );
    bus.register_platform_driver(
        DeviceMatchType::FdtCompatible("samsung,exynos8895-usi"),
        Box::new(exynos8895_usi_probe),
    );

    Ok(())
}

kernel_driver!(exynos_usi_init);
