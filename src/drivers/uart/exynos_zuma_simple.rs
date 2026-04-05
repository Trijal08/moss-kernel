use crate::{
    arch::ArchImpl,
    drivers::{
        DeviceDescriptor, Driver, DriverManager,
        init::PlatformBus,
        probe::{DeviceMatchType, FdtFlags},
    },
    kernel_driver,
};
use aarch64_cpu::registers::{Readable, Writeable};
use alloc::{boxed::Box, sync::Arc};
use core::hint::spin_loop;
use libkernel::{
    KernAddressSpace, VirtualMemory,
    error::{ProbeError, Result},
    memory::{
        address::{PA, VA},
        region::PhysMemoryRegion,
    },
};
use tock_registers::{register_structs, registers::ReadWrite};

use super::{UART_CHAR_DEV, Uart, UartDriver};

const UFCON_FIFO_ENABLE: u32 = 1 << 0;
const UTRSTAT_TXFE: u32 = 1 << 1;
const UFSTAT_TXFULL: u32 = 1 << 24;
const UFSTAT_RXFULL: u32 = 1 << 8;
const UFSTAT_RXMASK: u32 = 0xff;

register_structs! {
    ZumaUartRegBank {
        (0x000 => _ulcon: ReadWrite<u32>),
        (0x004 => _ucon: ReadWrite<u32>),
        (0x008 => ufcon: ReadWrite<u32>),
        (0x00c => _umcon: ReadWrite<u32>),
        (0x010 => utrstat: ReadWrite<u32>),
        (0x014 => _uerstat: ReadWrite<u32>),
        (0x018 => ufstat: ReadWrite<u32>),
        (0x01c => _umstat: ReadWrite<u32>),
        (0x020 => utxh: ReadWrite<u32>),
        (0x024 => urxh: ReadWrite<u32>),
        (0x028 => @END),
    }
}

pub struct ZumaSimpleUart {
    regs: &'static mut ZumaUartRegBank,
    fifo_size: usize,
}

unsafe impl Send for ZumaSimpleUart {}
unsafe impl Sync for ZumaSimpleUart {}

impl ZumaSimpleUart {
    pub fn new(addr: VA, fifo_size: usize) -> Self {
        let regs = unsafe { &mut *(addr.as_ptr_mut() as *mut ZumaUartRegBank) };
        Self { regs, fifo_size }
    }

    fn tx_ready(&self) -> bool {
        if (self.regs.ufcon.get() & UFCON_FIFO_ENABLE) != 0 {
            (self.regs.ufstat.get() & UFSTAT_TXFULL) == 0
        } else {
            (self.regs.utrstat.get() & UTRSTAT_TXFE) != 0
        }
    }

    fn rx_count(&self) -> usize {
        let ufstat = self.regs.ufstat.get();

        if (ufstat & UFSTAT_RXFULL) != 0 {
            self.fifo_size
        } else {
            (ufstat & UFSTAT_RXMASK) as usize
        }
    }
}

impl core::fmt::Write for ZumaSimpleUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_buf(s.as_bytes());
        Ok(())
    }
}

impl UartDriver for ZumaSimpleUart {
    fn write_buf(&mut self, buf: &[u8]) {
        for &byte in buf {
            while !self.tx_ready() {
                spin_loop();
            }

            self.regs.utxh.set(byte as u32);
        }
    }

    fn drain_uart_rx(&mut self, buf: &mut [u8]) -> usize {
        let mut count = self.rx_count();
        let mut bytes_read = 0;

        while count > 0 && bytes_read < buf.len() {
            buf[bytes_read] = self.regs.urxh.get() as u8;
            bytes_read += 1;
            count -= 1;
        }

        bytes_read
    }
}

pub fn zuma_simple_uart_probe(
    dm: &mut DriverManager,
    d: DeviceDescriptor,
) -> Result<Arc<dyn Driver>> {
    match d {
        DeviceDescriptor::Fdt(fdt_node, flags) => {
            let region = fdt_node
                .reg()
                .ok_or(ProbeError::NoReg)?
                .next()
                .ok_or(ProbeError::NoReg)?;

            let size = region.size.ok_or(ProbeError::NoRegSize)?;

            let fifo_size = fdt_node
                .find_property("samsung,uart-fifosize")
                .or_else(|| fdt_node.find_property("samsung,fifo-size"))
                .map(|prop| prop.u32() as usize)
                .unwrap_or(64);

            let mut interrupts = fdt_node
                .interrupts()
                .ok_or(ProbeError::NoInterrupts)?
                .next()
                .ok_or(ProbeError::NoInterrupts)?;

            let interrupt_node = fdt_node
                .interrupt_parent()
                .ok_or(ProbeError::NoParentInterrupt)?
                .node;

            let interrupt_manager = dm
                .find_by_name(interrupt_node.name)
                .ok_or(ProbeError::Deferred)?
                .as_interrupt_manager()
                .ok_or(ProbeError::NotInterruptController)?;

            let uart_cdev = UART_CHAR_DEV.get().ok_or(ProbeError::Deferred)?;

            let mem =
                ArchImpl::kern_address_space()
                    .lock_save_irq()
                    .map_mmio(PhysMemoryRegion::new(
                        PA::from_value(region.address as usize),
                        size,
                    ))?;

            let interrupt_config = interrupt_manager.parse_fdt_interrupt_regs(&mut interrupts)?;

            let dev = interrupt_manager.claim_interrupt(interrupt_config, |claimed_interrupt| {
                Uart::new(
                    ZumaSimpleUart::new(mem, fifo_size),
                    claimed_interrupt,
                    fdt_node.name,
                )
            })?;

            uart_cdev.register_console(dev.clone(), flags.contains(FdtFlags::ACTIVE_CONSOLE))?;

            Ok(dev)
        }
    }
}

pub fn zuma_simple_uart_init(bus: &mut PlatformBus, _dm: &mut DriverManager) -> Result<()> {
    for compatible in ["google,gs101-uart", "google,zuma-uart"] {
        bus.register_platform_driver(
            DeviceMatchType::FdtCompatible(compatible),
            Box::new(zuma_simple_uart_probe),
        );
    }

    Ok(())
}

kernel_driver!(zuma_simple_uart_init);
