use crate::{
    arch::ArchImpl,
    console::tty::{Termios2, TermiosControlFlags},
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
use tock_registers::{register_bitfields, register_structs, registers::ReadWrite};

use super::{UART_CHAR_DEV, Uart, UartDriver};

const UCON_DEFAULT: u32 = (1 << 9) | (1 << 8) | (1 << 2) | (1 << 0) | (1 << 7) | (1 << 6);
const ULCON_8N1: u32 = 0x3;
const ULCON_CS5: u32 = 0x0;
const ULCON_CS6: u32 = 0x1;
const ULCON_CS7: u32 = 0x2;
const ULCON_CS8: u32 = 0x3;
const ULCON_STOPB: u32 = 1 << 2;
const ULCON_PODD: u32 = 0x4 << 3;
const ULCON_PEVEN: u32 = 0x5 << 3;
const UFCON_FIFO_ENABLE: u32 = 1 << 0;
const UFCON_RESET_RX: u32 = 1 << 1;
const UFCON_RESET_TX: u32 = 1 << 2;
const UFCON_TXTRIG4: u32 = 1 << 8;
const UFCON_RXTRIG4: u32 = 1 << 4;
const UFCON_DEFAULT: u32 = UFCON_FIFO_ENABLE | UFCON_TXTRIG4 | UFCON_RXTRIG4;
const UTRSTAT_TIMEOUT: u32 = 1 << 3;
const UTRSTAT_TXE: u32 = 1 << 2;
const UINT_RXD_BIT: u32 = 0;
const UINT_RXD_MASK: u32 = 1 << UINT_RXD_BIT;
const UINT_ALL_MASK: u32 = 0x0f;
const UMCON_AFC: u32 = 1 << 4;
const FIFOCNT_MASK: u32 = 0xff;
const TXFULL_MASK: u32 = 1 << 24;
const RXFULL_MASK: u32 = 1 << 8;
const DEFAULT_UART_CLOCK_HZ: u32 = 24_576_000;

register_bitfields![u32,
    UERSTAT [
        OVERRUN OFFSET(0) NUMBITS(1) [],
        PARITY OFFSET(1) NUMBITS(1) [],
        FRAME OFFSET(2) NUMBITS(1) [],
        BREAK OFFSET(3) NUMBITS(1) []
    ]
];

register_structs! {
    ExynosUartRegBank {
        (0x000 => ulcon: ReadWrite<u32>),
        (0x004 => ucon: ReadWrite<u32>),
        (0x008 => ufcon: ReadWrite<u32>),
        (0x00c => umcon: ReadWrite<u32>),
        (0x010 => utrstat: ReadWrite<u32>),
        (0x014 => uerstat: ReadWrite<u32, UERSTAT::Register>),
        (0x018 => ufstat: ReadWrite<u32>),
        (0x01c => umstat: ReadWrite<u32>),
        (0x020 => utxh: ReadWrite<u32>),
        (0x024 => urxh: ReadWrite<u32>),
        (0x028 => ubrdiv: ReadWrite<u32>),
        (0x02c => ufracval: ReadWrite<u32>),
        (0x030 => uintp: ReadWrite<u32>),
        (0x034 => uintsp: ReadWrite<u32>),
        (0x038 => uintm: ReadWrite<u32>),
        (0x03c => @END),
    }
}

pub struct ExynosS5pUart {
    regs: &'static mut ExynosUartRegBank,
    fifo_size: usize,
    uart_clock_hz: u32,
}

unsafe impl Send for ExynosS5pUart {}
unsafe impl Sync for ExynosS5pUart {}

impl ExynosS5pUart {
    pub fn new(addr: VA, fifo_size: usize, uart_clock_hz: u32) -> Self {
        let regs = unsafe { &mut *(addr.as_ptr_mut() as *mut ExynosUartRegBank) };

        let mut uart = Self {
            regs,
            fifo_size,
            uart_clock_hz,
        };
        uart.init();
        uart
    }

    fn init(&mut self) {
        self.reset_port();
        self.regs.ulcon.set(ULCON_8N1);
        self.configure_baud(115_200);
        self.regs.umcon.set(0);
        self.unmask_rx_interrupt();
    }

    fn reset_port(&mut self) {
        self.regs.uintm.set(UINT_ALL_MASK);
        self.regs.uintp.set(UINT_ALL_MASK);
        self.regs.uintsp.set(UINT_ALL_MASK);

        self.regs.ucon.set(UCON_DEFAULT);
        self.regs.ufcon.set(UFCON_DEFAULT);
        self.regs
            .ufcon
            .set(UFCON_DEFAULT | UFCON_RESET_RX | UFCON_RESET_TX);
        self.regs.ufcon.set(UFCON_DEFAULT);
    }

    fn unmask_rx_interrupt(&mut self) {
        let mut mask = self.regs.uintm.get();
        mask &= !UINT_RXD_MASK;
        self.regs.uintm.set(mask);
    }

    fn rx_fifo_count(&self) -> usize {
        let ufstat = self.regs.ufstat.get();

        if (ufstat & RXFULL_MASK) != 0 {
            self.fifo_size
        } else {
            (ufstat & FIFOCNT_MASK) as usize
        }
    }

    fn tx_fifo_full(&self) -> bool {
        (self.regs.ufstat.get() & TXFULL_MASK) != 0
    }

    fn tx_empty(&self) -> bool {
        (self.regs.utrstat.get() & UTRSTAT_TXE) != 0
    }

    fn write_byte(&mut self, byte: u8) {
        while self.tx_fifo_full() {
            spin_loop();
        }

        self.regs.utxh.set(byte as u32);
    }

    fn ack_rx_interrupts(&mut self) {
        self.regs.utrstat.set(UTRSTAT_TIMEOUT);
        self.regs.uintp.set(UINT_RXD_MASK);
        self.unmask_rx_interrupt();
    }

    fn configure_baud(&mut self, baud: u32) {
        let baud = baud.max(1);
        let div = (self.uart_clock_hz / baud).max(16);
        let ubrdiv = div / 16 - 1;
        let frac = div & 0xf;

        self.regs.ubrdiv.set(ubrdiv);
        self.regs.ufracval.set(frac);
    }

    fn encode_ulcon(termios: &Termios2) -> u32 {
        let mut ulcon = if termios.c_cflag.contains(TermiosControlFlags::CS8) {
            ULCON_CS8
        } else if termios.c_cflag.bits() & TermiosControlFlags::CS8.bits() == 0x20 {
            ULCON_CS7
        } else if termios.c_cflag.bits() & TermiosControlFlags::CS8.bits() == 0x10 {
            ULCON_CS6
        } else {
            ULCON_CS5
        };

        if termios.c_cflag.contains(TermiosControlFlags::CSTOPB) {
            ulcon |= ULCON_STOPB;
        }

        if termios.c_cflag.contains(TermiosControlFlags::PARENB) {
            if termios.c_cflag.contains(TermiosControlFlags::PARODD) {
                ulcon |= ULCON_PODD;
            } else {
                ulcon |= ULCON_PEVEN;
            }
        }

        ulcon
    }
}

impl core::fmt::Write for ExynosS5pUart {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_buf(s.as_bytes());
        Ok(())
    }
}

impl UartDriver for ExynosS5pUart {
    fn write_buf(&mut self, buf: &[u8]) {
        for &byte in buf {
            self.write_byte(byte);
        }

        while !self.tx_empty() {
            spin_loop();
        }
    }

    fn drain_uart_rx(&mut self, buf: &mut [u8]) -> usize {
        let mut bytes_read = 0;
        let mut fifo_count = 0usize;
        let mut saw_activity = false;

        while bytes_read < buf.len() {
            if fifo_count == 0 {
                fifo_count = self.rx_fifo_count();
                if fifo_count == 0 {
                    break;
                }
            }

            fifo_count -= 1;
            saw_activity = true;

            let _ = self.regs.uerstat.get();
            buf[bytes_read] = self.regs.urxh.get() as u8;
            bytes_read += 1;
        }

        if saw_activity || self.rx_fifo_count() == 0 {
            self.ack_rx_interrupts();
        }

        bytes_read
    }

    fn configure(&mut self, termios: &Termios2) -> Result<()> {
        let baud = if termios.o_speed != 0 {
            termios.o_speed
        } else if termios.i_speed != 0 {
            termios.i_speed
        } else {
            115_200
        };

        self.regs.ulcon.set(Self::encode_ulcon(termios));
        self.configure_baud(baud);

        let mut umcon = self.regs.umcon.get() & !UMCON_AFC;
        if termios.c_cflag.contains(TermiosControlFlags::CREAD) {
            umcon &= !UMCON_AFC;
        }
        self.regs.umcon.set(umcon);

        Ok(())
    }
}

pub fn exynos_s5p_probe(dm: &mut DriverManager, d: DeviceDescriptor) -> Result<Arc<dyn Driver>> {
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

            let uart_clock_hz = fdt_node
                .clocks()
                .find(|clk| clk.name == Some("uart"))
                .and_then(|clk| clk.clock_frequency)
                .unwrap_or(DEFAULT_UART_CLOCK_HZ);

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
                    ExynosS5pUart::new(mem, fifo_size, uart_clock_hz),
                    claimed_interrupt,
                    fdt_node.name,
                )
            })?;

            uart_cdev.register_console(dev.clone(), flags.contains(FdtFlags::ACTIVE_CONSOLE))?;

            Ok(dev)
        }
    }
}

pub fn exynos_s5p_init(bus: &mut PlatformBus, _dm: &mut DriverManager) -> Result<()> {
    for compatible in [
        "samsung,exynos-uart",
        "samsung,s5pv210-uart",
        "samsung,exynos4210-uart",
        "samsung,exynos5433-uart",
        "samsung,exynos850-uart",
        // "google,gs101-uart",
        "samsung,exynos8895-uart",
    ] {
        bus.register_platform_driver(
            DeviceMatchType::FdtCompatible(compatible),
            Box::new(exynos_s5p_probe),
        );
    }

    Ok(())
}

kernel_driver!(exynos_s5p_init);
