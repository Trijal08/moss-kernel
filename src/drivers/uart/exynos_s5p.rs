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
// Constants aligned with your Linux defines
const S5PV210_UFSTAT_TXFULL: u32 = 1 << 24;
const S5PV210_UFSTAT_RXFULL: u32 = 1 << 8;
const S5PV210_UFSTAT_TXMASK: u32 = 255 << 16;
const S5PV210_UFSTAT_TXSHIFT: u32 = 16;
const S5PV210_UFSTAT_RXMASK: u32 = 255 << 0;

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
        // Comment out the reset and baud config for now.
        // Let's see if we can "hijack" the bootloader's settings.

        // self.reset_port();
        // self.configure_baud(115_200);

        // Just ensure interrupts are masked/unmasked as needed for your kernel
        self.regs.uintm.set(UINT_ALL_MASK);
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

    fn tx_fifo_full(&self) -> bool {
        // According to Linux: (1 << 24)
        (self.regs.ufstat.get() & S5PV210_UFSTAT_TXFULL) != 0
    }

    fn rx_fifo_count(&self) -> usize {
        let ufstat = self.regs.ufstat.get();
        // RXMASK is 255 << 0
        (ufstat & S5PV210_UFSTAT_RXMASK) as usize
    }

    fn tx_empty(&self) -> bool {
        (self.regs.utrstat.get() & UTRSTAT_TXE) != 0
    }

    fn write_byte(&mut self, byte: u8) {
        // 1. Handle the "Staircase" effect
        if byte == b'\n' {
            while self.tx_fifo_full() {
                core::hint::spin_loop();
            }
            self.regs.utxh.set(b'\r' as u32);
        }

        // 2. Wait for space in the FIFO
        while self.tx_fifo_full() {
            core::hint::spin_loop();
        }

        // 3. Write the actual byte
        self.regs.utxh.set(byte as u32);
    }

    fn ack_rx_interrupts(&mut self) {
        self.regs.utrstat.set(UTRSTAT_TIMEOUT); // Clear timeout bit
        self.regs.uintp.set(UINT_RXD_MASK);     // Clear pending bit
        // Crucially: You must read URXH to clear the hardware FIFO state
    }

    fn configure_baud(&mut self, baud: u32) {
        let baud = baud.max(1);

        // Calculate the total divisor in fixed-point (shifted by 4 bits for the fraction)
        // Formula: (Clock * 16 / (Baud * 16)) -> simplifies to Clock / Baud
        // But we want (Clock / (Baud * 16)) as the integer,
        // and the remainder mapped to a 4-bit fraction.

        let div = (self.uart_clock_hz as u64 * 10 / (baud as u64 * 16)) as u32;
        let ubrdiv = (div / 10).saturating_sub(1);

        // Fractional part: ((Clock / (Baud * 16)) - (Integer Part + 1)) * 16
        // Using integer-only math to get the .33 fraction mapped to 0-15:
        let ufracval = (((self.uart_clock_hz % (baud * 16)) * 16) / (baud * 16)) as u32;

        self.regs.ubrdiv.set(ubrdiv);
        self.regs.ufracval.set(ufracval);
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
        "google,gs101-uart",
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
