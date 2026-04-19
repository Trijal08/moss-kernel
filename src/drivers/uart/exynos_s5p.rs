use crate::{
    arch::ArchImpl,
    drivers::{
        DeviceDescriptor, Driver, DriverManager,
        init::PlatformBus,
        probe::{DeviceMatchType, FdtFlags},
        uart::{UART_CHAR_DEV, Uart},
    },
    kernel_driver,
};
use tock_registers::interfaces::{Readable, Writeable, ReadWriteable};
use alloc::{boxed::Box, sync::Arc};
use core::hint::spin_loop;
use libkernel::{
    error::Result,
    memory::{
        address::{PA, VA},
        region::PhysMemoryRegion,

        proc_vm::{
            address_space::KernAddressSpace,
            address_space::VirtualMemory
        }
    },
};
use tock_registers::{
    register_bitfields, register_structs,
    registers::{ReadOnly, ReadWrite, WriteOnly},
};

use super::UartDriver;

const GS101_RX_FIFO_COUNT_MASK: u32 = 0xff;
const GS101_TX_FIFO_FULL_MASK: u32 = 1 << 24;
const UINT_RXD_MASK: u32 = 1 << 0;

register_bitfields![u32,
    UCON [
        ReceiveMode    OFFSET(0) NUMBITS(2) [
            Disable = 0,
            Polling = 1,
            Interrupt = 2
        ],
        TransmitMode   OFFSET(2) NUMBITS(2) [
            Disable = 0,
            Polling = 1,
            Interrupt = 2
        ],
        RxErrIrqEnable OFFSET(6) NUMBITS(1) [
            Disable = 0,
            Enable = 1
        ]
    ]
];

register_structs! {
    pub ExynosUartRegBank {
        (0x000 => ulcon: ReadWrite<u32>),
        (0x004 => ucon: ReadWrite<u32, UCON::Register>),
        (0x008 => ufcon: ReadWrite<u32>),
        (0x00c => umcon: ReadWrite<u32>),
        (0x010 => utrstat: ReadWrite<u32>), // Changed to ReadWrite to allow clearing timeouts
        (0x014 => uerstat: ReadOnly<u32>),
        (0x018 => ufstat: ReadOnly<u32>),
        (0x01c => umstat: ReadOnly<u32>),
        (0x020 => utxh: WriteOnly<u32>),
        (0x024 => urxh: ReadOnly<u32>),
        (0x028 => ubrdiv: ReadWrite<u32>),
        (0x02c => ufracval: ReadWrite<u32>),
        (0x030 => uintp: ReadWrite<u32>),   // Shifted from previous struct to match GS101 offsets
        (0x034 => uintsp: ReadWrite<u32>),
        (0x038 => uintm: ReadWrite<u32>),
        (0x03c => @END),
    }
}

pub struct ExynosS5P {
    regs: &'static mut ExynosUartRegBank,
}

unsafe impl Send for ExynosS5P {}
unsafe impl Sync for ExynosS5P {}

impl ExynosS5P {
    pub fn new(addr: VA) -> Self {
        let regs = unsafe { &mut *(addr.as_ptr_mut() as *mut ExynosUartRegBank) };

        let mut uart = Self { regs };
        uart.init();
        uart
    }

    fn init(&mut self) {
        // 1. Inherit the bootloader's clock/baud.
        // We only modify the modes needed for our kernel.
        self.regs.ucon.modify(
            UCON::ReceiveMode::Interrupt +
            UCON::TransmitMode::Polling +
            UCON::RxErrIrqEnable::Enable
        );

        // 2. Clear all pending interrupt bits at the hardware level
        self.regs.uintp.set(0xF);
        self.regs.uintsp.set(0xF);

        // 3. Unmask RX interrupt in the mask register
        let current_mask = self.regs.uintm.get();
        self.regs.uintm.set(current_mask & !UINT_RXD_MASK);

        // 4. Ensure FIFO is enabled (0x1)
        let current_ufcon = self.regs.ufcon.get();
        self.regs.ufcon.set(current_ufcon | 0x1);
    }

    fn ack_interrupts(&mut self) {
        // Clear Timeout status in UTRSTAT (Bit 3)
        let utrstat = self.regs.utrstat.get();
        self.regs.utrstat.set(utrstat | (1 << 3));

        // Clear Pending bits in UINTP and UINTSP
        self.regs.uintp.set(0xF);
        self.regs.uintsp.set(0xF);
    }
}

impl UartDriver for ExynosS5P {
    fn write_buf(&mut self, buf: &[u8]) {
        for &ch in buf {
            // 1. Check TX Full (Bit 24 for GS101)
            while (self.regs.ufstat.get() & GS101_TX_FIFO_FULL_MASK) != 0 {
                spin_loop();
            }

            // 2. 32-bit Write (Mandatory for reg_width == 4)
            self.regs.utxh.set(ch as u32);

            // 3. Error Check (Matches serial_err_check(uart, 1))
            // U-Boot checks mask 0x8 (Break Detect) on TX
            let _ = self.regs.uerstat.get();
        }
    }

    fn drain_uart_rx(&mut self, buf: &mut [u8]) -> usize {
        let mut bytes_read = 0;

        while bytes_read < buf.len() {
            // 1. Check Pending (ufstat & rx_fifo_count_mask)
            let ufstat = self.regs.ufstat.get();
            if (ufstat & GS101_RX_FIFO_COUNT_MASK) == 0 {
                break;
            }

            // 2. Error Check (Matches serial_err_check(uart, 0))
            let _err = self.regs.uerstat.get();

            // 3. 32-bit Read (Mandatory for GS101)
            let data = self.regs.urxh.get() & 0xff;
            let byte = data as u8;

            // --- THE CHERRY ON TOP ---
            // Direct kernel log for every byte received.
            // Using :? or {:c} to see the character clearly.
            log::info!("[UART RX] Received: 0x{:02x} ('{}')", byte, byte as char);

            buf[bytes_read] = byte;
            bytes_read += 1;
        }

        // 4. Acknowledge Interrupts
        self.ack_interrupts();

        bytes_read
    }
}

impl core::fmt::Write for ExynosS5P {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write_buf(s.as_bytes());
        Ok(())
    }
}

pub fn exynos_s5p_probe(dm: &mut DriverManager, d: DeviceDescriptor) -> Result<Arc<dyn Driver>> {
    match d {
        DeviceDescriptor::Fdt(fdt_node, flags) => {
            use libkernel::error::ProbeError::*;
            let mut regs_prop = fdt_node.reg().ok_or(NoReg)?;
            let region = regs_prop.next().ok_or(NoReg)?;
            let size = region.size.ok_or(NoRegSize)?;

            let mut interrupts = fdt_node.interrupts().ok_or(NoInterrupts)?.next().ok_or(NoInterrupts)?;
            let interrupt_node = fdt_node.interrupt_parent().ok_or(NoParentInterrupt)?.node;

            let interrupt_manager = dm.find_by_name(interrupt_node.name)
                .ok_or(Deferred)?
                .as_interrupt_manager()
                .ok_or(NotInterruptController)?;

            let uart_cdev = UART_CHAR_DEV.get().ok_or(Deferred)?;
            let interrupt_config = interrupt_manager.parse_fdt_interrupt_regs(&mut interrupts)?;

            let mem = ArchImpl::kern_address_space()
                .lock_save_irq()
                .map_mmio(PhysMemoryRegion::new(PA::from_value(region.address as usize), size))?;

            let dev = interrupt_manager.claim_interrupt(interrupt_config, |claimed_interrupt| {
                Uart::new(ExynosS5P::new(mem), claimed_interrupt, fdt_node.name)
            })?;

            uart_cdev.register_console(dev.clone(), flags.contains(FdtFlags::ACTIVE_CONSOLE))?;
            Ok(dev)
        }
    }
}

pub fn exynos_s5p_init(bus: &mut PlatformBus, _dm: &mut DriverManager) -> Result<()> {
    bus.register_platform_driver(
        DeviceMatchType::FdtCompatible("google,gs101-uart"),
        Box::new(exynos_s5p_probe),
    );
    Ok(())
}

kernel_driver!(exynos_s5p_init);
