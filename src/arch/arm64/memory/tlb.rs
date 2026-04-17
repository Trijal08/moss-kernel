use core::arch::asm;

use libkernel::memory::paging::TLBInvalidator;

pub struct AllEl1TlbInvalidator;

impl AllEl1TlbInvalidator {
    pub fn new() -> Self {
        Self
    }
}

impl Drop for AllEl1TlbInvalidator {
    fn drop(&mut self) {
        unsafe {
            asm!(
                // Data Synchronization Barrier, Inner Shareable, write to
                // read/write.
                "dsb ishst",
                // Invalidate TLB by VA, for EL1, Inner Shareable
                "tlbi vmalle1is",
                // Data Synchronization Barrier, Inner Shareable.
                "dsb ish",
                // Instruction Synchronization Barrier.
                "isb",
                options(nostack, preserves_flags)
            );
        }
    }
}

impl TLBInvalidator for AllEl1TlbInvalidator {}

pub struct AllEl0TlbInvalidator;

impl AllEl0TlbInvalidator {
    pub fn new() -> Self {
        Self
    }
}

impl Drop for AllEl0TlbInvalidator {
    fn drop(&mut self) {
        unsafe {
            asm!(
                // Data Synchronization Barrier, Inner Shareable, write to
                // read/write.
                "dsb ishst",
                // Invalidate TLB by VA, for EL1, Inner Shareable
                "tlbi vmalle1is",
                // Data Synchronization Barrier, Inner Shareable.
                "dsb ish",
                // Instruction Synchronization Barrier.
                "isb",
                options(nostack, preserves_flags)
            );
        }
    }
}

impl TLBInvalidator for AllEl0TlbInvalidator {}
