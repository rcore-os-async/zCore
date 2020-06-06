use {
    super::super::*,
    alloc::{collections::VecDeque, vec::Vec},
    apic::{IoApic, LocalApic, XApic},
    core::convert::TryFrom,
    core::fmt::{Arguments, Write},
    core::time::Duration,
    rcore_console::{Console, ConsoleOnGraphic, DrawTarget, Pixel, Rgb888, Size},
    spin::Mutex,
    uart_16550::SerialPort,
    x86_64::{
        registers::control::{Cr2, Cr3, Cr3Flags, Cr4, Cr4Flags},
        structures::paging::{PageTableFlags as PTF, *},
    },
};

mod interrupt;
mod keyboard;

/// Page Table
#[repr(C)]
pub struct PageTableImpl {
    root_paddr: PhysAddr,
}

impl PageTableImpl {
    #[export_name = "hal_pt_current"]
    pub fn current() -> Self {
        PageTableImpl {
            root_paddr: Cr3::read().0.start_address().as_u64() as _,
        }
    }

    /// Create a new `PageTable`.
    #[allow(clippy::new_without_default)]
    #[export_name = "hal_pt_new"]
    pub fn new() -> Self {
        let root_frame = Frame::alloc().expect("failed to alloc frame");
        let root_vaddr = phys_to_virt(root_frame.paddr);
        let root = unsafe { &mut *(root_vaddr as *mut PageTable) };
        root.zero();
        map_kernel(root_vaddr as _, frame_to_page_table(Cr3::read().0) as _);
        trace!("create page table @ {:#x}", root_frame.paddr);
        PageTableImpl {
            root_paddr: root_frame.paddr,
        }
    }

    /// Map the page of `vaddr` to the frame of `paddr` with `flags`.
    #[export_name = "hal_pt_map"]
    pub fn map(
        &mut self,
        vaddr: x86_64::VirtAddr,
        paddr: x86_64::PhysAddr,
        flags: MMUFlags,
    ) -> Result<(), ()> {
        let mut pt = self.get();
        unsafe {
            pt.map_to_with_table_flags(
                Page::<Size4KiB>::from_start_address(vaddr).unwrap(),
                PhysFrame::from_start_address(paddr).unwrap(),
                flags.to_ptf(),
                PTF::PRESENT | PTF::WRITABLE | PTF::USER_ACCESSIBLE,
                &mut FrameAllocatorImpl,
            )
            .unwrap()
            .flush();
        };
        trace!("map: {:x?} -> {:x?}, flags={:?}", vaddr, paddr, flags);
        Ok(())
    }

    /// Unmap the page of `vaddr`.
    #[export_name = "hal_pt_unmap"]
    pub fn unmap(&mut self, vaddr: x86_64::VirtAddr) -> Result<(), ()> {
        let mut pt = self.get();
        let page = Page::<Size4KiB>::from_start_address(vaddr).unwrap();
        if let Ok((_, flush)) = pt.unmap(page) {
            flush.flush();
        }
        trace!("unmap: {:x?}", vaddr);
        Ok(())
    }

    /// Change the `flags` of the page of `vaddr`.
    #[export_name = "hal_pt_protect"]
    pub fn protect(&mut self, vaddr: x86_64::VirtAddr, flags: MMUFlags) -> Result<(), ()> {
        let mut pt = self.get();
        let page = Page::<Size4KiB>::from_start_address(vaddr).unwrap();
        if let Ok(flush) = unsafe { pt.update_flags(page, flags.to_ptf()) } {
            flush.flush();
        }
        trace!("protect: {:x?}, flags={:?}", vaddr, flags);
        Ok(())
    }

    /// Query the physical address which the page of `vaddr` maps to.
    #[export_name = "hal_pt_query"]
    pub fn query(&mut self, vaddr: x86_64::VirtAddr) -> Result<x86_64::PhysAddr, ()> {
        let pt = self.get();
        let ret = pt.translate_addr(vaddr).ok_or(());
        trace!("query: {:x?} => {:x?}", vaddr, ret);
        ret
    }

    fn get(&mut self) -> OffsetPageTable<'_> {
        let root_vaddr = phys_to_virt(self.root_paddr);
        let root = unsafe { &mut *(root_vaddr as *mut PageTable) };
        let offset = x86_64::VirtAddr::new(phys_to_virt(0) as u64);
        unsafe { OffsetPageTable::new(root, offset) }
    }
}

/// Set page table.
///
/// # Safety
/// This function will set CR3 to `vmtoken`.
pub unsafe fn set_page_table(vmtoken: usize) {
    let frame = PhysFrame::containing_address(x86_64::PhysAddr::new(vmtoken as _));
    if Cr3::read().0 == frame {
        return;
    }
    Cr3::write(frame, Cr3Flags::empty());
}

fn frame_to_page_table(frame: PhysFrame) -> *mut PageTable {
    let vaddr = phys_to_virt(frame.start_address().as_u64() as usize);
    vaddr as *mut PageTable
}

trait FlagsExt {
    fn to_ptf(self) -> PTF;
}

impl FlagsExt for MMUFlags {
    fn to_ptf(self) -> PTF {
        let mut flags = PTF::empty();
        if self.contains(MMUFlags::READ) {
            flags |= PTF::PRESENT;
        }
        if self.contains(MMUFlags::WRITE) {
            flags |= PTF::WRITABLE;
        }
        if !self.contains(MMUFlags::EXECUTE) {
            flags |= PTF::NO_EXECUTE;
        }
        if self.contains(MMUFlags::USER) {
            flags |= PTF::USER_ACCESSIBLE;
        }
        let cache_policy = (self.bits() & 3) as u32; // 最低三位用于储存缓存策略
        match CachePolicy::try_from(cache_policy) {
            Ok(CachePolicy::Cached) => {
                flags.remove(PTF::WRITE_THROUGH);
            }
            Ok(CachePolicy::Uncached) | Ok(CachePolicy::UncachedDevice) => {
                flags |= PTF::NO_CACHE | PTF::WRITE_THROUGH;
            }
            Ok(CachePolicy::WriteCombining) => {
                flags |= PTF::NO_CACHE | PTF::WRITE_THROUGH;
                // 当位于level=1时，页面更大，在1<<12位上（0x100）为1
                // 但是bitflags里面没有这一位。由页表自行管理标记位去吧
            }
            Err(_) => unreachable!("invalid cache policy"),
        }
        flags
    }
}

struct FrameAllocatorImpl;

unsafe impl FrameAllocator<Size4KiB> for FrameAllocatorImpl {
    fn allocate_frame(&mut self) -> Option<PhysFrame> {
        Frame::alloc().map(|f| {
            let paddr = x86_64::PhysAddr::new(f.paddr as u64);
            PhysFrame::from_start_address(paddr).unwrap()
        })
    }
}

impl FrameDeallocator<Size4KiB> for FrameAllocatorImpl {
    unsafe fn deallocate_frame(&mut self, frame: PhysFrame) {
        Frame {
            paddr: frame.start_address().as_u64() as usize,
        }
        .dealloc()
    }
}

static CONSOLE: Mutex<Option<ConsoleOnGraphic<Framebuffer>>> = Mutex::new(None);

struct Framebuffer {
    width: u32,
    height: u32,
    buf: &'static mut [u32],
}

impl DrawTarget<Rgb888> for Framebuffer {
    type Error = core::convert::Infallible;

    fn draw_pixel(&mut self, item: Pixel<Rgb888>) -> Result<(), Self::Error> {
        let idx = (item.0.x as u32 + item.0.y as u32 * self.width) as usize;
        self.buf[idx] = unsafe { core::mem::transmute(item.1) };
        Ok(())
    }

    fn size(&self) -> Size {
        Size::new(self.width, self.height)
    }
}

/// Initialize console on framebuffer.
pub fn init_framebuffer(width: u32, height: u32, paddr: PhysAddr) {
    let fb = Framebuffer {
        width,
        height,
        buf: unsafe {
            core::slice::from_raw_parts_mut(
                phys_to_virt(paddr) as *mut u32,
                (width * height) as usize,
            )
        },
    };
    let console = Console::on_frame_buffer(fb);
    *CONSOLE.lock() = Some(console);
}

static COM1: Mutex<SerialPort> = Mutex::new(unsafe { SerialPort::new(0x3F8) });

pub fn putfmt(fmt: Arguments) {
    COM1.lock().write_fmt(fmt).unwrap();
    if let Some(console) = CONSOLE.lock().as_mut() {
        console.write_fmt(fmt).unwrap();
    }
}

lazy_static! {
    static ref STDIN: Mutex<VecDeque<u8>> = Mutex::new(VecDeque::new());
    static ref STDIN_CALLBACK: Mutex<Vec<Box<dyn FnOnce() + Send + Sync>>> = Mutex::new(Vec::new());
}

/// Put a char by serial interrupt handler.
fn serial_put(mut x: u8) {
    if x == b'\r' {
        x = b'\n';
    }
    STDIN.lock().push_back(x);
    for callback in STDIN_CALLBACK.lock().drain(..) {
        callback();
    }
}

#[export_name = "hal_serial_set_callback"]
pub fn serial_set_callback(callback: Box<dyn FnOnce() + Send + Sync>) {
    STDIN_CALLBACK.lock().push(callback);
}

#[export_name = "hal_serial_read"]
pub fn serial_read(buf: &mut [u8]) -> usize {
    let mut stdin = STDIN.lock();
    let len = stdin.len().min(buf.len());
    for c in &mut buf[..len] {
        *c = stdin.pop_front().unwrap();
    }
    len
}

#[export_name = "hal_serial_write"]
pub fn serial_write(s: &str) {
    putfmt(format_args!("{}", s));
}

/// Get TSC frequency.
///
/// WARN: This will be very slow on virtual machine since it uses CPUID instruction.
fn tsc_frequency() -> u16 {
    const DEFAULT: u16 = 2600;
    if let Some(info) = raw_cpuid::CpuId::new().get_processor_frequency_info() {
        let f = info.processor_base_frequency();
        return if f == 0 { DEFAULT } else { f };
    }
    // FIXME: QEMU, AMD, VirtualBox
    DEFAULT
}

#[export_name = "hal_timer_now"]
pub fn timer_now() -> Duration {
    let tsc = unsafe { core::arch::x86_64::_rdtsc() };
    Duration::from_nanos(tsc * 1000 / unsafe { TSC_FREQUENCY } as u64)
}

fn timer_init() {
    let mut lapic = unsafe { XApic::new(phys_to_virt(LAPIC_ADDR)) };
    lapic.cpu_init();
}

#[export_name = "hal_irq_enable"]
pub fn irq_enable(irq: u8) {
    let mut ioapic = unsafe { IoApic::new(phys_to_virt(IOAPIC_ADDR)) };
    ioapic.enable(irq, 0);
}

#[export_name = "hal_irq_disable"]
pub fn irq_disable(irq: u8) {
    let mut ioapic = unsafe { IoApic::new(phys_to_virt(IOAPIC_ADDR)) };
    ioapic.disable(irq);
}

const LAPIC_ADDR: usize = 0xfee0_0000;
const IOAPIC_ADDR: usize = 0xfec0_0000;

#[export_name = "hal_vdso_constants"]
fn vdso_constants() -> VdsoConstants {
    let tsc_frequency = unsafe { TSC_FREQUENCY };
    VdsoConstants {
        max_num_cpus: 1,
        features: Features {
            cpu: 0,
            hw_breakpoint_count: 0,
            hw_watchpoint_count: 0,
        },
        dcache_line_size: 0,
        icache_line_size: 0,
        ticks_per_second: tsc_frequency as u64 * 1_000_000,
        ticks_to_mono_numerator: 1000,
        ticks_to_mono_denominator: tsc_frequency as u32,
        physmem: 0,
        buildid: Default::default(),
    }
}

/// Initialize the HAL.
pub fn init(config: Config) {
    timer_init();
    interrupt::init();
    COM1.lock().init();
    unsafe {
        // enable global page
        Cr4::update(|f| f.insert(Cr4Flags::PAGE_GLOBAL));
        // store config
        CONFIG = config;
        // get tsc frequency
        TSC_FREQUENCY = tsc_frequency();
    }
}

pub fn init_ap() {
    timer_init();
    unsafe {
        Cr4::update(|f| f.insert(Cr4Flags::PAGE_GLOBAL));
    }
}

pub fn wakeup_all_ap() {
    interrupt::wakeup_all_ap();
}

/// Configuration of HAL.
pub struct Config {
    pub acpi_rsdp: u64,
    pub smbios: u64,
}

#[export_name = "fetch_fault_vaddr"]
pub fn fetch_fault_vaddr() -> VirtAddr {
    Cr2::read().as_u64() as _
}

/// Get physical address of `acpi_rsdp` and `smbios` on x86_64.
#[export_name = "hal_pc_firmware_tables"]
pub fn pc_firmware_tables() -> (u64, u64) {
    unsafe { (CONFIG.acpi_rsdp, CONFIG.smbios) }
}

// AP trampoline memory layout:
// 0x5000: trampoline entry
// 0x6000: real mode code
// 0x7010: page table address
// 0x7020: kernel stack base
// 0x7030: kernel stack size for each core
// 0x7040: core ticket
// 0x7052: GDT pointer
// 0x7060: debug stepper
// 0x7090: tmp sp
// 0x70A0: jump addr seg
// 0x70A4: jump addr seg
// 0x8000: long mode code
// 0x9000: GDT
global_asm!("
  .section .data, \"ax\", %progbits
  .code16
_ap_boot_entry:
  ljmp $0,$0x6000
_ap_boot_entry_end:

  .section .data, \"ax\", %progbits
  .code16
_ap_boot_16:
  cli
  xorw %ax, %ax
  movw %ax, %ds
  movw %ax, %es
  movw %ax, %ss
  movw %ax, %fs
  movw %ax, %gs

  movw $0x7090, %sp
  cld

  movl $0x0, [0x7060]

  # Disable IRQs
  mov $0xFF, %al
  outb %al, $0xA1
  outb %al, $0x21

  nop
  nop

  # Enter long mode
  mov $0b10100000, %eax
  movl %eax, %cr4

  movl $0x1, [0x7060]

  # Setup paging
  movl [0x7010], %eax
  movl %eax, %cr3

  movl $0x2, [0x7060]

  # Set LME
  movl 0xC0000080, %ecx
  rdmsr
  orl 1 << 8, %eax
  wrmsr

  movl $0x3, [0x7060]

  # 64-bit submode gdt
  lgdt [0x7052]

  movl $0x4, [0x7060]

  movl $0x8, [0x70A0]
  movl $0x8000, [0x70A4]

  movl $0x5, [0x7060]

  # Activate long mode
  movl %cr0, %eax
  orl $0x80000001, %eax
  movl %eax, %cr0

  ljmp *(0x70A0)
_ap_boot_16_end:

.code64
_ap_boot_64:
  # Setup DS
  nop
  movq $0x10, %rax
  movq %rax, %ds
  movq %rax, %es
  movq %rax, %fs
  movq %rax, %gs
  movq %rax, %ss

  movl $0x7, [0x7060]

  # Setup stack
  lock; xaddq %rax, [0x7040]
  addq $1, %rax
  movq [0x7030], %rbx
  mulq %rbx
  shlq $12, %rax
  movq [0x7020], %rbx
  addq %rbx, %rax
  movq  %rax, %rsp

  movl $0x8, [0x7060]

  call _ap_start

_ap_boot_64_end:

GDT64:                                # Global Descriptor Table (64-bit).
    .quad 0x0000000000000000          # Null Descriptor - should be present.
    .quad 0x00209A0000000000          # 64-bit code descriptor (exec/read).
    .quad 0x0000920000000000          # 64-bit data descriptor (read/write).

GDT64_end:
");

pub fn start_aps() {
    extern "C" {
        fn _ap_boot_entry();
        fn _ap_boot_entry_end();

        fn _ap_boot_16();
        fn _ap_boot_16_end();

        fn _ap_boot_64();
        fn _ap_boot_64_end();

        fn GDT64();
        fn GDT64_end();
    }

    // Map zero page
    let mut pt = PageTableImpl::current();
    let mut pt = pt.get();
    let query = pt.translate(x86_64::VirtAddr::new(0));
    info!("Original: {:?}", query);
    unsafe {
        let page = Page::<Size2MiB>::from_start_address(x86_64::VirtAddr::new(0)).unwrap();
        pt.update_flags(page, (MMUFlags::READ | MMUFlags::WRITE | MMUFlags::EXECUTE).to_ptf() | PTF::HUGE_PAGE).unwrap().flush();
    }

    info!("0x6000: {:?}", pt.translate(x86_64::VirtAddr::new(0x6000)));

    let len_entry = _ap_boot_entry_end as usize - _ap_boot_entry as usize;
    let start_entry = _ap_boot_entry as *const u8;
    let len_16 = _ap_boot_16_end as usize - _ap_boot_16 as usize;
    let start_16 = _ap_boot_16 as *const u8;
    let len_64 = _ap_boot_64_end as usize - _ap_boot_64 as usize;
    let start_64 = _ap_boot_64 as *const u8;
    let len_gdt= GDT64_end as usize - GDT64 as usize;
    let start_gdt= GDT64 as *const u8;
    unsafe {
        info!("Copying trampoline entry...");
        core::ptr::copy_nonoverlapping(start_entry, 0x5000 as *mut u8, len_entry);
        info!("Copying real mode code...");
        core::ptr::copy_nonoverlapping(start_16, 0x6000 as *mut u8, len_16);
        info!("Copying long mode code...");
        core::ptr::copy_nonoverlapping(start_64, 0x8000 as *mut u8, len_64);
        info!("Copying GDT content...");
        core::ptr::copy_nonoverlapping(start_gdt, 0x9000 as *mut u8, len_gdt);
    }

    let cr3: u64;
    unsafe { llvm_asm!("mov %cr3, $0" : "=r" (cr3)) };
    unsafe {
        info!("Writing CR3 template...: 0x{:016X}", cr3);
        core::ptr::write(0x7010 as *mut u64, cr3);
        info!("Writing kernel stack base...");
        core::ptr::write(0x7020 as *mut u64, 0xFFFFFF8000000000);
        info!("Writing kernel stack size per core...");
        core::ptr::write( 0x7030 as *mut u64, 128);
        info!("Resetting core lottery...");
        core::ptr::write( 0x7030 as *mut u64, 128);

        info!("Writing GDT size...: {}", len_gdt-1);
        core::ptr::write(0x7052 as *mut u16, (len_gdt-1) as u16);
        info!("Writing GDT addr...:");
        core::ptr::write(0x7054 as *mut u32, 0x9000);
    }

    info!("Sending ISS sequence...");
    interrupt::wakeup_all_ap();
}

static mut CONFIG: Config = Config {
    acpi_rsdp: 0,
    smbios: 0,
};

static mut TSC_FREQUENCY: u16 = 2600;
