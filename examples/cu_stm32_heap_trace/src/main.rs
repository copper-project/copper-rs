#![no_std]
#![no_main]

extern crate alloc;

use alloc::sync::Arc;
use alloc::vec;
use core::alloc::{GlobalAlloc, Layout};
use core::hint::black_box;
use core::ptr::addr_of_mut;
use core::sync::atomic::{AtomicUsize, Ordering};
use cortex_m_rt::entry;
use cu29::prelude::*;
use embedded_alloc::LlffHeap;
use spin::Mutex;

// GDB reads r0..r3 at function entry using the ARM C ABI. Keep these calls
// observable without formatting, unwinding or allocating in the target.
#[inline(never)]
#[unsafe(no_mangle)]
extern "C" fn heap_event(kind: usize, ptr: usize, size: usize, used: usize) {
    black_box((kind, ptr, size, used));
}

static PEAK: AtomicUsize = AtomicUsize::new(0);
static HEAP: LlffHeap = LlffHeap::empty();
struct TraceAllocator;

#[global_allocator]
static ALLOCATOR: TraceAllocator = TraceAllocator;

// SAFETY: Forward each request and its original layout to the same heap.
unsafe impl GlobalAlloc for TraceAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { HEAP.alloc(layout) };
        if ptr.is_null() {
            heap_out_of_memory(layout.size());
        }
        let used = HEAP.used();
        PEAK.fetch_max(used, Ordering::Relaxed);
        heap_event(0, ptr as usize, layout.size(), used);
        ptr
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { HEAP.dealloc(ptr, layout) };
        heap_event(1, ptr as usize, layout.size(), HEAP.used());
    }
}

#[derive(Reflect)]
pub struct Source;
impl Freezable for Source {}
impl CuSrcTask for Source {
    type Resources<'r> = ();
    type Output<'m> = output_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, output: &mut Self::Output<'_>) -> CuResult<()> {
        output.set_payload(42);
        Ok(())
    }
}

#[derive(Reflect)]
pub struct Sink;
impl Freezable for Sink {}
impl CuSinkTask for Sink {
    type Resources<'r> = ();
    type Input<'m> = input_msg!(u32);
    fn new(_: Option<&ComponentConfig>, _: ()) -> CuResult<Self> {
        Ok(Self)
    }
    fn process(&mut self, _: &CuContext, input: &Self::Input<'_>) -> CuResult<()> {
        assert_eq!(input.payload(), Some(&42));
        Ok(())
    }
}

#[copper_runtime(config = "copperconfig.ron")]
struct App {}

// Separate entry points let the debugger test boot capacity without tracing each event.
#[unsafe(no_mangle)]
static mut heap_limit: usize = 96 * 1024;

#[inline(never)]
#[unsafe(no_mangle)]
extern "C" fn heap_capacity() -> usize {
    unsafe { core::ptr::read_volatile(&raw const heap_limit) }
}

#[inline(never)]
#[unsafe(no_mangle)]
extern "C" fn heap_out_of_memory(request: usize) {
    black_box(request);
}

#[inline(never)]
#[unsafe(no_mangle)]
extern "C" fn heap_complete(free: usize, largest: usize, peak: usize) {
    black_box((free, largest, peak));
}

fn largest_free_block() -> usize {
    // Probe the backing allocator directly so the measurement stays outside the trace.
    for size in (4..=HEAP.free()).rev().filter(|size| size % 4 == 0) {
        let layout = Layout::from_size_align(size, 4).unwrap();
        let ptr = unsafe { HEAP.alloc(layout) };
        if !ptr.is_null() {
            unsafe { HEAP.dealloc(ptr, layout) };
            return size;
        }
    }
    0
}

#[entry]
fn main() -> ! {
    const SIZE: usize = 96 * 1024;
    static mut ARENA: [u8; SIZE] = [0; SIZE];
    // SAFETY: This single-core firmware gives the static arena to HEAP once.
    unsafe { HEAP.init(addr_of_mut!(ARENA) as usize, heap_capacity().min(SIZE)) };
    let (clock, _controller) = RobotClock::mock();
    let writer = Arc::new(Mutex::new(NoopLogger::new()));
    heap_event(2, 0, 0, HEAP.used());
    let app = App::builder()
        .with_clock(clock)
        .with_logger::<NoopSectionStorage, NoopLogger>(writer)
        .build()
        .expect("build");
    heap_event(2, 1, 0, HEAP.used());
    let mut app = app.start().unwrap_or_else(|_| panic!("start"));
    heap_event(2, 2, 0, HEAP.used());
    app.run_one_iteration().expect("iteration");
    heap_complete(
        HEAP.free(),
        largest_free_block(),
        PEAK.load(Ordering::Relaxed),
    );
    heap_event(2, 3, 0, HEAP.used());
    loop {
        cortex_m::asm::wfi()
    }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    heap_event(3, 0, 0, HEAP.used());
    loop {
        cortex_m::asm::bkpt()
    }
}
