// Linked only into the measurement checkout. Per-thread samples reach disk after shutdown.
use std::cell::{Cell, RefCell};
use std::io::Write;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

static START: OnceLock<Instant> = OnceLock::new();
static FINISHED: Mutex<Vec<Batch>> = Mutex::new(Vec::new());
struct Event { begin: u64, end: u64, kind: &'static str, trigger: &'static str, slab: usize, offset: usize, bytes: usize }
struct Batch { name: String, events: Vec<Event> }
struct Log(Option<Batch>);
impl Drop for Log {
    fn drop(&mut self) { if let Some(batch) = self.0.take() { FINISHED.lock().unwrap().push(batch); } }
}
thread_local! {
    static LOG: RefCell<Log> = RefCell::new(Log(Some(Batch { name: std::thread::current().name().unwrap_or("unnamed").into(), events: Vec::with_capacity(8192) })));
    static TRIGGER: Cell<&'static str> = const { Cell::new("startup") };
}
fn now() -> u64 { START.get_or_init(Instant::now).elapsed().as_nanos() as u64 }
pub struct Span(Event);
pub fn span(kind: &'static str, slab: usize, offset: usize, bytes: usize) -> Span {
    Span(Event { begin: now(), end: 0, kind, trigger: TRIGGER.get(), slab, offset, bytes })
}
impl Drop for Span {
    fn drop(&mut self) {
        let e=&mut self.0;
        e.end=now();
        let event=Event { ..*e };
        LOG.with(|log| { if let Some(batch)=log.borrow_mut().0.as_mut() { batch.events.push(event); } });
    }
}
pub struct Trigger(&'static str);
pub fn trigger(name: &'static str) -> Trigger { Trigger(TRIGGER.replace(name)) }
impl Drop for Trigger { fn drop(&mut self){ TRIGGER.set(self.0); } }
pub fn dump(path: &std::path::Path) {
    LOG.with(|log| { if let Some(batch)=log.borrow_mut().0.take(){FINISHED.lock().unwrap().push(batch);} });
    let mut out=std::io::BufWriter::new(std::fs::File::create(path).unwrap());
    writeln!(out,"begin_ns,end_ns,thread,kind,trigger,slab,offset,bytes").unwrap();
    for batch in FINISHED.lock().unwrap().drain(..) {
        for e in batch.events {
            writeln!(out,"{},{},{},{},{},{},{},{}",e.begin,e.end,batch.name,e.kind,e.trigger,e.slab,e.offset,e.bytes).unwrap();
        }
    }
}
