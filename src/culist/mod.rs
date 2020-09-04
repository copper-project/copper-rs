use spinlock::Spinlock;
use std::time::Instant;
use std::mem::size_of;

use crate::curpc;
use core::fmt;


pub struct CuList {
    pub shared_mem_id: i32,
    pub list_creation_time: Instant,
    pub tov: Instant,
    pub value: i32,
    lock: Spinlock<Instant>,
}

impl fmt::Debug for CuList {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "id: {:#x}", self.shared_mem_id);
        writeln!(f, "creation time: {:?}", self.list_creation_time);
        writeln!(f, "tov: {:?}", self.tov)
    }
}

impl CuList {
    pub(crate) fn reset(&mut self) {
        self.list_creation_time = Instant::now();
        self.value = -1;
    }
    pub(crate) fn init(&mut self, shared_mem_id: i32) {
        self.shared_mem_id = shared_mem_id;
        self.reset();
    }
}

pub(crate) fn get_culist_from_shared_mem(id: i32) -> &'static mut CuList {
    let cu: &mut CuList;
    let shared_mem_id = id;
    unsafe {
        cu = &mut *(curpc::sharedmem::get_shared_mem(shared_mem_id, curpc::sharedmem::round_to_page(size_of::<CuList>())) as *mut CuList);
    }
    return cu;
}
