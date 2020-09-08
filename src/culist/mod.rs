use spinlock::{Spinlock, SpinlockGuard};
use std::time::Instant;
use std::mem::size_of;

use crate::curpc;
use core::fmt;
use std::borrow::BorrowMut;

pub const VALUE_SIZE: usize = 1024 * 8;

pub(crate) type CuMsg = [u64; VALUE_SIZE];

pub struct CuList {
    pub shared_mem_id: i32,
    pub list_creation_time: Instant,
    pub tov: Instant,
    pub msg: Spinlock<CuMsg>,
}

impl fmt::Debug for CuList {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "id: {:#x}", self.shared_mem_id)?;
        writeln!(f, "creation time: {:?}", self.list_creation_time)?;
        writeln!(f, "tov: {:?}", self.tov)
    }
}

impl CuList {
    pub(crate) fn reset(&mut self) {
        self.list_creation_time = Instant::now();
        self.tov = self.list_creation_time;
        let mut l = self.msg.lock();
        let values = l.borrow_mut();
        values[0] = 1;
        values[1] = 1;
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
        let culist_size = curpc::sharedmem::round_to_page(size_of::<CuList>());
        // println!("Culist size {} bytes.", culist_size);
        cu = &mut *(curpc::sharedmem::get_shared_mem(shared_mem_id, culist_size) as *mut CuList);
    }
    return cu;
}
