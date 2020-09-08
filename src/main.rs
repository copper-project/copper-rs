#![feature(min_const_generics)]
#![feature(wrapping_int_impl)]

extern crate nix;

use std::{thread, time};
use std::time::Instant;

use nix::unistd::{fork, ForkResult};

use crate::culistserver::{get_culist_source, get_base_channel_id, NB_LIST};
use crate::cutask::{CuTask, CuTaskRunner, get_ctr};
use crate::culist::{CuList, VALUE_SIZE, CuMsg};
use crate::curpc::sharedmem::clean_up_existing_shared_mem;
use std::num::Wrapping;
use arrayvec::ArrayString;


mod culist;
mod curpc;
mod culistserver;
mod common;
mod cutask;

fn main() {
    clean_up_existing_shared_mem(get_base_channel_id(), NB_LIST);
    let mut cuserver = get_culist_source();
    cuserver.start();

    match fork() {
        Ok(ForkResult::Parent { child, .. }) => {
            println!("Continuing execution in parent process, new child has pid: {}", child);
            thread::sleep(time::Duration::from_millis(1000));
        }
        Ok(ForkResult::Child) => {
            second_task();
        },
        Err(_) => println!("Fork failed"),
    }
    match fork() {
        Ok(ForkResult::Parent { child, .. }) => {
            println!("Continuing execution in parent process, new child has pid: {}", child);
        }
        Ok(ForkResult::Child) => {
            first_task();
        },
        Err(_) => println!("Fork failed"),
    }
    thread::sleep(time::Duration::from_millis(1000));

}

struct FibonacciTask {}

impl CuTask for FibonacciTask{
    fn incoming_msg(&self, msg: &mut CuMsg) {
        for i in 2..VALUE_SIZE-1 {
           msg[i] = (Wrapping(msg[i-1]) + Wrapping(msg[i-2])).0;
        }
    }
}


fn  first_task() {
    let ctr = &mut get_ctr();
    ctr.register_task(FibonacciTask{}, "first");
    ctr.register_task(FibonacciTask{}, "second");
    for _ in 0..10 {
        ctr.run_first(0);
    }
}

fn  second_task() {
    let ctr = &mut get_ctr();
    ctr.register_task(FibonacciTask{}, "first");
    ctr.register_task(FibonacciTask{}, "second");
    for _ in 0..10 {
        ctr.run_middle(1);
    }
}

