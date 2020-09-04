#![feature(min_const_generics)]

extern crate nix;

use std::{thread, time};
use std::time::Instant;

use nix::unistd::{fork, ForkResult};

use crate::culistserver::get_culist_source;

mod culist;
mod curpc;
mod culistserver;
mod common;

fn main() {
    match fork() {
        Ok(ForkResult::Parent { child, .. }) => {
            println!("Continuing execution in parent process, new child has pid: {}", child);
            server();
        }
        Ok(ForkResult::Child) => {
            println!("I'm a new child process");
            client();
        },
        Err(_) => println!("Fork failed"),
    }



    thread::sleep(time::Duration::from_millis(1000));
}


fn server() {
    //let path = setup();
    //let origin_of_time = Instant::now();
    //let channel_id = get_channel_id();
    //println!("Channel ID: {:?}", channel_id);
    //let cu = get_culist_from_shared_mem(&channel_id);
    //emitter(origin_of_time, &path, cu);

    let mut cuserver = get_culist_source();
    cuserver.start();
    thread::sleep(time::Duration::from_millis(1000));

}

fn client() {
    //let path = setup();
    //let origin_of_time = Instant::now();
    //let channel_id = get_channel_id();
    //println!("Channel ID: {:?}", channel_id);
    //let cu = get_culist_from_shared_mem(&channel_id);
    //receiver(origin_of_time, &path, cu);
    let mut cuserver = get_culist_source();
    for i in 0..40 {
        let cu0id: i32;
        let cu1id: i32;
        {
            let cu0 = cuserver.get_fresh_cu();
            println!("{:?}", cu0);
            cu0id =  cu0.shared_mem_id;
        }
        {
            let cu1 = cuserver.get_fresh_cu();
            println!("{:?}", cu1);
            cu1id =  cu1.shared_mem_id;
        }
        cuserver.recycle_cu(cu0id);
        cuserver.recycle_cu(cu1id);
    }
    thread::sleep(time::Duration::from_millis(1000));
}



/*
fn get_ro_shared_mem(id: i32, size: usize) -> *const i32
{
    let id = match shm::shmget!(id, 0o0666 | shm::ffi::Ipc::CREAT as i32 | shm::ffi::Ipc::EXCL as i32,
                                    size) {
        Some(id) => id,
        None => shm::shmget!( id, 0o0666, size).unwrap(),
    };

    let ptr = match shm::shmat!(id, std::ptr::null_mut(), 0) {
        Some(ptr) => ptr as *const i32,
        None => panic!("Could not attach to shared memory")
    };
    return ptr;
}
*/



/*
fn emitter(timeref: Instant, outgoing_fifo: &PathBuf, cu: &mut Cu){
    let wave: [u8; 1] = [3];
    let mut file = OpenOptions::new().write(true).open(outgoing_fifo).unwrap();

    {
        let mut test = cu.lock.lock();

        // warn something is coming...
        file.write(&wave).unwrap();

        // here some business logic taking some time.
        thread::sleep(time::Duration::from_millis(100));

        cu.tov = Instant::now();
        *test = cu.tov;
        cu.value = (cu.tov.elapsed().as_nanos() % (255 as u128)) as i32;
    }
    println!("Value sent {}", cu.value)
    // unlock the receiver
}

fn receiver(timeref: Instant, incoming_fifo: &PathBuf, cu: &mut Cu) {
    let mut wave: &mut[u8; 1] = &mut [1];
    let mut file = OpenOptions::new().read(true).open(incoming_fifo).unwrap();
    match file.read(wave) {
        Ok(n) => {
            //println!("prepare to receive wave {:?}", wave[0]);
            let l = cu.lock.lock();
            println!("time: {:?}", cu.tov.elapsed());
            println!("time locked: {:?}", l.elapsed());
            println!("cu value: {:?}", cu.value);
        },
        Err(err) => println!("Error reading fifo: {}", err),
    }
}
*/
