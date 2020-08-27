extern crate nix;


use std::path::PathBuf;
use std::fs::OpenOptions;
use std::io::Write;
use std::io::Read;
use std::{thread, time};
use std::time::Instant;
use std::mem::size_of;

use nix::unistd;
use nix::sys::stat;
use nix::unistd::{fork, ForkResult};
use nix::Error::Sys;
use nix::errno::Errno::EEXIST;
use shm;
mod culist;
mod rpc;
use culist::Cu;

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
    let path = setup();
    let origin_of_time = Instant::now();
    let channel_id = get_channel_id();
    println!("Channel ID: {:?}", channel_id);
    let cu = get_culist_from_shared_mem(&channel_id);
    emitter(origin_of_time, &path, cu);
    thread::sleep(time::Duration::from_millis(1000));

}

fn client() {
    let path = setup();
    let origin_of_time = Instant::now();
    let channel_id = get_channel_id();
    println!("Channel ID: {:?}", channel_id);
    let cu = get_culist_from_shared_mem(&channel_id);
    receiver(origin_of_time, &path, cu);
    thread::sleep(time::Duration::from_millis(1000));
}

fn get_channel_id() -> i32 {
    return match shm::ftok!("./shared\0") {
        Some(id) => id as i32,
        None => -1,
    };
}

fn get_culist_from_shared_mem(id: &i32) -> &mut Cu {
    let cu: &mut Cu;
    let shared_mem_id = *id;
    unsafe {
        cu = &mut *(rpc::sharedmem::get_shared_mem(shared_mem_id, rpc::sharedmem::round_to_page(size_of::<Cu>())) as *mut Cu);
    }
    return cu;
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



fn setup() -> PathBuf {
    let fifo_path = PathBuf::from("./wave1");
    // create new fifo and give read, write and execute rights to the owner
    match unistd::mkfifo(&fifo_path, stat::Mode::S_IRWXU) {
        Ok(_) => println!("created {:?}", fifo_path),
        Err(Sys(EEXIST)) => println!("Fifo is already existing, reusing it."),
        Err(err) =>  println!("Error getting or creating Fifo {:?}", err),
    }
    return fifo_path;
}

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

