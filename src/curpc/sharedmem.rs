use shm::ffi::Ipc::{CREAT, EXCL, RMID};
use std::process::Command;

pub(crate) fn get_shared_mem(id: i32, size: usize) -> *mut i32
{
    let id = match shm::shmget!(id, 0o0666 | CREAT as i32 | EXCL as i32,
                                    size) {
        Some(id) => id,
        None => shm::shmget!( id, 0o0666, size).unwrap(),
    };

    let ptr = match shm::shmat!(id, std::ptr::null_mut(), 0) {
        Some(ptr) => ptr,
        None => panic!("Could not attach to shared memory")
    };
    return ptr;
}

pub(crate) fn clean_up_existing_shared_mem(base_id: i32, nb: usize) {
    for id in base_id..base_id + nb as i32 {
        let hex = &format!("0x{:x}", id);
        let status = Command::new("ipcrm")
            .arg("-M")
            .arg(hex)
            .status().unwrap();
        if status.success()
        {
            println!("seg id {} deleted", hex);
        } else {
            println!("seg id {} could not be deleted", hex);
        }
        // TODO: I don't understand why this does not work.
        //if shm::shmctl!(id, RMID, std::ptr::null_mut()) {
        // println!("seg id {:x} deleted", id);}
        //else {
        //    println!("seg id {:x} could not be deleted", id);
        //}
    }
}


pub(crate) fn round_to_page(size: usize) -> usize {
    assert_ne!(size, 0);
    let nb_pages = size % 4096 + 1;
    return nb_pages * 4096;
}
