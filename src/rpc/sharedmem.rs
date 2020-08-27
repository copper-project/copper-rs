pub(crate) fn get_shared_mem(id: i32, size: usize) -> *mut i32
{
    let id = match shm::shmget!(id, 0o0666 | shm::ffi::Ipc::CREAT as i32 | shm::ffi::Ipc::EXCL as i32,
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
pub(crate) fn round_to_page(size: usize) -> usize {
    assert_ne!(size, 0);
    let nb_pages = size % 4096 + 1;
    return nb_pages * 4096;
}
