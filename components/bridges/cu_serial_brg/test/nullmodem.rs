use serialport::{SerialPort, TTYPort};
use std::{
    fs,
    io::{Read, Write},
    os::unix::fs::symlink,
    path::Path,
    thread,
};

/// use /tmp/... for the paths for example
fn create_null_serials(alice_dev_name: &str, bob_dev_name: &str) -> anyhow::Result<()> {
    let (mut a_m, mut a_s) = TTYPort::pair()?;
    let (mut b_m, mut b_s) = TTYPort::pair()?;

    let a = a_s.name().unwrap();
    let b = b_s.name().unwrap();

    let l0 = Path::new(alice_dev_name);
    let l1 = Path::new(bob_dev_name);
    let _ = fs::remove_file(l0);
    let _ = fs::remove_file(l1);
    symlink(&a, l0)?;
    symlink(&b, l1)?;

    let mut ar = a_m.try_clone()?.take_reader()?;
    let mut aw = a_m.take_writer()?;
    let mut br = b_m.try_clone()?.take_reader()?;
    let mut bw = b_m.take_writer()?;

    thread::spawn(move || {
        let mut buf = [0u8; 8192];
        while let Ok(n) = ar.read(&mut buf) {
            if n == 0 {
                break;
            }
            let _ = bw.write_all(&buf[..n]);
            let _ = bw.flush();
        }
    });
    let mut buf = [0u8; 8192];
    while let Ok(n) = br.read(&mut buf) {
        if n == 0 {
            break;
        }
        let _ = aw.write_all(&buf[..n]);
        let _ = aw.flush();
    }
    Ok(())
}
