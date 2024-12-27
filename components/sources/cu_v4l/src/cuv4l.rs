use cu29::prelude::{CuBufferHandle, CuMemoryPool};
use std::convert::TryInto;
use std::ops::Deref;
use std::rc::Rc;
use std::time::Duration;
use std::{io, mem, sync::Arc};
use v4l::buffer::{Metadata, Type};
use v4l::device::Handle;
use v4l::io::traits::{CaptureStream, Stream};
use v4l::memory::Memory;
use v4l::v4l_sys::*;
use v4l::{v4l2, Device};

pub struct CuV4LStream<const BS: usize> {
    handle: Arc<Handle>,
    memory_pool: Rc<CuMemoryPool<BS>>,
    buf_type: Type,
    // Arena matching the vl42 metadata and the Copper Buffers
    arena: Vec<(Metadata, Option<CuBufferHandle<BS>>)>,
    arena_index: usize,
    timeout: Option<i32>,
    active: bool,
}

impl<const BS: usize> CuV4LStream<BS> {
    pub fn new(dev: &Device, buf_type: Type) -> io::Result<Self> {
        CuV4LStream::with_buffers(dev, buf_type, 4)
    }

    pub fn with_buffers(dev: &Device, buf_type: Type, buf_count: u32) -> io::Result<Self> {
        let mut memory_pool = CuMemoryPool::new(buf_count + 1, page_size::get()); // +1 to be able to queue one last buffer before zapping the first
        let mut arena = Vec::new();
        arena.resize(buf_count as usize, (Metadata::default(), None));

        let mut result = CuV4LStream {
            handle: dev.handle(),
            memory_pool: Rc::new(memory_pool),
            arena,
            arena_index: 0,
            buf_type,
            active: false,
            timeout: None,
        };
        result.allocate_request_buffers(buf_count)?;
        Ok(result)
    }

    /// Returns the raw device handle
    pub fn handle(&self) -> Arc<Handle> {
        self.handle.clone()
    }

    /// Sets a timeout of the v4l file handle.
    pub fn set_timeout(&mut self, duration: Duration) {
        self.timeout = Some(duration.as_millis().try_into().unwrap());
    }

    /// Clears the timeout of the v4l file handle.
    pub fn clear_timeout(&mut self) {
        self.timeout = None;
    }

    fn buffer_desc(&self) -> v4l2_buffer {
        v4l2_buffer {
            type_: self.buf_type as u32,
            memory: Memory::UserPtr as u32,
            ..unsafe { mem::zeroed() }
        }
    }

    // From v4l::Arena

    fn requestbuffers_desc(&self) -> v4l2_requestbuffers {
        v4l2_requestbuffers {
            type_: self.buf_type as u32,
            memory: Memory::UserPtr as u32,
            ..unsafe { mem::zeroed() }
        }
    }

    pub fn allocate_request_buffers(&mut self, count: u32) -> io::Result<u32> {
        // we need to get the maximum buffer size from the format first
        let mut v4l2_fmt = v4l2_format {
            type_: self.buf_type as u32,
            ..unsafe { mem::zeroed() }
        };
        unsafe {
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_G_FMT,
                &mut v4l2_fmt as *mut _ as *mut std::os::raw::c_void,
            )?;
        }

        let mut v4l2_reqbufs = v4l2_requestbuffers {
            count,
            ..self.requestbuffers_desc()
        };
        unsafe {
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_REQBUFS,
                &mut v4l2_reqbufs as *mut _ as *mut std::os::raw::c_void,
            )?;
        }

        Ok(v4l2_reqbufs.count)
    }

    pub fn release(&mut self) -> io::Result<()> {
        // free all buffers by requesting 0
        let mut v4l2_reqbufs = v4l2_requestbuffers {
            count: 0,
            ..self.requestbuffers_desc()
        };
        unsafe {
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_REQBUFS,
                &mut v4l2_reqbufs as *mut _ as *mut std::os::raw::c_void,
            )
        }
    }
}

impl<const BS: usize> Drop for CuV4LStream<BS> {
    fn drop(&mut self) {
        if let Err(e) = self.stop() {
            if let Some(code) = e.raw_os_error() {
                // ENODEV means the file descriptor wrapped in the handle became invalid, most
                // likely because the device was unplugged or the connection (USB, PCI, ..)
                // broke down. Handle this case gracefully by ignoring it.
                if code == 19 {
                    /* ignore */
                    return;
                }
            }

            panic!("{:?}", e)
        }
    }
}

impl<const BS: usize> Stream for CuV4LStream<BS> {
    type Item = CuBufferHandle<BS>;

    fn start(&mut self) -> io::Result<()> {
        // Enqueue all buffers once on stream start
        // -1 to leave one buffer unqueued as temporary storage
        for index in 0..self.arena.len() {
            println!("Queueing buffer {}", index);
            self.queue(index)?;
        }

        unsafe {
            let mut typ = self.buf_type as u32;
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_STREAMON,
                &mut typ as *mut _ as *mut std::os::raw::c_void,
            )?;
        }
        self.active = true;
        Ok(())
    }

    fn stop(&mut self) -> io::Result<()> {
        unsafe {
            let mut typ = self.buf_type as u32;
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_STREAMOFF,
                &mut typ as *mut _ as *mut std::os::raw::c_void,
            )?;
        }

        self.active = false;
        Ok(())
    }
}

impl<'a, const BS: usize> CaptureStream<'a> for CuV4LStream<BS> {
    fn queue(&mut self, index: usize) -> io::Result<()> {
        println!("queue: queueing buffer {}", index);

        let buffer_handle = CuMemoryPool::<BS>::allocate(&self.memory_pool).ok_or(
            io::Error::new(io::ErrorKind::Other, "Failed to allocate buffer"),
        )?;

        let buf: &[u8] = buffer_handle.deref();
        println!("queue: Buffer size: {}", buf.len());
        println!("queue: Buffer ptr: {:?}", buf.as_ptr());

        let mut v4l2_buf = v4l2_buffer {
            index: index as u32,
            m: v4l2_buffer__bindgen_ty_1 {
                userptr: buf.as_ptr() as std::os::raw::c_ulong,
            },
            length: buf.len() as u32,
            ..self.buffer_desc()
        };
        let result = unsafe {
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_QBUF,
                &mut v4l2_buf as *mut _ as *mut std::os::raw::c_void,
            )
        };
        if result.is_err() {
            let msg = format!("Failed to queue buffer: {:?}", result);
            println!("Error: {:?}", result);
        }
        self.arena[index] = (Metadata::default(), Some(buffer_handle));
        println!(
            "queue end ... : Buffer size: {}",
            self.arena[index].1.as_ref().unwrap().len()
        );
        println!(
            "queue end: Buffer ptr: {:?}",
            self.arena[index].1.as_ref().unwrap().as_ptr()
        );

        Ok(())
    }

    fn dequeue(&mut self) -> io::Result<usize> {
        let mut v4l2_buf = self.buffer_desc();

        if self.handle.poll(libc::POLLIN, self.timeout.unwrap_or(-1))? == 0 {
            // This condition can only happen if there was a timeout.
            // A timeout is only possible if the `timeout` value is non-zero, meaning we should
            // propagate it to the caller.
            return Err(io::Error::new(io::ErrorKind::TimedOut, "VIDIOC_DQBUF"));
        }

        unsafe {
            v4l2::ioctl(
                self.handle.fd(),
                v4l2::vidioc::VIDIOC_DQBUF,
                &mut v4l2_buf as *mut _ as *mut std::os::raw::c_void,
            )?;
        }
        self.arena_index = v4l2_buf.index as usize;
        println!("dequeue: dequeueing buffer {}", self.arena_index);
        println!(" length {}", v4l2_buf.length);
        println!(" bytesused {}", v4l2_buf.bytesused);
        println!(" timestamp {:?}", v4l2_buf.timestamp);
        println!(" sequence {}", v4l2_buf.sequence);
        println!(" field {}", v4l2_buf.field);

        self.arena[self.arena_index].0 = Metadata {
            bytesused: v4l2_buf.bytesused,
            flags: v4l2_buf.flags.into(),
            field: v4l2_buf.field,
            timestamp: v4l2_buf.timestamp.into(),
            sequence: v4l2_buf.sequence,
        };

        Ok(self.arena_index)
    }

    fn next(&'a mut self) -> io::Result<(&Self::Item, &Metadata)> {
        println!("next: active: {}", self.active);
        let dequeued_index = if !self.active {
            return Err(io::Error::new(
                io::ErrorKind::Other,
                "Stream is not active. Call start() first.",
            ));
        } else {
            self.queue(self.arena_index)?;
        };

        self.arena_index = self.dequeue()?;
        let buffer = self.arena[self.arena_index].1.as_ref().unwrap();
        let meta = &self.arena[self.arena_index].0;
        Ok((buffer, meta))
    }
}
