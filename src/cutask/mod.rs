use std::path::PathBuf;
use crate::culist::{CuList, CuMsg};
use crate::culistserver::{get_culist_source, CuListSource};
use std::fs::{OpenOptions, File};
use std::io::Write;
use std::io::Read;
use std::borrow::BorrowMut;

pub struct CuTaskRunner {
    cuserver: CuListSource,
    tasks: Vec<Box<dyn CuTask>>,
    fifos: Vec<File>,
}


const CUBASE: &'static str = "./cubase";

impl CuTaskRunner {
    pub fn register_task<T: 'static + CuTask>(&mut self, t: T, name: &str) {
        println!("Task Runner, registering task #{} , with name {}", self.tasks.len(), name);
        self.tasks.push(Box::new(t));
        let mut fifo_path = PathBuf::from(CUBASE);
        fifo_path.push(name);
        let path = fifo_path.as_path();

        let mut f = match OpenOptions::new().read(true).write(true).open(path) {
            Err(why) => panic!("couldn't open {:?}: {}", path, why),
            Ok(file) => file,
        };
        self.fifos.push(f);
    }

    pub fn run_first(&mut self, task_nb:usize) {
        println!("Task Runner, first task, outgoing fifo {:?}", task_nb+1);

        let culist_id: i32;
        // this is the first task do it needs a fresh culist from the cuserver.
        let culist = self.cuserver.get_fresh_culist();
        // lock first the culist
        {
            let mut msg = culist.msg.lock();
        // start to warn downstream that a culist is coming
        culist_id = culist.shared_mem_id;
        println!("First task: put {:?} on fifo {}.", culist_id, task_nb+1);
        self.fifos[task_nb+1].write(unsafe {&std::mem::transmute::<i64, [u8; 8]>(culist_id as i64)}).unwrap();
        self.fifos[task_nb+1].flush();
        // execute locally the task
        self.tasks[task_nb].incoming_msg(msg.borrow_mut());}
        println!("First task: done on {:?}.", culist_id);
        // release the client. (end of scope for the lock)
    }

    pub fn run_middle(&mut self, task_nb:usize) {
        let culist_id: i32;
        let id_as_bytes: &mut[u8; 8] = &mut[0_u8,0,0,0,0,0,0,0];

        println!("Second Task: Block on reading on fifo {:?}", task_nb);
        match self.fifos[task_nb].read(id_as_bytes) {
            Ok(n) => {
                if n!= 8 {
                    panic!("Read a channel id of {} bytes instead of 4.", n);
                }
                culist_id = unsafe { std::mem::transmute::<[u8;8], i64>(*id_as_bytes)} as i32;
                {
                    println!("Second Task: Preparing to receive culist id {:?}", culist_id);
                    let culist = self.cuserver.get_culist(culist_id);
                    // spinlock on the list
                    let mut msg = culist.msg.lock();

                    println!("Second task: processing id {:?}", culist.shared_mem_id);
                    self.tasks[task_nb].incoming_msg(msg.borrow_mut());
                }
                println!("Second task done on {:?}.", culist_id);
                // TEMP
                self.cuserver.recycle_culist(culist_id);
                //println!("time: {:?}", cu.tov.elapsed());
                //println!("time locked: {:?}", l.elapsed());
                //println!("cu value: {:?}", cu.value);
            },
            Err(err) => println!("Error reading fifo: {}", err),
        }
    }


    pub fn run_last(&mut self, task_nb:usize) {
        //self.cuserver.recycle_culist(culist_id);
    }
}

pub fn get_ctr() -> CuTaskRunner {
    return CuTaskRunner { cuserver: get_culist_source(), tasks: vec![], fifos: vec![]};
}

pub trait CuTask {
    fn incoming_msg(&self, msg: &mut CuMsg);
}
