mod payload_clearing {
    use cu29::copperlist::CuListsManager;
    use cu29::prelude::*;

    gen_cumsgs!("tests/payload_clearing.ron");

    #[test]
    fn test_clear_default_tuple_vector_payload() {
        let mut msgs = cumsgs::CuStampedDataSet::default();
        msgs.0.0.clear_payload();
        assert!(msgs.0.0.payload().is_none());
    }

    #[test]
    fn test_drop_unused_tuple_vector_pool() {
        drop(CuListsManager::<cumsgs::CuStampedDataSet, 3>::new());
    }

    // Reproduces https://github.com/copper-project/copper-rs/issues/1395.
    #[test]
    fn test_clear_fresh_copperlist_tuple_vector_payload() {
        let mut lists = CuListsManager::<cumsgs::CuStampedDataSet, 3>::new();
        for _ in 0..3 {
            let list = lists.create().unwrap();
            assert!(list.msgs.0.0.payload().is_none());
            list.msgs.0.0.clear_payload();
            assert!(list.msgs.0.0.payload().is_none());
        }
    }

    #[test]
    fn test_reuse_preserves_tuple_vector_payload() {
        let mut lists = CuListsManager::<cumsgs::CuStampedDataSet, 1>::new();
        let list = lists.create().unwrap();
        list.msgs.0.0.set_payload((true, vec![1, 2, 3]));
        list.msgs.0.0.metadata.set_status("previous cycle");
        list.msgs.0.0.metadata.process_time.start = CuTime::from_nanos(42).into();
        let address = list as *const _;
        lists.pop().unwrap();

        let reused = lists.create().unwrap();
        assert_eq!(reused as *const _, address);
        assert_eq!(reused.id, 1);
        let (flag, values) = reused.msgs.0.0.payload().unwrap();
        assert!(*flag);
        assert_eq!(values, &vec![1, 2, 3]);
        assert!(reused.msgs.0.0.metadata.status_txt.0.is_empty());
        assert!(reused.msgs.0.0.metadata.process_time.start.is_none());
        reused.msgs.0.0.clear_payload();
    }

    #[cfg(any(feature = "async-cl-io", feature = "parallel-rt"))]
    #[test]
    fn test_clear_fresh_boxed_tuple_vector_payload() {
        #[cfg(feature = "parallel-rt")]
        for mut list in cu29::curuntime::allocate_boxed_copperlists::<cumsgs::CuStampedDataSet, 3>()
        {
            assert!(list.msgs.0.0.payload().is_none());
            list.msgs.0.0.clear_payload();
        }

        #[cfg(feature = "async-cl-io")]
        {
            let mut lists =
                cu29::curuntime::AsyncCopperListsManager::<cumsgs::CuStampedDataSet, 3>::new(None)
                    .unwrap();
            for _ in 0..3 {
                let list = lists.create().unwrap();
                assert!(list.msgs.0.0.payload().is_none());
                list.msgs.0.0.clear_payload();
                list.change_state(cu29::copperlist::CopperListState::Processing);
                let id = list.id;
                lists.end_of_processing(id).unwrap();
            }
        }
    }
}

mod large_payload_initialization {
    use cu29::copperlist::CuListsManager;
    use cu29::prelude::*;

    gen_cumsgs!("tests/large_payload_initialization.ron");

    #[test]
    fn test_large_payload_initializes_on_small_stack() {
        std::thread::Builder::new()
            .stack_size(128 * 1024)
            .spawn(|| {
                let mut lists = CuListsManager::<cumsgs::CuStampedDataSet, 2>::new();
                for _ in 0..2 {
                    let list = lists.create().unwrap();
                    assert!(list.msgs.0.0.0.payload().is_none());
                    assert!(list.msgs.0.0.1.payload().is_none());
                }
            })
            .unwrap()
            .join()
            .unwrap();
    }
}
