// Regression test for https://github.com/copper-project/copper-rs/issues/956
//
// `resources!` must not let the declaration order leak an internal
// borrow-evaluation constraint: an `Owned<G>` entry declared *after* one or
// more `Shared<_>` entries used to fail with E0502 because the generated
// `from_bindings` evaluated the shared `borrow` (immutable, `'r`-long) before
// the owned `take` (mutable). Both orderings must now compile.

use cu29::resources;

trait GenericTrait {}

struct GenericImpl;

impl GenericTrait for GenericImpl {}

struct SharedBus;
struct GlobalLog;

mod shared_first {
    use super::*;

    resources!(for<G> where G: GenericTrait + Send + Sync + 'static {
        bus => Shared<SharedBus>,
        note => Shared<String>,
        global => Shared<GlobalLog>,
        generic => Owned<G>,
    });
}

mod owned_first {
    use super::*;

    resources!(for<G> where G: GenericTrait + Send + Sync + 'static {
        generic => Owned<G>,
        bus => Shared<SharedBus>,
        note => Shared<String>,
        global => Shared<GlobalLog>,
    });
}

mod interleaved {
    use super::*;

    resources!(for<G> where G: GenericTrait + Send + Sync + 'static {
        bus => Shared<SharedBus>,
        generic => Owned<G>,
        global => Shared<GlobalLog>,
    });
}

fn main() {
    let _ = core::marker::PhantomData::<GenericImpl>;
}
