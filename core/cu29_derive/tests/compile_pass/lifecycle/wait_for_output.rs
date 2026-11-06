//! Recording barriers keep normal and simulation applications Running.

use cu29::prelude::*;

fn wait_normal<A: CuStdApplication + CuApplicationOutput>(
    mut app: CuStdAppLifecycle<A, Running>,
) -> CuResult<()> {
    app.run_one_iteration()?;
    app.wait_for_output()?;
    app.run_one_iteration()?;
    Ok(())
}

fn wait_sim<A>(mut app: CuStdSimAppLifecycle<A, Running>) -> CuResult<()>
where
    A: CuSimApplication<memmap::MmapSectionStorage, UnifiedLoggerWrite> + CuApplicationOutput,
{
    app.wait_for_output()?;
    app.run_one_iteration(&mut |_| cu29::simulation::SimOverride::ExecuteByRuntime)?;
    Ok(())
}
