use cu29::prelude::*;

pub struct DynThreshold {}

impl Freezable for DynThreshold {}

impl<'cl> CuTask<'cl> for DynThreshold {
    type Input = input_msg!('cl, CuGstBuffer);
    type Output = output_msg!('cl, CuImage);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(DynThreshold {})
    }

    fn process(&mut self, _clock: &RobotClock, new_msg: Self::Input) -> CuResult<()> {
        let mut new_msg = new_msg;
        new_msg.sort();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn test_dynthreshold() {}
}
