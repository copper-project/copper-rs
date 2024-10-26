use cu29_traits::CuResult;

pub struct InputPin {
    #[allow(dead_code)]
    pin_nb: u8,
}

pub fn get_pin(pin_nb: u8) -> CuResult<InputPin> {
    Ok(InputPin { pin_nb })
}
