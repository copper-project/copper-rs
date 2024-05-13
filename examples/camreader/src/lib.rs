
struct CamReaderConfig {

}

struct CamReader {

}

pub fn build(config: CamReaderConfig) -> CamReader {
    CamReader {}
}


pub fn add(left: usize, right: usize) -> usize {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
