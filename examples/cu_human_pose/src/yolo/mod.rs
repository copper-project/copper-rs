pub mod model;
pub mod postprocess;
pub mod preprocess;

pub use model::{Multiples, YoloV8Pose};
pub use postprocess::process_predictions;
pub use preprocess::preprocess_image;
