use bincode::Encode;
use std::error::Error;
use std::fmt::{Display, Formatter};

/// Common copper Error type.
#[derive(Debug)]
pub struct CuError {
    message: String,
    context: Option<String>,
}

impl Display for CuError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let context_str = match &self.context {
            Some(c) => format!("{}", c),
            None => "None".to_string(),
        };
        write!(f, "{}\n   context:{}", self.message, context_str)?;
        Ok(())
    }
}

impl Error for CuError {}

impl From<&str> for CuError {
    fn from(s: &str) -> CuError {
        CuError {
            message: s.to_string(),
            context: None,
        }
    }
}

impl From<String> for CuError {
    fn from(s: String) -> CuError {
        CuError {
            message: s,
            context: None,
        }
    }
}

impl CuError {
    pub fn add_context(mut self, context: &str) -> CuError {
        self.context = Some(context.into());
        self
    }
}

// Generic Result type for copper.
pub type CuResult<T> = Result<T, CuError>;

/// Defines a basic write, append only stream trait to be able to log or send serializable objects.
pub trait Stream: Sync + Send {
    fn log(&mut self, obj: &impl Encode) -> CuResult<()>;
}
