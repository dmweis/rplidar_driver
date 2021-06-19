use std::io;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum RposError {
    /// The execution of operation failed
    #[error("operation failed: {}", description)]
    OperationFail { description: String },

    /// The execution of operation is timed out
    #[error("operation timeout")]
    OperationTimeout,

    /// The device doesn't support this operation
    #[error("operation not support")]
    OperationNotSupport,

    /// The decoding data is invalid according to current protocol
    #[error("protocol error: {}", description)]
    ProtocolError { description: String },

    /// The buffer is too small for message encoding
    #[error("buffer is too small for message encoding")]
    BufferTooSmall,

    #[error("Io error")]
    IoError(#[from] io::Error),

    #[error("Serial port error")]
    SerialPortError(#[from] serialport::Error),
}

pub type Result<T> = std::result::Result<T, RposError>;
