// ![mod]

//! # RPOS Driver Infrastructure
//!
//! `rpos_drv` is a collection of structs and traits to build drivers for RPOS.

extern crate failure;

mod channel;
mod errors;
mod prelude;
mod ring_byte_buffer;

pub use self::channel::*;
pub use self::errors::*;
pub use self::prelude::*;
pub use self::ring_byte_buffer::RingByteBuffer;
