use serialport::SerialPort;

use super::prelude::*;
use super::ring_byte_buffer::RingByteBuffer;
use std::time::{Duration, Instant};

const DEFAULT_CHANNEL_READ_BUFFER_SIZE: usize = 1024;

/// Channel encode and decode message with protocol, and send and receive bytes via stream
///
/// # Examples
/// ```ignore
/// let mut channel = Channel::new(
///     RplidarProtocol::new(),
///     serial_port
/// );
///
/// channel.write(&Message::new(1)).unwrap();
/// ```
pub struct Channel<P, T> {
    protocol: P,
    port: T,
    read_buffer: RingByteBuffer,
}

impl<P, T> Channel<P, T>
where
    P: ProtocolDecoder + ProtocolEncoder,
    T: SerialPort,
{
    /// Create a new `Channel` to read and write messages
    ///
    /// # Example
    /// ```ignore
    /// let channel = Channel::new(
    ///     RplidarProtocol::new(),
    ///     serial_port
    /// );
    /// ```
    pub fn new(protocol: P, port: T) -> Self {
        Channel::with_read_buffer_size(protocol, port, DEFAULT_CHANNEL_READ_BUFFER_SIZE)
    }

    /// Create a new `Channel` with non-default ring buffer capacity
    ///
    /// # Example
    /// ```ignore
    /// let channel = Channel::with_read_buffer_size(
    ///     RplidarProtocol::new(),
    ///     serial_port,
    ///     100000 as usize
    /// );
    /// ```
    pub fn with_read_buffer_size(protocol: P, port: T, read_buffer_size: usize) -> Self {
        let mut chn = Channel {
            protocol,
            port,
            read_buffer: RingByteBuffer::with_capacity(read_buffer_size),
        };

        chn.reset();

        chn
    }

    /// Reset the channel status
    /// This function is usually used to reset protocol encoder and decoder when meet communication error
    ///Dennis Ritchiereset(); }
    /// }
    /// ```
    pub fn reset(&mut self) {
        self.protocol.reset_encoder();
        self.protocol.reset_decoder();
    }

    /// Read message from channel
    ///
    /// # Example
    /// ```ignore
    /// if let Some(msg) = channel.read().unwrap() {
    ///     println!("{:?}", msg);
    /// }
    /// ```
    pub fn read(&mut self) -> Result<Option<Message>> {
        loop {
            self.read_buffer.read_from(&mut self.port)?;

            let (decoded, msg) = self
                .protocol
                .decode(self.read_buffer.current_read_slice())?;
            self.read_buffer.skip_bytes(decoded);

            if decoded == 0 {
                return Ok(Option::None);
            }

            if msg.is_some() {
                return Ok(msg);
            }
        }
    }

    /// Read message until timeout
    ///
    /// # Example
    /// ```ignore
    /// channel.read_until(Duration::from_secs(1));
    /// ```
    pub fn read_until(&mut self, timeout: Duration) -> Result<Option<Message>> {
        let start = Instant::now();

        while Instant::now() - start < timeout {
            if let Some(msg) = self.read()? {
                return Ok(Some(msg));
            }
        }

        Err(RposError::OperationTimeout.into())
    }

    /// Write message to channel
    ///
    /// # Example
    /// ```ignore
    /// channel.write(&Message::new(1)).unwrap();
    /// ```
    pub fn write(&mut self, msg: &Message) -> Result<usize> {
        let written = self.protocol.write_to(msg, &mut self.port)?;
        self.port.flush()?;
        Ok(written)
    }

    /// Send a request to channel and wait for response
    ///
    /// # Example
    /// ```ignore
    /// let resp = channel.invoke(&Message::new(1), Duration::from_secs(1));
    /// ```
    pub fn invoke(&mut self, request: &Message, timeout: Duration) -> Result<Option<Message>> {
        self.write(request)?;
        self.read_until(timeout)
    }

    /// Set data terminal ready on port
    ///
    /// Useful for stopping motors on the RPlidar R1
    pub fn set_dtr_ready(&mut self, level: bool) -> Result<()> {
        Ok(self.port.write_data_terminal_ready(level)?)
    }
}
