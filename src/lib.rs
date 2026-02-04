//! # SH8601 Driver Crate
//!
//! An embedded-graphics compatible driver for the SH8601 AMOLED display controller IC.
//!
//! This driver is not embedded-hal compatible, but provides a generic interface
//! for controlling the SH8601 display controller.
//! Different displays can be supported by implementing the `ControllerInterface` and `ResetInterface` traits.
//! This is because the SH8601 is used in different displays with various controller interfaces such as SPI or QSPI.
//! Additionally, the reset pin is controlled via GPIO or I2C GPIO expander.
//!
//! The driver currently incorporates support the Waveshare 1.8" AMOLED display out of the box, but can be extended to support other displays using the SH8601 controller.
//!
//! ## Usage
//!
//! 1. Implement the `ControllerInterface` trait for the controller driving interface Ex. QSPI
//! 2. Implement the `ResetInterface` trait for the Reset pin.
//! 3. Create a `Sh8601Driver` instance with the display interface, reset pin, and color mode.
//! 4. Use the driver to draw using `embedded-graphics`.
//!
//! If you are going to use heap allocated framebuffer, you will need to make sure that an allocator is available in your environment.
//! In some crates this is done by enabling the `alloc` feature.
//!
//! ## Color Modes
//!
//! The driver uses compile-time color mode configuration through marker types:
//!
//! | Color Mode    | Marker Type   | Bytes/Pixel | Framebuffer Size (368×448) |
//! |---------------|---------------|-------------|----------------------------|
//! | 8-bit Gray    | `Gray8Mode`   | 1           | 164 KB                     |
//! | 16-bit RGB565 | `Rgb565Mode`  | 2           | 328 KB                     |
//! | 18-bit RGB666 | `Rgb666Mode`  | 3           | 495 KB                     |
//! | 24-bit RGB888 | `Rgb888Mode`  | 3           | 495 KB                     |
//!
//! ### Using Rgb565 (Recommended for embedded)
//!
//! ```ignore
//! use sh8601_rs::{Sh8601Driver, Rgb565Mode, DisplaySize, framebuffer_size};
//! use embedded_graphics::prelude::*;
//! use embedded_graphics::primitives::{Circle, PrimitiveStyle};
//! use embedded_graphics::pixelcolor::Rgb565;
//!
//! const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
//! const FB_SIZE: usize = framebuffer_size::<Rgb565Mode>(DISPLAY_SIZE);
//!
//! let mut display = Sh8601Driver::<_, _, Rgb565Mode>::new_heap::<_, FB_SIZE>(
//!     interface,
//!     reset,
//!     DISPLAY_SIZE,
//!     delay,
//! )?;
//!
//! // Draw directly - the driver implements DrawTarget with the configured color type
//! Circle::new(Point::new(100, 100), 50)
//!     .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
//!     .draw(&mut display)?;
//!
//! display.flush()?;
//! ```
//!
//! ### Using Rgb888 (Full color depth)
//!
//! ```ignore
//! use sh8601_rs::{Sh8601Driver, Rgb888Mode, DisplaySize, framebuffer_size};
//! use embedded_graphics::pixelcolor::Rgb888;
//!
//! const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
//! const FB_SIZE: usize = framebuffer_size::<Rgb888Mode>(DISPLAY_SIZE);
//!
//! let mut display = Sh8601Driver::<_, _, Rgb888Mode>::new_heap::<_, FB_SIZE>(
//!     interface,
//!     reset,
//!     DISPLAY_SIZE,
//!     delay,
//! )?;
//!
//! Circle::new(Point::new(100, 100), 50)
//!     .into_styled(PrimitiveStyle::with_fill(Rgb888::RED))
//!     .draw(&mut display)?;
//!
//! display.flush()?;
//! ```
//!
//! ### Grayscale Mode
//!
//! ```ignore
//! use sh8601_rs::{Sh8601Driver, Gray8Mode, DisplaySize, framebuffer_size};
//! use embedded_graphics::pixelcolor::Gray8;
//!
//! const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
//! const FB_SIZE: usize = framebuffer_size::<Gray8Mode>(DISPLAY_SIZE);
//!
//! let mut display = Sh8601Driver::<_, _, Gray8Mode>::new_heap::<_, FB_SIZE>(
//!     interface,
//!     reset,
//!     DISPLAY_SIZE,
//!     delay,
//! )?;
//!
//! Circle::new(Point::new(100, 100), 50)
//!     .into_styled(PrimitiveStyle::with_fill(Gray8::WHITE))
//!     .draw(&mut display)?;
//!
//! display.flush()?;
//! ```
//!
//! ## Feature Flags
#![doc = document_features::document_features!()]
//!
//! ## Examples
//!
//! See the `examples` directory for a usage example with the WaveShare 1.8" AMOLED Display.
//!
//! The WaveShare 1.8" AMOLED Display controls the SH8601 via an ESP32-S3 over QSPI and uses an I2C GPIO expander for the reset pin.
//! The example implementation uses a PSRAM heap allocated framebuffer and DMA for efficient transfers.
//!
//! The schematic is available here: <https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf>
//!
//! To run the example, with the WaveShare 1.8" AMOLED Display, clone the project and run following command from the project root:
//! ```bash
//! cargo run --example ws_18in_amoled --features "waveshare_18_amoled"
//! ```
//!

#![no_std]
#[cfg(feature = "waveshare_18_amoled")]
pub mod displays;

#[cfg(feature = "waveshare_18_amoled")]
pub use displays::waveshare_18_amoled::*;

extern crate alloc;

mod color;
pub use color::{ColorConfig, Gray8Mode, Rgb565Mode, Rgb666Mode, Rgb888Mode};

mod graphics_core;

use alloc::boxed::Box;
use core::marker::PhantomData;
use embedded_hal::delay::DelayNs;

/// Configuration for the display dimensions.
#[derive(Debug, Clone, Copy)]
pub struct DisplaySize {
    /// Display width in pixels.
    pub width: u16,
    /// Display height in pixels.
    pub height: u16,
}

impl DisplaySize {
    pub const fn new(width: u16, height: u16) -> Self {
        DisplaySize { width, height }
    }
}

/// SH8601 Driver Errors
#[derive(Debug)]
pub enum DriverError<InterfaceError, ResetError> {
    /// Error originating from the display interface (QSPI/SPI/I2C).
    InterfaceError(InterfaceError),
    /// Error originating from the reset pin control.
    ResetError(ResetError),
    /// Invalid configuration provided to the driver.
    InvalidConfiguration(&'static str),
}

/// Trait to implement the SH8601 controller communication interface (QSPI, SPI, etc.).
pub trait ControllerInterface {
    /// The specific error type for this interface implementation.
    type Error;

    /// Sends a command byte to the display.
    fn send_command(&mut self, cmd: u8) -> Result<(), Self::Error>;

    /// Sends data bytes to the display following a command.
    fn send_command_with_data(&mut self, cmd: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Sends pixel data using RAMWR (Memory Write Start) and RAMWRC (Memory Write Continue)
    fn send_pixels(&mut self, pixels: &[u8]) -> Result<(), Self::Error>;

    /// Sends pixel data only using RAMWRC (Memory Write Continue)
    /// Used for sending additional data after initial RAMWR without re-sending command
    fn send_pixels_continue(&mut self, pixels: &[u8]) -> Result<(), Self::Error>;

    // fn read_data(&mut self, cmd: u8, buffer: &mut [u8], read_length: u8) -> Result<(), Self::Error>;
}

/// Trait for controlling the SH8601 hardware reset pin.
pub trait ResetInterface {
    /// The specific error type for this reset implementation.
    type Error;

    /// Performs the hardware reset sequence according to the SH8601 datasheet definition.
    /// This could be a GPIO port or an I2C expander-controlled pin.
    /// Implmentation should clear the reset line and wait for 20 ms then set the line high and wait for 150 ms.
    fn reset(&mut self) -> Result<(), Self::Error>;
}

/// SH8601 Command Set
pub mod commands {
    pub const NOP: u8 = 0x00;
    pub const SWRESET: u8 = 0x01;
    pub const RDDIDIF: u8 = 0x04; // Read Display Identification Information
    pub const RDDPM: u8 = 0x0A; // Read Display Power Mode
    pub const RDDMADCTL: u8 = 0x0B; // Read Display MADCTL
    pub const RDDCOLMOD: u8 = 0x0C; // Read Display Pixel Format
    pub const RDDSDR: u8 = 0x0F; // Read Display Self-Diagnostic Result
    pub const SLPIN: u8 = 0x10;
    pub const SLPOUT: u8 = 0x11;
    pub const PTLON: u8 = 0x12; // Partial Display Mode On
    pub const NORON: u8 = 0x13; // Normal Display Mode On
    pub const INVOFF: u8 = 0x20; // Display Inversion Off
    pub const INVON: u8 = 0x21; // Display Inversion On
    pub const DISPOFF: u8 = 0x28;
    pub const DISPON: u8 = 0x29;
    pub const CASET: u8 = 0x2A; // Column Address Set
    pub const PASET: u8 = 0x2B; // Page Address Set
    pub const RAMWR: u8 = 0x2C; // Memory Write Start
    pub const PTLAR: u8 = 0x30; // Partial Area Row Set
    pub const TEOFF: u8 = 0x34; // Tearing Effect Off
    pub const TEON: u8 = 0x35; // Tearing Effect On
    pub const MADCTL: u8 = 0x36; // Memory Data Access Control
    pub const IDMOFF: u8 = 0x38; // Idle Mode Off
    pub const IDMON: u8 = 0x39; // Idle Mode On
    pub const COLMOD: u8 = 0x3A; // Control Interface Pixel Format
    pub const RAMWRC: u8 = 0x3C; // Memory Write Continue
    pub const TESCAN: u8 = 0x44; // Set Tear Scan Line
    pub const AODMOFF: u8 = 0x48; // AOD Mode Off
    pub const AODMON: u8 = 0x49; // AOD Mode On
    pub const WRAOD: u8 = 0x4A; // Write AOD Brightness
    pub const RDAOD: u8 = 0x4B; // Read AOD Brightness
    pub const DEEPSTBY: u8 = 0x4F; // Deep Standby On
    pub const WRDISBV: u8 = 0x51; // Write Display Brightness Value
    pub const RDDISBV: u8 = 0x52; // Read Display Brightness Value
    pub const WRCTRLD1: u8 = 0x53; // Write CTRL Display 1
    pub const RDCTRLD1: u8 = 0x54; // Read CTRL Display 1
}

/// Computes the framebuffer size (in bytes) for a given display and color mode.
///
/// This is a compile-time helper for defining the framebuffer size constant.
///
/// # Example
///
/// ```ignore
/// use sh8601_rs::{DisplaySize, Rgb565Mode, framebuffer_size};
///
/// const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
/// const FB_SIZE: usize = framebuffer_size::<Rgb565Mode>(DISPLAY_SIZE);
/// ```
pub const fn framebuffer_size<C: ColorConfig>(display: DisplaySize) -> usize {
    (display.width as usize) * (display.height as usize) * C::BYTES_PER_PIXEL
}

/// Frambuffer enum to hold either a static array or a boxed array
pub enum Framebuffer {
    Static(&'static mut [u8]),
    Heap(Box<[u8]>),
}

impl Framebuffer {
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        match self {
            Framebuffer::Static(arr) => arr,
            Framebuffer::Heap(boxed) => boxed,
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        match self {
            Framebuffer::Static(arr) => arr,
            Framebuffer::Heap(boxed) => boxed,
        }
    }

    pub fn len(&self) -> usize {
        self.as_slice().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

// Implement Framebuffer Deref to allow accessing the underlying array
impl core::ops::Deref for Framebuffer {
    type Target = [u8];
    fn deref(&self) -> &Self::Target {
        match self {
            Framebuffer::Static(arr) => arr,
            Framebuffer::Heap(boxed) => boxed,
        }
    }
}

// Implement Frambuffer DerefMut for mutable access
impl core::ops::DerefMut for Framebuffer {
    fn deref_mut(&mut self) -> &mut Self::Target {
        match self {
            Framebuffer::Static(arr) => arr,
            Framebuffer::Heap(boxed) => boxed,
        }
    }
}

/// Main Driver for the SH8601 display controller.
///
/// Generic over the display interface (`IFACE`), reset pin (`RST`), and color mode (`C`).
///
/// The color mode is specified at compile time using marker types that implement
/// [`ColorConfig`], such as [`Rgb565Mode`], [`Rgb888Mode`], [`Rgb666Mode`], or [`Gray8Mode`].
///
/// # Example
///
/// ```ignore
/// use sh8601_rs::{Sh8601Driver, Rgb565Mode, DisplaySize, framebuffer_size};
///
/// const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
/// const FB_SIZE: usize = framebuffer_size::<Rgb565Mode>(DISPLAY_SIZE);
///
/// let mut display = Sh8601Driver::<_, _, Rgb565Mode>::new_heap::<_, FB_SIZE>(
///     interface, reset, DISPLAY_SIZE, delay
/// )?;
/// ```
pub struct Sh8601Driver<IFACE, RST, C>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
    C: ColorConfig,
{
    interface: IFACE,
    reset: RST,
    framebuffer: Framebuffer,
    config: DisplaySize,
    _color: PhantomData<C>,
}

impl<IFACE, RST, C> Sh8601Driver<IFACE, RST, C>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
    C: ColorConfig,
{
    /// Creates a new driver instance with static array and initializes the display.
    ///
    /// `N` is the framebuffer size in bytes, calculated as:
    /// `width * height * bytes_per_pixel`
    ///
    /// Use the [`framebuffer_size`] helper function to calculate `N`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use sh8601_rs::{Sh8601Driver, Rgb565Mode, DisplaySize, framebuffer_size};
    ///
    /// const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
    /// const FB_SIZE: usize = framebuffer_size::<Rgb565Mode>(DISPLAY_SIZE);
    ///
    /// static mut FRAMEBUFFER: [u8; FB_SIZE] = [0u8; FB_SIZE];
    ///
    /// let display = Sh8601Driver::<_, _, Rgb565Mode>::new_static::<_, FB_SIZE>(
    ///     interface, reset, DISPLAY_SIZE, delay, unsafe { &mut FRAMEBUFFER }
    /// )?;
    /// ```
    pub fn new_static<DELAY, const N: usize>(
        interface: IFACE,
        reset: RST,
        config: DisplaySize,
        mut delay: DELAY,
        framebuffer: &'static mut [u8; N],
    ) -> Result<Self, DriverError<IFACE::Error, RST::Error>>
    where
        DELAY: DelayNs,
    {
        let expected_size = framebuffer_size::<C>(config);
        assert_eq!(
            N,
            expected_size,
            "Framebuffer size mismatch: provided {} bytes but expected {} bytes for {}x{} display with {} bpp",
            N,
            expected_size,
            config.width,
            config.height,
            C::BYTES_PER_PIXEL
        );

        let mut driver = Self {
            interface,
            reset,
            framebuffer: Framebuffer::Static(&mut framebuffer[..]),
            config,
            _color: PhantomData,
        };
        driver.hard_reset()?;
        driver.initialize_display(&mut delay)?;
        Ok(driver)
    }

    /// Creates a new driver instance with a heap-allocated framebuffer.
    ///
    /// `N` is the framebuffer size in bytes, calculated as:
    /// `width * height * bytes_per_pixel`
    ///
    /// Use the [`framebuffer_size`] helper function to calculate `N`.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use sh8601_rs::{Sh8601Driver, Rgb565Mode, DisplaySize, framebuffer_size};
    ///
    /// const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
    /// const FB_SIZE: usize = framebuffer_size::<Rgb565Mode>(DISPLAY_SIZE);
    ///
    /// let display = Sh8601Driver::<_, _, Rgb565Mode>::new_heap::<_, FB_SIZE>(
    ///     interface, reset, DISPLAY_SIZE, delay
    /// )?;
    /// ```
    pub fn new_heap<DELAY, const N: usize>(
        interface: IFACE,
        reset: RST,
        config: DisplaySize,
        mut delay: DELAY,
    ) -> Result<Self, DriverError<IFACE::Error, RST::Error>>
    where
        DELAY: DelayNs,
    {
        let expected_size = framebuffer_size::<C>(config);
        assert_eq!(
            N,
            expected_size,
            "Framebuffer size mismatch: provided {} bytes but expected {} bytes for {}x{} display with {} bpp",
            N,
            expected_size,
            config.width,
            config.height,
            C::BYTES_PER_PIXEL
        );

        let mut driver = Self {
            interface,
            reset,
            framebuffer: Framebuffer::Heap(Box::new([0u8; N])),
            config,
            _color: PhantomData,
        };
        driver.hard_reset()?;
        driver.initialize_display(&mut delay)?;
        Ok(driver)
    }

    /// Performs a hardware reset using the provided `ResetPin` implementation,
    pub fn hard_reset(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.reset.reset().map_err(DriverError::ResetError)?;
        Ok(())
    }

    /// Sends the essential initialization command sequence to the display.
    pub fn initialize_display<DELAY>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>>
    where
        DELAY: DelayNs,
    {
        self.send_command(commands::SWRESET)?;
        delay.delay_ms(10);
        self.send_command(commands::SLPOUT)?;
        delay.delay_ms(120);
        self.send_command_with_data(commands::COLMOD, &[C::COLMOD_VALUE])?;
        delay.delay_ms(5);
        self.send_command_with_data(commands::MADCTL, &[0x00])?;
        self.send_command_with_data(commands::TESCAN, &[0x01, 0xC5])?;
        self.send_command_with_data(commands::TEON, &[0x00])?;

        self.send_command(commands::DISPON)?;
        delay.delay_ms(120);

        self.send_command_with_data(commands::PTLAR, &[0x00, 0x80, 0x00, 0x02])?;
        delay.delay_ms(10);

        Ok(())
    }

    /// Send a command with no data
    fn send_command(&mut self, cmd: u8) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.interface
            .send_command(cmd)
            .map_err(DriverError::InterfaceError)
    }

    /// Helper to send a command with associated data parameters
    fn send_command_with_data(
        &mut self,
        cmd: u8,
        data: &[u8],
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.interface
            .send_command_with_data(cmd, data)
            .map_err(DriverError::InterfaceError)?;
        Ok(())
    }

    /// Returns the number of bytes per pixel for the configured color mode.
    pub const fn bytes_per_pixel(&self) -> usize {
        C::BYTES_PER_PIXEL
    }

    /// Sleep Mode In (SLPIN)
    pub fn sleep_in<DELAY>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>>
    where
        DELAY: DelayNs,
    {
        self.send_command(commands::SLPIN)?;
        delay.delay_ms(5);
        Ok(())
    }

    /// SH8601 Sleep Out (SLPOUT)
    pub fn sleep_out<DELAY>(
        &mut self,
        delay: &mut DELAY,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>>
    where
        DELAY: DelayNs,
    {
        self.send_command(commands::SLPOUT)?;
        delay.delay_ms(5);
        Ok(())
    }

    /// Turns the display panel off
    pub fn display_off(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.send_command(commands::DISPOFF)
    }

    /// Turns the display panel on
    pub fn display_on(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.send_command(commands::DISPON)
    }

    /// Sets the active drawing window on the display RAM.
    pub fn set_window(
        &mut self,
        x_start: u16,
        y_start: u16,
        x_end: u16,
        y_end: u16,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        if x_end == 0 || y_end == 0 {
            return Err(DriverError::InvalidConfiguration(
                "Window width/height cannot be zero",
            ));
        }
        if x_start >= self.config.width || y_start >= self.config.height {
            return Err(DriverError::InvalidConfiguration(
                "Window start coordinates out of bounds",
            ));
        }

        if x_end < x_start || y_end < y_start {
            return Err(DriverError::InvalidConfiguration(
                "Invalid window dimensions (end < start)",
            ));
        }

        // CASET (2Ah): Column Address Set
        self.send_command_with_data(
            commands::CASET,
            &[
                (x_start >> 8) as u8,
                (x_start & 0xFF) as u8, // Start Column SC[15:0]
                (x_end >> 8) as u8,
                (x_end & 0xFF) as u8, // End Column EC[15:0]
            ],
        )?;

        // PASET (2Bh): Page Address Set
        self.send_command_with_data(
            commands::PASET,
            &[
                (y_start >> 8) as u8,
                (y_start & 0xFF) as u8, // Start Page SP[15:0]
                (y_end >> 8) as u8,
                (y_end & 0xFF) as u8, // End Page EP[15:0]
            ],
        )?;
        Ok(())
    }

    /// Sets the Memory Data Access Control (MADCTL) register (controls orientation, color order).
    pub fn set_madctl(&mut self, value: u8) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.send_command_with_data(commands::MADCTL, &[value])
    }

    /// Sets the display brightness (0x000 - 0x3FF).
    pub fn set_brightness(
        &mut self,
        value: u16,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        let brightness = value.min(0x3FF); // Clamp to 10 bits (0-1023)
        let val_lsb = (brightness & 0xFF) as u8;
        let val_msb = ((brightness >> 8) & 0x03) as u8;
        self.send_command_with_data(commands::WRDISBV, &[val_lsb, val_msb])
    }

    /// Enters Always-On Display (AOD) mode.
    /// In AOD mode, the display operates in a low-power state while still showing content.
    pub fn aod_mode_on(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.send_command(commands::AODMON)
    }

    /// Exits Always-On Display (AOD) mode.
    /// Returns the display to normal operation mode.
    pub fn aod_mode_off(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.send_command(commands::AODMOFF)
    }

    /// Sets the AOD mode brightness (0x0000 - 0xFFFF).
    /// This brightness setting applies specifically when the display is in AOD mode.
    pub fn set_aod_brightness(
        &mut self,
        value: u16,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        let val_lsb = (value & 0xFF) as u8;
        let val_msb = ((value >> 8) & 0xFF) as u8;
        self.send_command_with_data(commands::WRAOD, &[val_lsb, val_msb])
    }

    /// Enters deep standby mode (lowest power state).
    /// In deep standby, the display is essentially off and typically requires a hardware reset to wake.
    /// This is a hard-off state with minimal power consumption.
    pub fn deep_standby_on(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.send_command(commands::DEEPSTBY)
    }

    /// Writes the pixel data to the display RAM.
    /// This is typically called for low-level pixel data updates, like writing widget updates directly.
    pub fn write_raw(
        &mut self,
        x_start: u16,
        x_end: u16,
        y_start: u16,
        y_end: u16,
        pixel_data: &[u8],
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.set_window(x_start, y_start, x_end, y_end)?;
        self.interface
            .send_pixels(pixel_data)
            .map_err(DriverError::InterfaceError)?;
        Ok(())
    }

    /// Writes the contents of the framebuffer to the display RAM.
    /// This is typically called after drawing operations are complete.
    pub fn flush(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        // Ensure the window covers the whole framebuffer before writing
        self.set_window(0, 0, self.config.width - 1, self.config.height - 1)?;
        // Send the pixel data via the interface's optimized method.
        // The send_pixels method itself should handle sending RAMWR (0x2C).
        self.interface
            .send_pixels(&self.framebuffer)
            .map_err(DriverError::InterfaceError)?;
        Ok(())
    }

    pub fn partial_flush(
        &mut self,
        x_start: u16,
        x_end: u16,
        y_start: u16,
        y_end: u16,
    ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
        self.set_window(x_start, y_start, x_end, y_end)?;

        let bytes_per_pixel = C::BYTES_PER_PIXEL;
        let fb_width = self.config.width as usize * bytes_per_pixel;
        let row_bytes = (x_end - x_start + 1) as usize * bytes_per_pixel;
        let x_offset = x_start as usize * bytes_per_pixel;

        // Full-width region fast path: send contiguous framebuffer slice directly
        if x_start == 0 && x_end == self.config.width - 1 {
            let start_offset = y_start as usize * fb_width;
            let end_offset = (y_end as usize + 1) * fb_width;

            if end_offset <= self.framebuffer.len() {
                return self
                    .interface
                    .send_pixels(&self.framebuffer[start_offset..end_offset])
                    .map_err(DriverError::InterfaceError);
            }
        }

        // Partial-width region: send row by row
        let first_row_offset = y_start as usize * fb_width + x_offset;
        let first_row_end = first_row_offset + row_bytes;

        if first_row_end > self.framebuffer.len() {
            return Err(DriverError::InvalidConfiguration(
                "Framebuffer slice out of bounds",
            ));
        }

        self.interface
            .send_pixels(&self.framebuffer[first_row_offset..first_row_end])
            .map_err(DriverError::InterfaceError)?;

        // Remaining rows use RAMWRC (via send_pixels_continue)
        for y in (y_start + 1)..=y_end {
            let row_offset = y as usize * fb_width + x_offset;
            let row_end = row_offset + row_bytes;

            if row_end > self.framebuffer.len() {
                return Err(DriverError::InvalidConfiguration(
                    "Framebuffer slice out of bounds",
                ));
            }

            self.interface
                .send_pixels_continue(&self.framebuffer[row_offset..row_end])
                .map_err(DriverError::InterfaceError)?;
        }

        Ok(())
    }

    /// Returns mutable access to the raw framebuffer bytes.
    ///
    /// This is useful for direct pixel manipulation operations like memmove-based
    /// scrolling optimization. The framebuffer is in row-major order with pixels
    /// stored according to the configured ColorMode (e.g., 2 bytes per pixel for RGB565).
    ///
    /// # Safety
    ///
    /// Caller must ensure writes stay within bounds and use the correct pixel format.
    pub fn framebuffer_mut(&mut self) -> &mut [u8] {
        self.framebuffer.as_mut_slice()
    }

    /// Returns the display configuration (width and height).
    pub fn display_size(&self) -> DisplaySize {
        self.config
    }
}
