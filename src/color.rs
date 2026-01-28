//! Color mode configuration traits and marker types.
//!
//! This module provides compile-time color mode configuration through marker types
//! that implement the [`ColorConfig`] trait. Each marker type encapsulates all the
//! information needed to handle a specific color format.

use embedded_graphics_core::pixelcolor::{Gray8, IntoStorage, PixelColor, Rgb565, Rgb666, Rgb888};

/// Trait defining color format configuration for the display driver.
///
/// This trait is implemented by marker types that represent different color modes.
/// It provides all the compile-time and runtime information needed to handle
/// pixel colors in a specific format.
///
/// # Example
///
/// ```ignore
/// use sh8601_rs::{Sh8601Driver, Rgb565Mode, DisplaySize};
///
/// // Create a driver with Rgb565 color mode
/// let display = Sh8601Driver::<_, _, Rgb565Mode>::new_heap::<_, FB_SIZE>(
///     interface, reset, config, delay
/// )?;
/// ```
pub trait ColorConfig: Sealed {
    /// The embedded-graphics pixel color type for this color mode.
    type Color: PixelColor;

    /// Byte array type for storing a single pixel.
    /// This allows each color mode to use the correct array size.
    type Bytes: AsRef<[u8]> + AsMut<[u8]> + Copy;

    /// Number of bytes per pixel.
    const BYTES_PER_PIXEL: usize;

    /// COLMOD register value for the SH8601 display controller.
    const COLMOD_VALUE: u8;

    /// Writes a pixel color to a byte buffer.
    ///
    /// The buffer is guaranteed to have at least `BYTES_PER_PIXEL` bytes.
    fn write_pixel(color: Self::Color, buf: &mut [u8]);

    /// Converts a color to its byte representation.
    fn to_bytes(color: Self::Color) -> Self::Bytes;

    /// Checks if the byte representation can be filled with a single uniform byte.
    /// Returns `Some(byte)` if all bytes are the same (useful for grayscale fast-fill).
    fn is_uniform(bytes: &Self::Bytes) -> Option<u8>;
}

/// Sealed trait to prevent external implementations of ColorConfig.
mod private {
    pub trait Sealed {}
}
use private::Sealed;

// ============================================================================
// Rgb565 Mode
// ============================================================================

/// Marker type for RGB565 color mode (16-bit, 2 bytes per pixel).
///
/// This is the recommended color mode for embedded systems with limited memory.
/// - 5 bits red, 6 bits green, 5 bits blue
/// - Framebuffer size for 368×448 display: ~328 KB
#[derive(Debug, Clone, Copy, Default)]
pub struct Rgb565Mode;

impl Sealed for Rgb565Mode {}

impl ColorConfig for Rgb565Mode {
    type Color = Rgb565;
    type Bytes = [u8; 2];

    const BYTES_PER_PIXEL: usize = 2;
    const COLMOD_VALUE: u8 = 0x55;

    #[inline]
    fn write_pixel(color: Self::Color, buf: &mut [u8]) {
        let rgb = color.into_storage();
        buf[0] = (rgb >> 8) as u8;
        buf[1] = rgb as u8;
    }

    #[inline]
    fn to_bytes(color: Self::Color) -> Self::Bytes {
        let rgb = color.into_storage();
        [(rgb >> 8) as u8, rgb as u8]
    }

    #[inline]
    fn is_uniform(_bytes: &Self::Bytes) -> Option<u8> {
        // RGB565 rarely has uniform bytes due to 5-6-5 encoding
        None
    }
}

// ============================================================================
// Rgb888 Mode
// ============================================================================

/// Marker type for RGB888 color mode (24-bit, 3 bytes per pixel).
///
/// Provides the highest color depth but requires more memory.
/// - 8 bits each for red, green, blue
/// - Framebuffer size for 368×448 display: ~495 KB
#[derive(Debug, Clone, Copy, Default)]
pub struct Rgb888Mode;

impl Sealed for Rgb888Mode {}

impl ColorConfig for Rgb888Mode {
    type Color = Rgb888;
    type Bytes = [u8; 3];

    const BYTES_PER_PIXEL: usize = 3;
    const COLMOD_VALUE: u8 = 0x77;

    #[inline]
    fn write_pixel(color: Self::Color, buf: &mut [u8]) {
        let rgb = color.into_storage();
        buf[0] = (rgb >> 16) as u8;
        buf[1] = (rgb >> 8) as u8;
        buf[2] = rgb as u8;
    }

    #[inline]
    fn to_bytes(color: Self::Color) -> Self::Bytes {
        let rgb = color.into_storage();
        [(rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8]
    }

    #[inline]
    fn is_uniform(bytes: &Self::Bytes) -> Option<u8> {
        if bytes[0] == bytes[1] && bytes[0] == bytes[2] {
            Some(bytes[0])
        } else {
            None
        }
    }
}

// ============================================================================
// Rgb666 Mode
// ============================================================================

/// Marker type for RGB666 color mode (18-bit, 3 bytes per pixel).
///
/// - 6 bits each for red, green, blue
/// - Framebuffer size for 368×448 display: ~495 KB
#[derive(Debug, Clone, Copy, Default)]
pub struct Rgb666Mode;

impl Sealed for Rgb666Mode {}

impl ColorConfig for Rgb666Mode {
    type Color = Rgb666;
    type Bytes = [u8; 3];

    const BYTES_PER_PIXEL: usize = 3;
    const COLMOD_VALUE: u8 = 0x66;

    #[inline]
    fn write_pixel(color: Self::Color, buf: &mut [u8]) {
        let rgb = color.into_storage();
        buf[0] = (rgb >> 12) as u8;
        buf[1] = (rgb >> 6) as u8;
        buf[2] = rgb as u8;
    }

    #[inline]
    fn to_bytes(color: Self::Color) -> Self::Bytes {
        let rgb = color.into_storage();
        [(rgb >> 12) as u8, (rgb >> 6) as u8, rgb as u8]
    }

    #[inline]
    fn is_uniform(bytes: &Self::Bytes) -> Option<u8> {
        if bytes[0] == bytes[1] && bytes[0] == bytes[2] {
            Some(bytes[0])
        } else {
            None
        }
    }
}

// ============================================================================
// Gray8 Mode
// ============================================================================

/// Marker type for Gray8 color mode (8-bit grayscale, 1 byte per pixel).
///
/// Most memory-efficient mode for monochrome/grayscale displays.
/// - Framebuffer size for 368×448 display: ~164 KB
#[derive(Debug, Clone, Copy, Default)]
pub struct Gray8Mode;

impl Sealed for Gray8Mode {}

impl ColorConfig for Gray8Mode {
    type Color = Gray8;
    type Bytes = [u8; 1];

    const BYTES_PER_PIXEL: usize = 1;
    const COLMOD_VALUE: u8 = 0x11;

    #[inline]
    fn write_pixel(color: Self::Color, buf: &mut [u8]) {
        buf[0] = color.into_storage();
    }

    #[inline]
    fn to_bytes(color: Self::Color) -> Self::Bytes {
        [color.into_storage()]
    }

    #[inline]
    fn is_uniform(bytes: &Self::Bytes) -> Option<u8> {
        // Always uniform for Gray8
        Some(bytes[0])
    }
}

