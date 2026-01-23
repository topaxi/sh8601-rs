//! High-level display wrapper types for different color modes.
//!
//! This module provides convenient wrapper types that encapsulate both the driver
//! and the color mode, making it easier to work with specific color formats.

use crate::{
    ColorMode, ControllerInterface, DisplaySize, DriverError, ResetInterface, Sh8601Driver,
};
use embedded_hal::delay::DelayNs;

/// Macro to generate common methods for display wrappers
macro_rules! impl_display_common {
    ($name:ident, $color_mode:expr) => {
        impl<IFACE, RST> $name<IFACE, RST>
        where
            IFACE: ControllerInterface,
            RST: ResetInterface,
        {
            /// Creates a new display with heap-allocated framebuffer
            pub fn new_heap<DELAY, const N: usize>(
                interface: IFACE,
                reset: RST,
                config: DisplaySize,
                delay: DELAY,
            ) -> Result<Self, DriverError<IFACE::Error, RST::Error>>
            where
                DELAY: DelayNs,
            {
                let driver = Sh8601Driver::new_heap::<DELAY, N>(
                    interface,
                    reset,
                    $color_mode,
                    config,
                    delay,
                )?;
                Ok(Self { driver })
            }

            /// Creates a new display with static array framebuffer
            pub fn new_static<DELAY, const N: usize>(
                interface: IFACE,
                reset: RST,
                config: DisplaySize,
                delay: DELAY,
                framebuffer: &'static mut [u8; N],
            ) -> Result<Self, DriverError<IFACE::Error, RST::Error>>
            where
                DELAY: DelayNs,
            {
                let driver = Sh8601Driver::new_static::<DELAY, N>(
                    interface,
                    reset,
                    $color_mode,
                    config,
                    delay,
                    framebuffer,
                )?;
                Ok(Self { driver })
            }

            /// Performs a hardware reset
            #[inline]
            pub fn hard_reset(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.hard_reset()
            }

            /// Sleep Mode In (SLPIN)
            #[inline]
            pub fn sleep_in<DELAY>(
                &mut self,
                delay: &mut DELAY,
            ) -> Result<(), DriverError<IFACE::Error, RST::Error>>
            where
                DELAY: DelayNs,
            {
                self.driver.sleep_in(delay)
            }

            /// Sleep Out (SLPOUT)
            #[inline]
            pub fn sleep_out<DELAY>(
                &mut self,
                delay: &mut DELAY,
            ) -> Result<(), DriverError<IFACE::Error, RST::Error>>
            where
                DELAY: DelayNs,
            {
                self.driver.sleep_out(delay)
            }

            /// Turns the display panel off
            #[inline]
            pub fn display_off(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.display_off()
            }

            /// Turns the display panel on
            #[inline]
            pub fn display_on(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.display_on()
            }

            /// Sets the active drawing window on the display RAM
            #[inline]
            pub fn set_window(
                &mut self,
                x_start: u16,
                y_start: u16,
                x_end: u16,
                y_end: u16,
            ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.set_window(x_start, y_start, x_end, y_end)
            }

            /// Sets the Memory Data Access Control (MADCTL) register
            #[inline]
            pub fn set_madctl(
                &mut self,
                value: u8,
            ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.set_madctl(value)
            }

            /// Sets the display brightness (0x000 - 0x3FF)
            #[inline]
            pub fn set_brightness(
                &mut self,
                value: u16,
            ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.set_brightness(value)
            }

            /// Writes the contents of the framebuffer to the display RAM
            #[inline]
            pub fn flush(&mut self) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver.flush()
            }

            /// Writes a partial region of the framebuffer to the display
            #[inline]
            pub fn partial_flush(
                &mut self,
                x_start: u16,
                x_end: u16,
                y_start: u16,
                y_end: u16,
            ) -> Result<(), DriverError<IFACE::Error, RST::Error>> {
                self.driver
                    .partial_flush(x_start, x_end, y_start, y_end, $color_mode)
            }

            /// Returns the color mode
            #[inline]
            pub fn color_mode(&self) -> ColorMode {
                $color_mode
            }

            /// Get display size configuration
            #[inline]
            pub fn size(&self) -> DisplaySize {
                self.driver.config
            }

            /// Get immutable reference to the underlying driver
            #[inline]
            pub fn driver(&self) -> &Sh8601Driver<IFACE, RST> {
                &self.driver
            }

            /// Get mutable reference to the underlying driver
            #[inline]
            pub fn driver_mut(&mut self) -> &mut Sh8601Driver<IFACE, RST> {
                &mut self.driver
            }
        }
    };
}

// ============================================================================
// Rgb565 Display
// ============================================================================

/// Display wrapper for Rgb565 mode (16-bit color, 2 bytes per pixel)
///
/// This is the recommended color mode for embedded systems with limited memory.
/// Framebuffer size for 368×448 display: ~328 KB
pub struct Sh8601Rgb565Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    driver: Sh8601Driver<IFACE, RST>,
}

impl_display_common!(Sh8601Rgb565Display, ColorMode::Rgb565);

impl<IFACE, RST> Sh8601Rgb565Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    /// Get mutable reference to the underlying driver as Rgb565 DrawTarget
    #[inline]
    pub fn as_draw_target(&mut self) -> crate::Rgb565DrawTarget<'_, IFACE, RST> {
        self.driver.as_rgb565()
    }
}

// ============================================================================
// Rgb888 Display
// ============================================================================

/// Display wrapper for Rgb888 mode (24-bit color, 3 bytes per pixel)
///
/// Provides the highest color depth but requires more memory.
/// Framebuffer size for 368×448 display: ~495 KB
pub struct Sh8601Rgb888Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    driver: Sh8601Driver<IFACE, RST>,
}

impl_display_common!(Sh8601Rgb888Display, ColorMode::Rgb888);

impl<IFACE, RST> Sh8601Rgb888Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    /// Get mutable reference to the underlying driver (implements DrawTarget for Rgb888)
    #[inline]
    pub fn as_draw_target(&mut self) -> &mut Sh8601Driver<IFACE, RST> {
        &mut self.driver
    }
}

// ============================================================================
// Rgb666 Display
// ============================================================================

/// Display wrapper for Rgb666 mode (18-bit color, 3 bytes per pixel)
///
/// Framebuffer size for 368×448 display: ~495 KB
pub struct Sh8601Rgb666Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    driver: Sh8601Driver<IFACE, RST>,
}

impl_display_common!(Sh8601Rgb666Display, ColorMode::Rgb666);

impl<IFACE, RST> Sh8601Rgb666Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    /// Get mutable reference to the underlying driver as Rgb666 DrawTarget
    #[inline]
    pub fn as_draw_target(&mut self) -> crate::Rgb666DrawTarget<'_, IFACE, RST> {
        self.driver.as_rgb666()
    }
}

// ============================================================================
// Gray8 Display
// ============================================================================

/// Display wrapper for Gray8 mode (8-bit grayscale, 1 byte per pixel)
///
/// Most memory-efficient mode for monochrome displays.
/// Framebuffer size for 368×448 display: ~164 KB
pub struct Sh8601Gray8Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    driver: Sh8601Driver<IFACE, RST>,
}

impl_display_common!(Sh8601Gray8Display, ColorMode::Gray8);

impl<IFACE, RST> Sh8601Gray8Display<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    /// Get mutable reference to the underlying driver as Gray8 DrawTarget
    #[inline]
    pub fn as_draw_target(&mut self) -> crate::Gray8DrawTarget<'_, IFACE, RST> {
        self.driver.as_gray8()
    }
}
