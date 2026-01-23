use crate::{ControllerInterface, ResetInterface, Sh8601Driver};
use embedded_graphics_core::{
    pixelcolor::{Gray8, Rgb565, Rgb666, Rgb888},
    prelude::*,
    primitives::Rectangle,
};

// =========== Macro for Color Mode Wrappers ===========

/// Macro to generate wrapper types and implementations for different color modes.
///
/// This macro reduces code duplication by generating:
/// 1. Wrapper struct definition
/// 2. Constructor method on Sh8601Driver
/// 3. Proxy methods for all public driver methods
/// 4. DrawTarget implementation with color-specific logic
macro_rules! define_color_wrapper {
    (
        name: $wrapper:ident,
        method_name: $method_name:ident,
        color: $color:ty,
        mode: $color_mode:expr,
        bpp: $bpp:literal,
        write_pixel: |$color_arg:ident, $buf:ident| $write_expr:expr,
        write_bytes: |$color_arg2:ident| $bytes_expr:expr,
        is_uniform: |$bytes_arg:ident| $uniform_expr:expr,
    ) => {
        /// Wrapper for drawing with specific color mode
        pub struct $wrapper<'a, IFACE, RST>(&'a mut Sh8601Driver<IFACE, RST>)
        where
            IFACE: ControllerInterface,
            RST: ResetInterface;

        impl<'a, IFACE, RST> $wrapper<'a, IFACE, RST>
        where
            IFACE: ControllerInterface,
            RST: ResetInterface,
        {
            /// Flush the framebuffer to the display
            pub fn flush(&mut self) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.flush()
            }

            /// Performs a hardware reset using the provided `ResetPin` implementation
            pub fn hard_reset(
                &mut self,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.hard_reset()
            }

            /// Sleep Mode In (SLPIN)
            pub fn sleep_in<DELAY>(
                &mut self,
                delay: &mut DELAY,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>>
            where
                DELAY: embedded_hal::delay::DelayNs,
            {
                self.0.sleep_in(delay)
            }

            /// Sleep Out (SLPOUT)
            pub fn sleep_out<DELAY>(
                &mut self,
                delay: &mut DELAY,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>>
            where
                DELAY: embedded_hal::delay::DelayNs,
            {
                self.0.sleep_out(delay)
            }

            /// Turns the display panel off
            pub fn display_off(
                &mut self,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.display_off()
            }

            /// Turns the display panel on
            pub fn display_on(
                &mut self,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.display_on()
            }

            /// Sets the active drawing window on the display RAM
            pub fn set_window(
                &mut self,
                x_start: u16,
                y_start: u16,
                x_end: u16,
                y_end: u16,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.set_window(x_start, y_start, x_end, y_end)
            }

            /// Sets the Memory Data Access Control (MADCTL) register
            pub fn set_madctl(
                &mut self,
                value: u8,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.set_madctl(value)
            }

            /// Sets the display brightness (0x000 - 0x3FF)
            pub fn set_brightness(
                &mut self,
                value: u16,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.set_brightness(value)
            }

            /// Writes a partial region of the framebuffer to the display
            pub fn partial_flush(
                &mut self,
                x_start: u16,
                x_end: u16,
                y_start: u16,
                y_end: u16,
                color: crate::ColorMode,
            ) -> Result<(), crate::DriverError<IFACE::Error, RST::Error>> {
                self.0.partial_flush(x_start, x_end, y_start, y_end, color)
            }

            /// Returns the color mode configured for this display
            pub fn color_mode(&self) -> crate::ColorMode {
                self.0.color_mode()
            }
        }

        impl<'a, IFACE, RST> Sh8601Driver<IFACE, RST>
        where
            IFACE: ControllerInterface,
            RST: ResetInterface,
        {
            /// Get a DrawTarget for this color mode
            ///
            /// # Panics
            /// Panics if the display was not initialized with the matching ColorMode
            pub fn $method_name(&'a mut self) -> $wrapper<'a, IFACE, RST> {
                assert_eq!(
                    self.color_mode,
                    $color_mode,
                    "Display initialized with {:?} but {}() requires {:?}",
                    self.color_mode,
                    stringify!($method_name),
                    $color_mode
                );
                $wrapper(self)
            }
        }

        impl<IFACE, RST> DrawTarget for $wrapper<'_, IFACE, RST>
        where
            IFACE: ControllerInterface,
            RST: ResetInterface,
        {
            type Color = $color;
            type Error = core::convert::Infallible;

            fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
            where
                I: IntoIterator<Item = Pixel<Self::Color>>,
            {
                draw_iter_impl::<$color, $bpp>(
                    &mut self.0.framebuffer,
                    self.0.config.width as i32,
                    self.0.config.height as i32,
                    pixels,
                    |$color_arg, $buf| $write_expr,
                )
            }

            fn fill_solid(
                &mut self,
                area: &Rectangle,
                color: Self::Color,
            ) -> Result<(), Self::Error> {
                fill_solid_impl::<$color, $bpp>(
                    &mut self.0.framebuffer,
                    self.0.config.width,
                    self.0.config.height,
                    area,
                    color,
                    |$color_arg2| $bytes_expr,
                    |$bytes_arg| $uniform_expr,
                )
            }

            fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
            where
                I: IntoIterator<Item = Self::Color>,
            {
                fill_contiguous_impl::<$color, $bpp>(
                    &mut self.0.framebuffer,
                    self.0.config.width,
                    self.0.config.height,
                    area,
                    colors,
                    |$color_arg, $buf| $write_expr,
                )
            }

            fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
                clear_impl::<$color, $bpp>(
                    &mut self.0.framebuffer,
                    color,
                    |$color_arg2| $bytes_expr,
                    |$bytes_arg| $uniform_expr,
                )
            }
        }

        impl<IFACE, RST> OriginDimensions for $wrapper<'_, IFACE, RST>
        where
            IFACE: ControllerInterface,
            RST: ResetInterface,
        {
            fn size(&self) -> Size {
                Size::new(self.0.config.width as u32, self.0.config.height as u32)
            }
        }
    };
}

// =========== Generic Helper Functions ===========

/// Generic implementation of draw_iter for any color format
///
/// # Safety
/// Assumes framebuffer size has been validated at initialization to be exactly
/// `width * height * BPP` bytes. This allows us to skip redundant bounds checking
/// in the hot path - coordinate bounds check is sufficient.
#[inline]
fn draw_iter_impl<C: PixelColor, const BPP: usize>(
    framebuffer: &mut [u8],
    width: i32,
    height: i32,
    pixels: impl IntoIterator<Item = Pixel<C>>,
    write_pixel: impl Fn(C, &mut [u8]),
) -> Result<(), core::convert::Infallible> {
    for Pixel(coord, color) in pixels {
        let x = coord.x;
        let y = coord.y;

        // Coordinate bounds check (required for clipping)
        if (x | y) < 0 || x >= width || y >= height {
            continue;
        }

        // SAFETY: If 0 <= x < width && 0 <= y < height, then:
        // idx = (y * width + x) * BPP < width * height * BPP = framebuffer.len()
        // This is guaranteed by framebuffer size validation at initialization.
        let idx = (y as usize * width as usize + x as usize) * BPP;

        write_pixel(color, &mut framebuffer[idx..idx + BPP]);
    }

    Ok(())
}

/// Generic implementation of fill_solid for any color format
#[inline]
fn fill_solid_impl<C: PixelColor, const BPP: usize>(
    framebuffer: &mut [u8],
    display_width: u16,
    display_height: u16,
    area: &Rectangle,
    color: C,
    write_bytes: impl Fn(C) -> [u8; BPP],
    is_uniform: impl Fn(&[u8; BPP]) -> Option<u8>,
) -> Result<(), core::convert::Infallible> {
    let display_area = Rectangle::new(
        Point::zero(),
        Size::new(display_width as u32, display_height as u32),
    );
    let area = match area.intersection(&display_area) {
        a if a.size.width == 0 || a.size.height == 0 => return Ok(()),
        a => a,
    };

    let bytes = write_bytes(color);

    let width = display_width as usize;
    let start_x = area.top_left.x as usize;
    let start_y = area.top_left.y as usize;
    let rect_width = area.size.width as usize;
    let rect_height = area.size.height as usize;

    // Fast path for uniform colors (grayscale)
    if let Some(uniform_byte) = is_uniform(&bytes) {
        for row in 0..rect_height {
            let y = start_y + row;
            let row_start = (y * width + start_x) * BPP;
            let row_end = row_start + rect_width * BPP;
            framebuffer[row_start..row_end].fill(uniform_byte);
        }
        return Ok(());
    }

    // General path
    for row in 0..rect_height {
        let y = start_y + row;
        let row_start = (y * width + start_x) * BPP;
        let row_end = row_start + rect_width * BPP;

        let row_slice = &mut framebuffer[row_start..row_end];
        for chunk in row_slice.chunks_exact_mut(BPP) {
            chunk.copy_from_slice(&bytes);
        }
    }

    Ok(())
}

/// Generic implementation of fill_contiguous for any color format
#[inline]
fn fill_contiguous_impl<C: PixelColor, const BPP: usize>(
    framebuffer: &mut [u8],
    display_width: u16,
    display_height: u16,
    area: &Rectangle,
    colors: impl IntoIterator<Item = C>,
    write_pixel: impl Fn(C, &mut [u8]),
) -> Result<(), core::convert::Infallible> {
    let display_area = Rectangle::new(
        Point::zero(),
        Size::new(display_width as u32, display_height as u32),
    );
    let clipped = area.intersection(&display_area);
    if clipped.size.width == 0 || clipped.size.height == 0 {
        return Ok(());
    }

    let width = display_width as usize;
    let area_width = area.size.width as usize;
    let clipped_width = clipped.size.width as usize;
    let clipped_height = clipped.size.height as usize;

    let skip_left = (clipped.top_left.x - area.top_left.x) as usize;
    let skip_top = (clipped.top_left.y - area.top_left.y) as usize;

    let mut colors = colors.into_iter();

    for _ in 0..(skip_top * area_width) {
        colors.next();
    }

    let start_x = clipped.top_left.x as usize;
    let start_y = clipped.top_left.y as usize;

    for row in 0..clipped_height {
        for _ in 0..skip_left {
            colors.next();
        }

        let y = start_y + row;
        let row_start = (y * width + start_x) * BPP;

        for col in 0..clipped_width {
            if let Some(color) = colors.next() {
                let idx = row_start + col * BPP;
                write_pixel(color, &mut framebuffer[idx..idx + BPP]);
            }
        }

        let skip_right = area_width - skip_left - clipped_width;
        for _ in 0..skip_right {
            colors.next();
        }
    }

    Ok(())
}

/// Generic implementation of clear for any color format
#[inline]
fn clear_impl<C: PixelColor, const BPP: usize>(
    framebuffer: &mut [u8],
    color: C,
    write_bytes: impl Fn(C) -> [u8; BPP],
    is_uniform: impl Fn(&[u8; BPP]) -> Option<u8>,
) -> Result<(), core::convert::Infallible> {
    let bytes = write_bytes(color);

    // Fast path for uniform colors (grayscale)
    if let Some(uniform_byte) = is_uniform(&bytes) {
        framebuffer.fill(uniform_byte);
        return Ok(());
    }

    // General path
    for chunk in framebuffer.chunks_exact_mut(BPP) {
        chunk.copy_from_slice(&bytes);
    }

    Ok(())
}

// =========== Rgb888 DrawTarget Implementation ===========

impl<IFACE, RST> DrawTarget for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = Rgb888;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        draw_iter_impl::<Rgb888, 3>(
            &mut self.framebuffer,
            self.config.width as i32,
            self.config.height as i32,
            pixels,
            |color, buf| {
                let rgb = color.into_storage();
                buf[0] = (rgb >> 16) as u8;
                buf[1] = (rgb >> 8) as u8;
                buf[2] = rgb as u8;
            },
        )
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        fill_solid_impl::<Rgb888, 3>(
            &mut self.framebuffer,
            self.config.width,
            self.config.height,
            area,
            color,
            |color| {
                let rgb = color.into_storage();
                [(rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8]
            },
            |bytes| {
                if bytes[0] == bytes[1] && bytes[0] == bytes[2] {
                    Some(bytes[0])
                } else {
                    None
                }
            },
        )
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        fill_contiguous_impl::<Rgb888, 3>(
            &mut self.framebuffer,
            self.config.width,
            self.config.height,
            area,
            colors,
            |color, buf| {
                let rgb = color.into_storage();
                buf[0] = (rgb >> 16) as u8;
                buf[1] = (rgb >> 8) as u8;
                buf[2] = rgb as u8;
            },
        )
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        clear_impl::<Rgb888, 3>(
            &mut self.framebuffer,
            color,
            |color| {
                let rgb = color.into_storage();
                [(rgb >> 16) as u8, (rgb >> 8) as u8, rgb as u8]
            },
            |bytes| {
                if bytes[0] == bytes[1] && bytes[0] == bytes[2] {
                    Some(bytes[0])
                } else {
                    None
                }
            },
        )
    }
}

impl<IFACE, RST> OriginDimensions for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    fn size(&self) -> Size {
        Size::new(self.config.width as u32, self.config.height as u32)
    }
}

// =========== Wrapper Types for Other Color Modes ===========

define_color_wrapper! {
    name: Rgb565DrawTarget,
    method_name: as_rgb565,
    color: Rgb565,
    mode: crate::ColorMode::Rgb565,
    bpp: 2,
    write_pixel: |color, buf| {
        let rgb = color.into_storage();
        buf[0] = (rgb >> 8) as u8;
        buf[1] = rgb as u8;
    },
    write_bytes: |color| {
        let rgb = color.into_storage();
        [(rgb >> 8) as u8, rgb as u8]
    },
    is_uniform: |bytes| {
        // Check if it's grayscale (R5 == G6 >> 1 == B5)
        let r = bytes[0] >> 3;
        let g = ((bytes[0] & 0x07) << 3) | (bytes[1] >> 5);
        let b = bytes[1] & 0x1F;
        if r == (g >> 1) && r == b {
            // Approximate uniform byte (won't be perfect due to 565 encoding)
            None
        } else {
            None
        }
    },
}

define_color_wrapper! {
    name: Rgb666DrawTarget,
    method_name: as_rgb666,
    color: Rgb666,
    mode: crate::ColorMode::Rgb666,
    bpp: 3,
    write_pixel: |color, buf| {
        let rgb = color.into_storage();
        buf[0] = (rgb >> 12) as u8;
        buf[1] = (rgb >> 6) as u8;
        buf[2] = rgb as u8;
    },
    write_bytes: |color| {
        let rgb = color.into_storage();
        [(rgb >> 12) as u8, (rgb >> 6) as u8, rgb as u8]
    },
    is_uniform: |bytes| {
        if bytes[0] == bytes[1] && bytes[0] == bytes[2] {
            Some(bytes[0])
        } else {
            None
        }
    },
}

define_color_wrapper! {
    name: Gray8DrawTarget,
    method_name: as_gray8,
    color: Gray8,
    mode: crate::ColorMode::Gray8,
    bpp: 1,
    write_pixel: |color, buf| {
        buf[0] = color.into_storage();
    },
    write_bytes: |color| {
        [color.into_storage()]
    },
    is_uniform: |bytes| {
        Some(bytes[0]) // Always uniform for Gray8
    },
}
