//! DrawTarget implementation for the SH8601 driver.
//!
//! This module provides the embedded-graphics `DrawTarget` implementation
//! using the `ColorConfig` trait for compile-time color mode configuration.

use crate::{ColorConfig, ControllerInterface, ResetInterface, Sh8601Driver};
use embedded_graphics_core::{
    prelude::*,
    primitives::Rectangle,
};

// =========== DrawTarget Implementation ===========

impl<IFACE, RST, C> DrawTarget for Sh8601Driver<IFACE, RST, C>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
    C: ColorConfig,
{
    type Color = C::Color;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let width = self.config.width as i32;
        let height = self.config.height as i32;
        let bpp = C::BYTES_PER_PIXEL;

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
            let idx = (y as usize * width as usize + x as usize) * bpp;

            C::write_pixel(color, &mut self.framebuffer[idx..idx + bpp]);
        }

        Ok(())
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let display_area = Rectangle::new(
            Point::zero(),
            Size::new(self.config.width as u32, self.config.height as u32),
        );
        let area = match area.intersection(&display_area) {
            a if a.size.width == 0 || a.size.height == 0 => return Ok(()),
            a => a,
        };

        let bytes = C::to_bytes(color);
        let bytes_ref = bytes.as_ref();
        let bpp = C::BYTES_PER_PIXEL;

        let width = self.config.width as usize;
        let start_x = area.top_left.x as usize;
        let start_y = area.top_left.y as usize;
        let rect_width = area.size.width as usize;
        let rect_height = area.size.height as usize;

        // Fast path for uniform colors (e.g., grayscale)
        if let Some(uniform_byte) = C::is_uniform(&bytes) {
            for row in 0..rect_height {
                let y = start_y + row;
                let row_start = (y * width + start_x) * bpp;
                let row_end = row_start + rect_width * bpp;
                self.framebuffer[row_start..row_end].fill(uniform_byte);
            }
            return Ok(());
        }

        // General path
        for row in 0..rect_height {
            let y = start_y + row;
            let row_start = (y * width + start_x) * bpp;
            let row_end = row_start + rect_width * bpp;

            let row_slice = &mut self.framebuffer[row_start..row_end];
            for chunk in row_slice.chunks_exact_mut(bpp) {
                chunk.copy_from_slice(bytes_ref);
            }
        }

        Ok(())
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let display_area = Rectangle::new(
            Point::zero(),
            Size::new(self.config.width as u32, self.config.height as u32),
        );
        let clipped = area.intersection(&display_area);
        if clipped.size.width == 0 || clipped.size.height == 0 {
            return Ok(());
        }

        let bpp = C::BYTES_PER_PIXEL;
        let width = self.config.width as usize;
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
            let row_start = (y * width + start_x) * bpp;

            for col in 0..clipped_width {
                if let Some(color) = colors.next() {
                    let idx = row_start + col * bpp;
                    C::write_pixel(color, &mut self.framebuffer[idx..idx + bpp]);
                }
            }

            let skip_right = area_width - skip_left - clipped_width;
            for _ in 0..skip_right {
                colors.next();
            }
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let bytes = C::to_bytes(color);
        let bytes_ref = bytes.as_ref();
        let bpp = C::BYTES_PER_PIXEL;

        // Fast path for uniform colors (e.g., grayscale)
        if let Some(uniform_byte) = C::is_uniform(&bytes) {
            self.framebuffer.as_mut_slice().fill(uniform_byte);
            return Ok(());
        }

        // General path
        for chunk in self.framebuffer.as_mut_slice().chunks_exact_mut(bpp) {
            chunk.copy_from_slice(bytes_ref);
        }

        Ok(())
    }
}

impl<IFACE, RST, C> OriginDimensions for Sh8601Driver<IFACE, RST, C>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
    C: ColorConfig,
{
    fn size(&self) -> Size {
        Size::new(self.config.width as u32, self.config.height as u32)
    }
}
