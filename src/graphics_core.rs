use crate::{ControllerInterface, DrawTarget, ResetInterface, Sh8601Driver};
use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*};

impl<IFACE, RST> DrawTarget for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = Rgb888;
    // Drawing to the framebuffer in memory is infallible.
    // Errors happen during flush with SPI comms.
    type Error = core::convert::Infallible;

    /// Draws a single pixel to the internal framebuffer.
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < self.config.width as i32
                && coord.y >= 0
                && coord.y < self.config.height as i32
            {
                let x = coord.x as u32;
                let y = coord.y as u32;
                let index = ((y * self.config.width as u32 + x) * 3) as usize;

                if index + 2 < self.framebuffer.len() {
                    let c = color.into_storage();
                    let r = (c >> 16) as u8; // 8-bit Red
                    let g = (c >> 8) as u8; // 8-bit Green
                    let b = c as u8; // 8-bit Blue

                    self.framebuffer[index] = r;
                    self.framebuffer[index + 1] = g;
                    self.framebuffer[index + 2] = b;
                }
            }
        }
        Ok(())
    }

    /// Optimized clear - fill entire framebuffer
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let c = color.into_storage();
        let r = (c >> 16) as u8;
        let g = (c >> 8) as u8;
        let b = c as u8;

        // Fast path for uniform colors (black, white, grayscale)
        if r == g && r == b {
            self.framebuffer.fill(r);

            return Ok(());
        }

        for chunk in self.framebuffer.chunks_exact_mut(3) {
            chunk[0] = r;
            chunk[1] = g;
            chunk[2] = b;
        }

        Ok(())
    }

    /// Optimized contiguous fill
    /// The trait requires the provided iterator to provide pixel color values in order from top
    /// left to the bottom right corner in a row-first order, this allows for more efficient
    /// filling than using `draw_iter` which bound checks each pixel coordinate individually.
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let drawable_area = area.intersection(&self.bounding_box());

        if drawable_area.size == Size::zero() {
            return Ok(());
        }

        let display_width = self.config.width as usize;
        let area_width = area.size.width as usize;
        let clipped_width = drawable_area.size.width as usize;
        let clipped_height = drawable_area.size.height as usize;

        // Calculate offset into color iterator for clipped region
        let skip_left = (drawable_area.top_left.x - area.top_left.x) as usize;
        let skip_top = (drawable_area.top_left.y - area.top_left.y) as usize;

        let mut colors = colors.into_iter();

        // Skip rows above clipped area
        for _ in 0..(skip_top * area_width) {
            colors.next();
        }

        let start_x = drawable_area.top_left.x as usize;
        let start_y = drawable_area.top_left.y as usize;

        for row in 0..clipped_height {
            // Skip pixels left of clipped area
            for _ in 0..skip_left {
                colors.next();
            }

            let y = start_y + row;
            let row_start = (y * display_width + start_x) * 3;

            for col in 0..clipped_width {
                if let Some(color) = colors.next() {
                    let idx = row_start + col * 3;
                    let c = color.into_storage();
                    let r = (c >> 16) as u8;
                    let g = (c >> 8) as u8;
                    let b = c as u8;

                    self.framebuffer[idx] = r;
                    self.framebuffer[idx + 1] = g;
                    self.framebuffer[idx + 2] = b;
                }
            }

            // Skip pixels right of clipped area
            let skip_right = area_width - skip_left - clipped_width;
            for _ in 0..skip_right {
                colors.next();
            }
        }

        Ok(())
    }
}

// =========== embedded-graphics OriginDimensions Implementation ===========

impl<IFACE, RST> OriginDimensions for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    fn size(&self) -> Size {
        Size::new((self.config.width) as u32, (self.config.height) as u32)
    }
}
