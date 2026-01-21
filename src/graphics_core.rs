use crate::{ControllerInterface, DrawTarget, ResetInterface, Sh8601Driver};
use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*, primitives::Rectangle};

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
        let width = self.config.width as i32;
        let height = self.config.height as i32;
        let fb = &mut self.framebuffer;

        for Pixel(coord, color) in pixels {
            let x = coord.x;
            let y = coord.y;

            // Single combined bounds check
            if (x | y) < 0 || x >= width || y >= height {
                continue;
            }

            let idx = (y as usize * width as usize + x as usize) * 3;

            if idx + 2 >= fb.len() {
                continue;
            }

            let rgb = color.into_storage();
            fb[idx] = (rgb >> 16) as u8;
            fb[idx + 1] = (rgb >> 8) as u8;
            fb[idx + 2] = rgb as u8;
        }

        Ok(())
    }

    /// Optimized clear - fill entire framebuffer
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let c = color.into_storage();
        let r = (c >> 16) as u8;
        let g = (c >> 8) as u8;
        let b = c as u8;

        fill_framebuffer(&mut self.framebuffer, r, g, b);

        Ok(())
    }

    /// Optimized fill for solid color rectangles
    ///
    /// Instead of doing bound checks for each pixel, this implementation calculates the
    /// drawable area first and then fills each row.
    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let drawable_area = area.intersection(&self.bounding_box());

        if drawable_area.size == Size::zero() {
            return Ok(());
        }

        let display_width = self.config.width as usize;
        let rect_width = drawable_area.size.width as usize;
        let start_x = drawable_area.top_left.x as usize;
        let end_x = start_x + rect_width;

        let c = color.into_storage();
        let r = (c >> 16) as u8;
        let g = (c >> 8) as u8;
        let b = c as u8;

        for y in drawable_area.rows() {
            let row_start_index = coordinates_to_index(start_x, y as usize, display_width);
            let row_end_index = coordinates_to_index(end_x, y as usize, display_width);
            let row = &mut self.framebuffer[row_start_index..row_end_index];

            fill_framebuffer(row, r, g, b);
        }

        Ok(())
    }

    /// Optimized contiguous fill
    ///
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

        // Calculate offset into color iterator for clipped region
        let skip_left = drawable_area.top_left.x as usize - area.top_left.x as usize;
        let skip_top = drawable_area.top_left.y as usize - area.top_left.y as usize;

        let mut colors = colors.into_iter();

        // Skip rows above clipped area
        for _ in 0..(skip_top * area_width) {
            colors.next();
        }

        for y in drawable_area.rows() {
            // Skip pixels left of clipped area
            for _ in 0..skip_left {
                colors.next();
            }

            for x in drawable_area.columns() {
                if let Some(color) = colors.next() {
                    write_framebuffer(
                        &mut self.framebuffer,
                        coordinates_to_index(x as usize, y as usize, display_width),
                        color,
                    );
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

#[inline(always)]
fn write_framebuffer(framebuffer: &mut [u8], index: usize, color: Rgb888) {
    let c = color.into_storage();
    let r = (c >> 16) as u8;
    let g = (c >> 8) as u8;
    let b = c as u8;

    framebuffer[index] = r;
    framebuffer[index + 1] = g;
    framebuffer[index + 2] = b;
}

#[inline(always)]
fn coordinates_to_index(x: usize, y: usize, width: usize) -> usize {
    (y * width + x) * 3
}

#[inline(always)]
fn fill_framebuffer(framebuffer: &mut [u8], r: u8, g: u8, b: u8) {
    // Fast path for uniform colors (black, white, grayscale)
    if r == g && r == b {
        framebuffer.fill(r);
        return;
    }

    for chunk in framebuffer.chunks_exact_mut(3) {
        chunk[0] = r;
        chunk[1] = g;
        chunk[2] = b;
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
