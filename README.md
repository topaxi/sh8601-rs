# SH8601 Driver Crate

An embedded-graphics compatible driver for the SH8601 AMOLED display controller IC.

This driver is not embedded-hal compatible but provides a generic interface for controlling the SH8601 display controller. Different displays can be supported by implementing the `ControllerInterface` and `ResetInterface` traits. This is because the SH8601 is used in different displays with various controller interfaces such as SPI or QSPI. Additionally, the reset pin is controlled via GPIO or I2C GPIO expander.

The driver currently incorporates support for the Waveshare 1.8" AMOLED display out of the box but can be extended to support other displays using the SH8601 controller.

## Usage

1. Implement the `ControllerInterface` trait for the controller driving interface (e.g., QSPI).
2. Implement the `ResetInterface` trait for the Reset pin.
3. Create a display wrapper instance (e.g., `Sh8601Rgb565Display`) or use `Sh8601Driver` directly.
4. Use the driver to draw using `embedded-graphics`.

If you are going to use a heap-allocated framebuffer, you will need to ensure that an allocator is available in your environment. In some crates, this is done by enabling the `alloc` feature.

### Color Mode Support

The driver supports multiple color modes via typed display wrappers:

| Wrapper Type | Color Mode | Bytes/Pixel | 368×448 Buffer | Description |
|--------------|------------|-------------|----------------|-------------|
| `Sh8601Rgb565Display` | Rgb565 | 2 | ~328 KB | **Recommended** - best memory/quality balance |
| `Sh8601Rgb888Display` | Rgb888 | 3 | ~495 KB | Highest color depth |
| `Sh8601Rgb666Display` | Rgb666 | 3 | ~495 KB | 18-bit color |
| `Sh8601Gray8Display` | Gray8 | 1 | ~164 KB | Most memory efficient |

#### Rgb565 Mode (Recommended)

```rust
use sh8601_rs::{Sh8601Rgb565Display, ColorMode, DisplaySize, framebuffer_size};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::pixelcolor::Rgb565;

const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb565);

let mut display = Sh8601Rgb565Display::new_heap::<_, FB_SIZE>(
    controller_driver,
    reset_driver,
    DISPLAY_SIZE,
    delay,
)?;

// Get Rgb565 drawing interface
Circle::new(Point::new(100, 100), 50)
    .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
    .draw(&mut display.as_draw_target())?;

display.flush()?;
```

#### Rgb888 Mode

```rust
use sh8601_rs::{Sh8601Rgb888Display, ColorMode, DisplaySize, framebuffer_size};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::pixelcolor::Rgb888;

const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb888);

let mut display = Sh8601Rgb888Display::new_heap::<_, FB_SIZE>(
    controller_driver,
    reset_driver,
    DISPLAY_SIZE,
    delay,
)?;

// Draw with Rgb888 colors directly
Circle::new(Point::new(100, 100), 50)
    .into_styled(PrimitiveStyle::with_fill(Rgb888::GREEN))
    .draw(&mut display.as_draw_target())?;

display.flush()?;
```

#### Gray8 Mode

```rust
use sh8601_rs::{Sh8601Gray8Display, ColorMode, DisplaySize, framebuffer_size};
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle};
use embedded_graphics::pixelcolor::Gray8;

const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);
const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Gray8);

let mut display = Sh8601Gray8Display::new_heap::<_, FB_SIZE>(
    controller_driver,
    reset_driver,
    DISPLAY_SIZE,
    delay,
)?;

// Get Gray8 drawing interface
Circle::new(Point::new(100, 100), 50)
    .into_styled(PrimitiveStyle::with_fill(Gray8::WHITE))
    .draw(&mut display.as_draw_target())?;

display.flush()?;
```

#### Low-Level API (Sh8601Driver)

For advanced use cases, you can use `Sh8601Driver` directly with manual color mode management:

```rust
use sh8601_rs::{Sh8601Driver, ColorMode, DisplaySize, framebuffer_size};

let mut driver = Sh8601Driver::new_heap::<_, FB_SIZE>(
    controller_driver,
    reset_driver,
    ColorMode::Rgb565,  // Must match draw target
    DISPLAY_SIZE,
    delay,
)?;

// Use .as_rgb565(), .as_rgb888(), .as_rgb666(), or .as_gray8()
let mut draw_target = driver.as_rgb565();
// ... draw ...
draw_target.flush()?;
```

**Important**: When using `Sh8601Driver` directly, the color mode passed to the constructor must match the drawing wrapper method used.

## Examples

See the `examples` directory for a usage example with the WaveShare 1.8" AMOLED Display.

The WaveShare 1.8" AMOLED Display controls the SH8601 via an ESP32-S3 over QSPI and uses an I2C GPIO expander for the reset pin. The example implementation uses a PSRAM heap-allocated framebuffer and DMA for efficient transfers.

The schematic is available here: Waveshare ESP32-S3-Touch-AMOLED-1.8

To run the example, with the WaveShare 1.8" AMOLED Display, clone the project and run following command from the project root:

```bash
cargo run --release --example ws_18in_amoled --features "waveshare_18_amoled"
```
