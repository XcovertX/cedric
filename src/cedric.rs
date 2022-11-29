// Example of running an ST7735 with an RP2040

#![no_std]
#![no_main]

// The macro for our start-up function
use tinybmp::Bmp;
use tinybmp::DynamicBmp;
use tinytga::DynamicTga;
use embedded_graphics::primitives::Circle;
use embedded_graphics::primitives::Rectangle;
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use defmt_rtt as _;
use panic_probe as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
//use cortex_m::prelude::*;
use embedded_graphics::image::{Image, ImageRaw, ImageRawLE};
use embedded_graphics::prelude::*;
use embedded_graphics::pixelcolor::*;
use embedded_graphics::pixelcolor::raw::*;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::Extensions;
use rp2040_hal::clocks::Clock;
use rp2040_hal::gpio::{Pin, PushPullOutput, Output, PushPull, PinId};
use st7735_lcd;
use st7735_lcd::Orientation;
use  embedded_graphics::{  primitives::{Line, PrimitiveStyle} };

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    text::{Text, Alignment},
};
use tinytga::Tga;
// Create a new character style

// Create a text at position (20, 30) and draw it using the previously defined style

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

use core::convert::TryInto;

use embedded_graphics::iterator::contiguous::IntoPixels;
/// SPI communication error
#[derive(Debug)]
struct CommError;

/// A fake 64px x 64px display.
struct ExampleDisplay {
    /// The framebuffer with one `u8` value per pixel.
    framebuffer: [u16; 129*130],
}

impl ExampleDisplay {
    /// Updates the display from the framebuffer.
    pub fn flush<A:embedded_hal::blocking::spi::write::Default<u8>,
    B: cortex_m::prelude::_embedded_hal_digital_OutputPin,
    C: cortex_m::prelude::_embedded_hal_digital_OutputPin>(&self,   to : &mut st7735_lcd::ST7735<A,B,C> ) -> Result<(), CommError> {
       
       Ok(())
    }
}

impl DrawTarget for ExampleDisplay {
    type Color = Rgb565;
    // `ExampleDisplay` uses a framebuffer and doesn't need to communicate with the display
    // controller to draw pixel, which means that drawing operations can never fail. To reflect
    // this the type `Infallible` was chosen as the `Error` type.
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            // Check if the pixel coordinates are out of bounds (negative or greater than
            // (63,63)). `DrawTarget` implementation are required to discard any out of bounds
            // pixels without returning an error or causing a panic.
            
            if let Ok((x @ 0..=127, y @ 0..=127)) = coord.try_into() {
                // Calculate the index in the framebuffer.
                let index: u32 = x + y * 128;
                let c = RawU16::from(color).into_inner();
                let mut R = c & 0xF800;
                //r = r << 1;
                //r = r | 0x0800;
                //r &= 0xF800;
                let mut G = c & 0x07E0;
                // G= G >> 1;
                //G = G &  0x07E0;
                let mut B = c & 0x001F;

                let b = B << 11;
                let g = G;
                let r = R >> 11;
                //let z = 0b0111101111111111;
                self.framebuffer[index as usize] = b | g | r;
            }
        }

        Ok(())
    }
}

impl OriginDimensions for ExampleDisplay {
    fn size(&self) -> Size {
        Size::new(128, 128)
    }
}

struct Quote <'a>{    
    image: &'a Image<'a, Bmp<'a, Rgb565>>,
    text: &'a Text<'a, MonoTextStyle<'a,  Rgb565>>,
}

pub fn make_text (x: &str) -> Text<MonoTextStyle<Rgb565>>{
    Text::with_alignment(
        x,
        Point::new(64, 60),
        MonoTextStyle::new(&FONT_6X10, Rgb565::BLACK),
        Alignment::Center,
    ) 
}


fn draw_quote<A:embedded_hal::blocking::spi::write::Default<u8>,
P1: PinId,
P2:PinId>(quote: &Quote, display: &mut ExampleDisplay, disp: &mut st7735_lcd::ST7735<A, Pin<P1, Output<PushPull>>, Pin<P2, Output<PushPull>>>) {
    quote.image.draw(display).unwrap();
    quote.text.draw(display).unwrap();
    // Create a text at position (20, 30) and draw it using the previously defined style

    disp.set_offset(3, 2);
    disp.set_pixels(0, 0, 127, 127, display.framebuffer.map(|f| RawU16::from(f).into_inner()));

}

pub enum Direction {
    Clockwise,
    CounterClockwise,
    None,
}

impl From<u8> for Direction {
    fn from(s: u8) -> Self {
        match s {
            0b0001 | 0b0111 | 0b1000 | 0b1110 => Direction::Clockwise,
            0b0010 | 0b0100 | 0b1011 | 0b1101 => Direction::CounterClockwise,
            _ => Direction::None,
        }
    }
}

pub enum RotaryError <A: InputPin,B:InputPin> {
    left(A::Error),
    right(B::Error)
}
pub struct Rotary<A, B> {
    pin_a: A,
    pin_b: B,
    state: u8,
}
impl<A, B> Rotary<A, B>
where
    A: InputPin,
    B: InputPin,
{
    pub fn new(pin_a: A, pin_b: B) -> Self {
        Self {
            pin_a,
            pin_b,
            state: 0u8,
        }
    }
    pub fn update(&mut self) -> Result<Direction, RotaryError<A,B>> {
        // use mask to get previous state value
        let mut s = self.state & 0b11;
        // move in the new state
        if self.pin_a.is_low().map_err(|d| RotaryError::left(d))? {
            s |= 0b100;
        }
        if self.pin_b.is_low().map_err(|d| RotaryError::right(d))? {
            s |= 0b1000;
        }
        // shift new to old
        self.state = s >> 2;
        // and here we use the From<u8> implementation above to return a Direction
        Ok(s.into())
    }
    
}


/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let spi = hal::Spi::<_, _, 8>::new(pac.SPI0);

    let mut lcd_led = pins.gpio12.into_push_pull_output();
    let dc      = pins.gpio13.into_push_pull_output();
    let rst = pins.gpio17.into_push_pull_output();
    let button1      = pins.gpio14.into_pull_down_input();
    let button2      = pins.gpio15.into_pull_down_input();

    let mut rotary = Rotary::new(pins.gpio10.into_pull_down_input(),
                            pins.gpio11.into_pull_down_input());

    
    // Exchange the uninitialised SPI driver for an initialised one
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16_000_000u32.Hz(),
        &embedded_hal::spi::MODE_0,
    );

    
    let mut disp = st7735_lcd::ST7735::new(spi, dc, rst, true, false, 129, 130);

    disp.init(&mut delay).unwrap();
    disp.set_orientation(&Orientation::Portrait).unwrap();
    disp.clear(Rgb565::BLACK).unwrap();
    disp.set_offset(10, 0);

    let mut display = ExampleDisplay {
        framebuffer: [0; 16770],
    };
    
    

    
    let bmp_data = include_bytes!("../assets/out.bmp");
    let bmp = Bmp::<>::from_slice(bmp_data).unwrap();

    let witch = Bmp::<>::from_slice(include_bytes!("../assets/witch-raw.bmp")).unwrap();
    let witch_img: Image<_> = Image::new(&witch, Point::new(5, 2));

    let owl = Bmp::<>::from_slice(include_bytes!("../assets/owl-raw.bmp")).unwrap();
    let owl_img: Image<_> = Image::new(&owl, Point::new(5, 2));

    let graham = Bmp::<>::from_slice(include_bytes!("../assets/graham-raw.bmp")).unwrap();
    let graham_img: Image<_> = Image::new(&graham, Point::new(5, 2));


    let image: Image<_> = Image::new(&bmp, Point::new(5, 2));

    let txt =  [
        Quote {
            image: &image,
            text: &make_text("Yonder's the\ncrystal cave.")
        },
        
        Quote {
            image: &graham_img,
            text: &make_text("Gootchy goo-\ntchy goo!")
        },
        Quote {
            image: &owl_img,
            text: &make_text("I'll wait for\nyou here!")
        },
        Quote {
            image: &witch_img,
            text: &make_text("Don't you know\nyou're\ntrespassing?")
        },
        Quote {
            image: &owl_img,
            text: &make_text("A poisonous\nsnake!")
        },
        Quote {
            image: &graham_img,
            text: &make_text("Begone ye\nslithery varmit!")
        }
    ];
    
    

    // Wait until the background and image have been rendered otherwise
    // the screen will show random pixels for a brief moment
    
    let mut i =0;
    let mut button1_pressed = false;
    let mut button2_pressed = false;
    lcd_led.set_low().unwrap();
    draw_quote(&txt[i], &mut display, &mut disp);
    lcd_led.set_high().unwrap();
    loop {
        match rotary.update() {
            Ok(Direction::Clockwise) => {
                if (i == txt.len() -1) {
                    i = 0;
                } else {
                i += 1;
                }
                lcd_led.set_low().unwrap();
                draw_quote(&txt[i], &mut display, &mut disp);
                lcd_led.set_high().unwrap();
            },
            Ok(Direction::CounterClockwise) => {
                if (i == 0) {
                    i = txt.len() - 1;
                } else {
                    i -= 1;
                }
                lcd_led.set_low().unwrap();
                draw_quote(&txt[i], &mut display, &mut disp);
                lcd_led.set_high().unwrap();
            },
            Ok(Direction::None) => {},
            Err(_) => {}

        }
        
        if (button1.is_high().unwrap() && !button1_pressed) {

            button1_pressed = true;
        }
        if (button1.is_low().unwrap() && button1_pressed) {
            button1_pressed = false;
            if (i == txt.len() -1) {
                i = 0;
            } else {
            i += 1;
            }

            lcd_led.set_low().unwrap();
            draw_quote(&txt[i], &mut display, &mut disp);
            lcd_led.set_high().unwrap();
        }
        if (button2.is_high().unwrap() && !button2_pressed) {

            button2_pressed = true;
        }
        if (button2.is_low().unwrap() && button2_pressed) {
            button2_pressed = false;
            if (i == 0) {
                i = txt.len() - 1;
            } else {
                i -= 1;
            }

            lcd_led.set_low().unwrap();
            draw_quote(&txt[i], &mut display, &mut disp);
            lcd_led.set_high().unwrap();
        }

        delay.delay_ms(20);


    }
}

// End of file
