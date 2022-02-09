# bltouch-gcode-ctrl
# `SW fro blackpill board to controll 3dtouch with gcode`


## This is a Rust repository
To compile it, you must install Rust first, follow this link:
https://www.rust-lang.org/tools/install

As the project is an embedded one, more additions are needed, at least those and probably some more:

rustup target install thumbv7em-none-eabihf

cargo install probe-run

cargo install flip-link  (recomended)

The embedded book has lot's of good informations:
https://doc.rust-lang.org/stable/embedded-book/


## Build it
cargo build

## Download and run it
cargo run   or  cargo run --release

## Connection for downloading
ST-Link V2 should be enough

## Connecting the probe with the blackpill
pc13 - probe event signal (my probe has a 5V exit which should be fine for stm32f4xx devices, tested with a small adaption network to limit inputs to 3.3V)
pa8  - servo control pwm to controll 3dtouch
connect power and GND of probe as needed

## G-Code Interface
is build as usb-serial

## developpmentstate
prof of idea

# openpnp configurations
Driver: ttyACMx (on linux)
Baud: 115200
parity: None
Data Bits: Eight
Stop Bits: One
Flow Control: Off

Gcode (driver): 
ACTUATE_COUBLE_COMMAND:  M280 P0 S{DoubleValue}
ACTUATE_BOOLEAN_COMMAND: M280 P0 {True: S10}{False: S160}
ACTUATE_STRING_COMMAND: {StringValue}  // most probably not needed
ACTUATOR_READ_COMMAND: M489
ACTUATOR_READ_REGEX: (?<Value>-?\d+)

Acutuator settings:
Value Type: Double
ON Value 1.000
OFF Value 16.000
// this is due to the fact, that openpnp-python interface always uses the double actuator, as of my experiance

# Binary image
you can also just download the binary image to a blackpill stm32f411-type