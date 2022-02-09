//! CDC-ACM serial port example using polling in a busy loop.
//! Target board: any STM32F4 with a OTG FS peripheral and a 25MHz HSE crystal
#![no_std]
#![no_main]

use panic_halt as _;

use rtt_target::{rprintln, rtt_init_print};
// use cortex_m_semihosting::{ hprintln}; // debug,

use cortex_m_rt::entry;
use cortex_m::interrupt::{free, Mutex};
use core::cell::{Cell, RefCell};

use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::{pac, prelude::*, timer::Timer}; // , pwm::*
use stm32f4::stm32f411::TIM1;
use usb_device::prelude::*;

use stm32f4xx_hal as hal;
use crate::hal::{
    interrupt,
    // delay::Delay,
    gpio::{gpioc::PC13, Edge, Input, PullUp},
};

// extern crate stm32f4;
// use stm32f4::interrupt;

// use embedded_hal::PwmPin;
use stm32f4xx_hal::dwt::DwtExt;

extern crate gcode;
extern crate arrayvec;
use arrayvec::ArrayVec;
use gcode::{Mnemonic}; // Callbacks, Span,

use core::ops::DerefMut;

use heapless::spsc::Queue;
use heapless::String;

static mut EP_MEMORY: [u32; 1024] = [0; 1024];

const READ_SWITCH:u32 = 489;
const SET_SERVO_POSITION:u32 = 280;
const READ_FW_VERSION:u32 = 115;
const READ_POSITION:u32 = 114;
const SET_MM_MODE:u32 = 21;

// servo duty-cycles for BLTouch (with 200Hz frequency = 5ms)
// 90° = 1473us  = 0.2946  19307:u16 (push pin up)
// 10° = 647us   = 0.1294   8480:u16 (pusch down)
// 60° = 1162us  = 0.2324  15231:u16 (alarm release)
// 120° = 1782us = 0.3564  23357:u16 (self test)
// 160° = 2194us = 0.4388  28757:u16 ( alarm releas and push pin up)
const PUSH_PIN_UP:u16           = 17100;
const PUSH_PIN_DOWN:u16         = 7670;
const RELEASE_ALARM:u16         = 13500;
const SELF_TEST:u16             = 21357;
const RELEASE_ALARM_PUSH_UP:u16 = 26400;

static BLE_EVENT: Mutex<RefCell<Option<PC13<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));
static STATE: Mutex<Cell<BleEventState>> = Mutex::new(Cell::new(BleEventState::Waiting));

#[derive(Clone, Copy)]
enum BleEventState {
    Triggered,
    Waiting,
}


#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp =  cortex_m::peripheral::Peripherals::take().unwrap();


    let rcc = dp.RCC.constrain();

    rtt_init_print!();
    rprintln!("Hello from main!");

    let clocks = rcc
        .cfgr
        .use_hse(25.mhz())
        .sysclk(48.mhz())
        .pclk2(100.mhz())
        .require_pll48clk()
        .freeze();

    // Create a delay abstraction based on DWT cycle counter
    let dwt = cp.DWT.constrain(cp.DCB, &clocks);
    let mut delay = dwt.delay();

    let gpioa = dp.GPIOA.split();

    let channels = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate());

    let pwm = Timer::new(dp.TIM1, &clocks).pwm(channels, 200u32.hz());
    let (mut ch1, _ch2) = pwm;
    // let max_duty = ch1.get_max_duty();

    let mut syscfg = dp.SYSCFG.constrain();

    // Create a button input with an interrupt
    let gpioc = dp.GPIOC.split();
    let mut ble_event = gpioc.pc13.into_pull_up_input();
    ble_event.make_interrupt_source(&mut syscfg);
    ble_event.enable_interrupt(&mut dp.EXTI);
    ble_event.trigger_on_edge(&mut dp.EXTI, Edge::Rising); //Edge::Falling);


    ch1.set_duty(PUSH_PIN_UP); // 1473us (90°)
    ch1.enable();


    let usb = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: gpioa.pa11.into_alternate(),
        pin_dp: gpioa.pa12.into_alternate(),
        hclk: clocks.hclk(),
    };

    

    let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

    let mut serial = usbd_serial::SerialPort::new(&usb_bus);

    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();


    free(|cs| {
        BLE_EVENT.borrow(cs).replace(Some(ble_event));
    });

    // Enable interrupts
    pac::NVIC::unpend(hal::pac::Interrupt::EXTI15_10);
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::EXTI15_10);
    };

    let mut rb: Queue<u8, 256> = Queue::new();
    let mut cmd_s: String<256> = String::new();

    loop {
        if !usb_dev.poll(&mut [&mut serial]) {
            if rb.len() == 0 {
                continue;
            }
            
        }

        let mut buf = [0u8; 64];
        let mut drop : bool = false;
        match serial.read(&mut buf) {
            Ok(count) if count > 0 => {
                for c in buf[0..count].iter() {
                    rprintln!("received: {:#?}", *c);
                    // if *c == 10 {
                    //     break;
                    // }
                    if *c  == 59 {
                        drop = true;
                        rprintln!("drop = true: {:#?}", *c);
                    } 

                    // rprintln!("enque: {:#?}", *c);
                    if drop == false {
                        let  res = rb.enqueue(*c).is_ok();
                        match res {
                            true  => rprintln!("enque: {:#?}", *c),
                            false => {rprintln!("error enque: {:#?}", *c);
                                break;},
                        }
                        // rb.enqueue(*c).unwrap();
                        if rb.is_full() {
                            rprintln!("######## que is full ########");
                            break;
                        }
                    }
                    if *c  == 10 {
                        drop = false;
                        rprintln!("drop = false: {:#?}", *c)
                    }
                    
                }
            }
            _ => {}
        }


        // M119 Read Endstop States
        // M402 Stow Probe
        // M401 Deploy Probe
        // M280 P0 Sxx   Servo Command
        // M289 read switch state of probe
        // M226 P<pin> S<state>
        let q_len = rb.len();
        // rprintln!("q_len start Zeitpunkt {:#?}", q_len);
        if q_len > 0 {
            let mut dropping : bool = false;
            for _i in [0..q_len] {
                // rprintln!("_i: {:#?}", _i);
                let c = rb.dequeue();
                let mut chr: u8 = 10;
                match c {
                    Some(charakter) => chr = charakter,
                    None => rprintln!("######## deque error ########"),
                }
                if dropping & (chr != 10) {
                    continue;
                } else if chr == 10 {
                    dropping = false;
                }
                // rprintln!("que len(): {:#?}", rb.len());
                // if (c.unwrap() != 10) & (c.unwrap() != 13){
                if (chr != 10) & (chr != 13) {
                    if chr != 59 {
                        rprintln!("dequed c: {:#?}", c.unwrap());
                        let _res = cmd_s.push_str(u8tostr(c.unwrap()));
                        rprintln!("cmd_part: {:#?} q-len: {:#?}", cmd_s.as_str(), rb.len());
                    } else {
                        rprintln!("######## dropping false ########");
                        dropping = true;
                    }
                } else {
                    rprintln!("cmd_final: {:#?}", cmd_s.as_str());
                    let gcmds: ArrayVec::<_, 32> = gcode::parse(cmd_s.as_str()).collect();
                    cmd_s.clear();

                    if gcmds.len() > 0 {
                        let gcmd = &gcmds[0];
                        // rprintln!("len:    {}", length);
                        rprintln!("mnemonic:    {}", gcmd.mnemonic());
                        rprintln!("Major:    {}", gcmd.major_number());
                        rprintln!("minor:    {}", gcmd.minor_number());
                        rprintln!("x:    {:#?}",gcmd.value_for('X'));
                        rprintln!("y:    {:#?}",gcmd.value_for('Y'));
                        rprintln!("P:    {:#?}",gcmd.value_for('P'));
                        rprintln!("S:    {:#?}",gcmd.value_for('S'));

                        let mnem   = gcmd.mnemonic();
                        let major  = gcmd.major_number();
                        let _minor  = gcmd.minor_number();
                        let mut p_val:f32  = 0.0;
                        let mut s_val:f32  = 0.0;

                        // delay introduced to hopefully remove lost answers on openpnp python scripts
                        delay.delay_ms(20_u32);

                        if mnem == Mnemonic::Miscellaneous {
                            rprintln!("######## M-Command detected ########");
                            match major {
                                READ_SWITCH => {  // M489
                                    free(|cs| {
                                        let state = STATE.borrow(cs).get();
                                        // let mut buf: [u8;8] = [49, 10, 13, 111, 107, 10, 13, 0];
                                        let mut buf: [u8;8] = [10, 49, 10, 111, 107, 10, 13, 0];
                                        match state {
                                            BleEventState::Triggered => {
                                                rprintln!("Triggered");
                                                STATE.borrow(cs).replace(BleEventState::Waiting);
                                                buf[1] = 49; // write 1 for event
                                            },
                                            BleEventState::Waiting => {
                                                rprintln!("Waiting");
                                                buf[1] = 48; // write 0 for no event
                                            }
                                        }
                                        let mut write_offset = 0;
                                        while write_offset < 8 {
                                            match serial.write(&buf[write_offset..8]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    });
                                    
                                },
                                SET_SERVO_POSITION => {  // M280
                                    let buf: [u8;4] = [10, 111, 107, 10,];
                                    match gcmd.value_for('P') {
                                        Some(val) => p_val = val,
                                        None => continue, // p_val = 0.0,
                                    }
                                    match gcmd.value_for('S') {
                                        Some(val) => s_val = val,
                                        None => continue, // s_val = 0.0,
                                    }
                                    // setServo(&mut ch1, s_val);
                                    let s_val_i32 = s_val as i32;
                                    match s_val_i32 {
                                        10 =>  {
                                                    ch1.set_duty(RELEASE_ALARM_PUSH_UP);
                                                    // wait till potential alarm is release
                                                    delay.delay_ms(200_u32);
                                                    // set automatic in wait for touch event mode
                                                    ch1.set_duty(PUSH_PIN_DOWN)
                                                    },
                                        60 =>  ch1.set_duty(RELEASE_ALARM),  // state, where waitin for touch event
                                        90 =>  ch1.set_duty(PUSH_PIN_UP),
                                        120 => ch1.set_duty(SELF_TEST),
                                        160 => ch1.set_duty(RELEASE_ALARM_PUSH_UP),
                                        _ => {},
                                    }
                                    let mut write_offset = 0;
                                        while write_offset < 4 {
                                            match serial.write(&buf[write_offset..4]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                                },
                                READ_FW_VERSION => {  //M115
                                    // F70 I73 R82 M77 W87 A65 R82 E69
                                    let buf: [u8;20] = [10, 70, 73, 82, 77, 87, 65, 82, 69, 32,
                                                        86, 49, 46, 48, 46, 48, 10, 111, 107, 10,];
                                    let mut write_offset = 0;
                                        while write_offset < 20 {
                                            match serial.write(&buf[write_offset..20]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                                } 
                                // READ_POSITION M114
                                READ_POSITION => {  // M114
                                    // Z90 :58 "0"48 .46 "0"48 
                                    let buf: [u8;20] = [10, 90, 58, 48, 46, 48, 48, 32, 32, 32,
                                                        32, 32, 32, 32, 32, 32, 10, 111, 107, 10,];
                                    let mut write_offset = 0;
                                        while write_offset < 20 {
                                            match serial.write(&buf[write_offset..20]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                                } 
                                _ => {
                                    let buf: [u8;4] = [10, 111, 107, 10,];
                                    let mut write_offset = 0;
                                        while write_offset < 4 {
                                            match serial.write(&buf[write_offset..4]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                                },
                            }
                            
                        }
                        if mnem == Mnemonic::General {
                            rprintln!("######## G-Command detected ########");
                            match major {
                                SET_MM_MODE => {  // G21
                                    let buf: [u8;4] =[ 10, 111, 107, 10,];
                                    let mut write_offset = 0;
                                        while write_offset < 4 {
                                            match serial.write(&buf[write_offset..4]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                                }
                                _ => {
                                    let buf: [u8;4] = [10, 111, 107, 10,];
                                    let mut write_offset = 0;
                                        while write_offset < 4 {
                                            match serial.write(&buf[write_offset..4]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                                },
                            }
                        } else {
                            let buf: [u8;4] = [10, 111, 107, 10,];
                                    let mut write_offset = 0;
                                        while write_offset < 4 {
                                            match serial.write(&buf[write_offset..4]) {
                                                Ok(len) if len > 0 => {
                                                    write_offset += len;
                                                 }
                                                _ => {}
                                            }
                                        }
                                    continue
                        }

                        
                    }
                }
            }

        }
        

    }
}

fn write_buffer(buf: &[u8], data_len: u32) {
    rprintln!("buf.len(): {}", buf.len());
}

#[interrupt]
fn EXTI15_10() {
    free(|cs| {
        let mut ble_event_ref = BLE_EVENT.borrow(cs).borrow_mut();
        if let Some(ref mut ble_event) = ble_event_ref.deref_mut() {
            // We cheat and don't bother checking _which_ exact interrupt line fired - there's only
            // ever going to be one in this example.
            ble_event.clear_interrupt_pending_bit();

            

            unsafe { 
                // (*<TIM1>::ptr()).ccr1.write(|w| w.bits(RELEASE_ALARM_PUSH_UP.into())); // duty.into() 
                (*<TIM1>::ptr()).ccr1.write(|w| w.bits(PUSH_PIN_UP.into()));
            }

            let state = STATE.borrow(cs).get();
            // // Run the state machine in an ISR - probably not something you want to do in most
            // // cases but this one only starts and stops TIM2 interrupts
            match state {
                BleEventState::Waiting => {
                    // stopwatch_start(cs);
                    STATE.borrow(cs).replace(BleEventState::Triggered);
                }
                BleEventState::Triggered => {
                    // stopwatch_stop(cs);
                    // STATE.borrow(cs).replace(StopwatchState::Stopped);
                }
            }
        }
    });
    // rprintln!("Hello from Exti 15");
}

// note: expected mutable reference `&mut stm32f4xx_hal::pwm::PwmChannel<TIM1, stm32f4xx_hal::pwm::C1>`
// found reference `&stm32f4xx_hal::pwm::PwmChannel<TIM, CHANNEL>`

// fn setServo<TIM1, C1>(mut ch : &PwmChannel<TIM1, C1>, s: f32) where PwmChannel<TIM1, C1>: embedded_hal::PwmPin {
//     rprintln!("setServo called:    S{}", s);
//     // servo duty-cycles for BLTouch (with 200Hz frequency = 5ms)
//     // 90° = 1473us  = 0.2946  19307:u16 (push pin up)
//     // 10° = 647us   = 0.1294   8480:u16 (pusch down)
//     // 60° = 1162us  = 0.2324  15231:u16 (alarm release)
//     // 120° = 1782us = 0.3564  23357:u16 (self test)
//     // 160° = 2194us = 0.4388  28757:u16 ( alarm releas and push pin up)
//     if s < 12.0 {
//         // ch.set_duty(PUSH_PIN_UP);
//         // <PwmChannel<TIM1, C1>>::set_duty(&mut ch, PUSH_PIN_UP);
//         let max_duty = <PwmChannel<TIM1, C1> as embedded_hal::PwmPin>::get_max_duty(&ch);
//         <PwmChannel<TIM1, C1> as PwmPin>::set_duty(&mut ch, max_duty);
            
//     } 
    
// }

fn u8tostr(c: u8) -> &'static str {
    match c {
        71 => r"G",
        77 => r#"M"#,
        83 => r#"S"#,
        80 => r#"P"#,
        59 => r#";"#,
        48 => r#"0"#,
        49 => r#"1"#,
        50 => r#"2"#,
        51 => r#"3"#,
        52 => r#"4"#,
        53 => r#"5"#,
        54 => r#"6"#,
        55 => r#"7"#,
        56 => r#"8"#,
        57 => r#"9"#,
        32 => r#" "#,
        13 => "\r",
        10 => "\n",
        _ => r#""#,
    }
}
