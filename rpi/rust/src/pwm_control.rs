use rppal::pwm::{Channel, Polarity, Pwm};
use std::{error::Error, sync::Mutex};
use once_cell::sync::Lazy;

const PWM_FREQUENCY: f64 = 50.0;
const DUTY_CYCLE_MIN: f64 = 0.05;
const DUTY_CYCLE_MAX: f64 = 0.10;
const DUTY_CYCLE_RANGE: f64 = DUTY_CYCLE_MAX - DUTY_CYCLE_MIN;

// Initialize PWM objects lazily and globally
static PWM1: Lazy<Mutex<Pwm>> = Lazy::new(|| {
    Mutex::new(Pwm::with_frequency(Channel::Pwm0, PWM_FREQUENCY, DUTY_CYCLE_MIN, Polarity::Normal, false).unwrap())
});

static PWM2: Lazy<Mutex<Pwm>> = Lazy::new(|| {
    Mutex::new(Pwm::with_frequency(Channel::Pwm1, PWM_FREQUENCY, DUTY_CYCLE_MIN, Polarity::Normal, false).unwrap())
});

pub fn set_servo_angles(angle1: f64, angle2: f64) -> Result<(), Box<dyn Error>> {
    let duty_cycle1 = DUTY_CYCLE_MIN + (angle1 / 180.0) * DUTY_CYCLE_RANGE;
    let duty_cycle2 = DUTY_CYCLE_MIN + (angle2 / 180.0) * DUTY_CYCLE_RANGE;

    let pwm1 = PWM1.lock().unwrap();
    pwm1.set_duty_cycle(duty_cycle1)?;
    pwm1.enable()?;

    let pwm2 = PWM2.lock().unwrap();
    pwm2.set_duty_cycle(duty_cycle2)?;
    pwm2.enable()?;

    Ok(())
}
