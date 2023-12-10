// setting pwm start on boot for rpi is necessary
use rppal::pwm::{Channel, Polarity, Pwm};
use std::error::Error;

const PWM_FREQUENCY: f64 = 50.0;  // Standard for servos
const DUTY_CYCLE_MIN: f64 = 0.05; // 5% duty cycle, typically 0 degrees
const DUTY_CYCLE_MAX: f64 = 0.10; // 10% duty cycle, typically 180 degrees

pub fn set_servo_angles(angle1: f64, angle2: f64) -> Result<(), Box<dyn Error>> {
    if angle1 < 0.0 || angle1 > 180.0 || angle2 < 0.0 || angle2 > 180.0 {
        return Err("Angles must be between 0 and 180".into());
    }

    let pwm1 = Pwm::with_frequency(Channel::Pwm0, PWM_FREQUENCY, duty_cycle1, Polarity::Normal, false)?;
    pwm1.enable()?;

    let pwm2 = Pwm::with_frequency(Channel::Pwm1, PWM_FREQUENCY, duty_cycle2, Polarity::Normal, false)?;
    pwm2.enable()?;

    // Set angle for servo on pin 12 (GPIO18)
    let duty_cycle1 = DUTY_CYCLE_MIN + (angle1 / 180.0) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN);
    let mut pwm1 = Pwm::with_frequency(Channel::Pwm0, PWM_FREQUENCY, duty_cycle1, Polarity::Normal, false)?;
    pwm1.enable()?;

    // Set angle for servo on pin 33 (GPIO13)
    let duty_cycle2 = DUTY_CYCLE_MIN + (angle2 / 180.0) * (DUTY_CYCLE_MAX - DUTY_CYCLE_MIN);
    let mut pwm2 = Pwm::with_frequency(Channel::Pwm1, PWM_FREQUENCY, duty_cycle2, Polarity::Normal, false)?;
    pwm2.enable()?;

    Ok(())
}
