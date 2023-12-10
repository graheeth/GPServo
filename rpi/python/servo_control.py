import wiringpi

# GPIO setup
SERVO_PIN1 = 13  # WiringPi pin 1 (BCM GPIO 12)
SERVO_PIN2 = 12  # WiringPi pin 2 (BCM GPIO 13)

# Initialize WiringPi
wiringpi.wiringPiSetupGpio()

# Set the pins to PWM output
wiringpi.pinMode(SERVO_PIN1, wiringpi.GPIO.PWM_OUTPUT)
wiringpi.pinMode(SERVO_PIN2, wiringpi.GPIO.PWM_OUTPUT)

# Set the PWM mode to milliseconds stype
wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)

# Divide down clock
wiringpi.pwmSetClock(192)
wiringpi.pwmSetRange(2000)

# Constants
SERVO_PIN1 = 12  # Example pin number, change according to your setup
SERVO_PIN2 = 13  # Example pin number, change according to your setup
SMOOTHING_STEP = 1  # Small step size for smoothing
UPDATE_INTERVAL = 0.01  # Interval between updates in seconds

current_angle1 = 45  # Initial angle for servo 1
current_angle2 = 90  # Initial angle for servo 2

def set_servo1_angle(servo_pin, angle):
    # Convert angle to PWM value
    print("angle1",angle)

    pwm_value = int((angle / 180.0) * 200 + 50)
    wiringpi.pwmWrite(servo_pin, pwm_value)

def set_servo2_angle(servo_pin, angle):
    # Convert angle to PWM value
    print("angle2",angle)
    pwm_value = int((angle / 180.0) * 200 + 50)
    wiringpi.pwmWrite(servo_pin, pwm_value)

def smooth_move_servo(set_servo_angle, servo_pin, current_angle, target_angle):
    # while abs(target_angle - current_angle) > SMOOTHING_STEP:
    #     # Move in small steps towards the target angle
    #     step = SMOOTHING_STEP if target_angle > current_angle else -SMOOTHING_STEP
    #     current_angle += step

    # set_servo_angle(servo_pin, current_angle)
        # time.sleep(UPDATE_INTERVAL)

    # Set final angle
    set_servo_angle(servo_pin, target_angle)

def move_servos(angle1, angle2):
    global current_angle1, current_angle2

    smooth_move_servo(set_servo1_angle, SERVO_PIN1, current_angle1, angle1 +90 )
    smooth_move_servo(set_servo2_angle, SERVO_PIN2, current_angle2, angle2 +45)

    current_angle1 = angle1 + 45
    current_angle2 = angle2 - 90
