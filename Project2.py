from machine import Pin, PWM, I2C
import time
import machine
import lis3dh
import servo


ServoLower1 = servo.Servo(19)
ServoUpper2 = servo.Servo(18)

button_record = machine.Pin(34, machine.Pin.IN, machine.Pin.PULL_UP)
button_play = machine.Pin(35, machine.Pin.IN, machine.Pin.PULL_UP)
# -------------------
# Encoder / Motor classes
# -------------------

class Count:
    def __init__(self, A, B):
        self.A = Pin(A, Pin.IN)
        self.B = Pin(B, Pin.IN)
        self.counter = 0
        self.A.irq(self.cb, self.A.IRQ_FALLING | self.A.IRQ_RISING)
        self.B.irq(self.cb, self.B.IRQ_FALLING | self.B.IRQ_RISING)

    def cb(self, msg):
        other, inc = (self.B, 1) if msg == self.A else (self.A, -1)
        self.counter += -inc if msg.value() != other.value() else inc

    def value(self):
        return self.counter

class Motor(Count):
    def __init__(self, m1, m2, A, B):
        self.enc = Count(A, B)
        self.M1 = PWM(Pin(m1), freq=100, duty_u16=0)
        self.M2 = PWM(Pin(m2), freq=100, duty_u16=0)
        self.stop()

    def pos(self):
        return self.enc.value()

    def stop(self):
        self.M1.duty_u16(0)
        self.M2.duty_u16(0)

    def start(self, direction=0, speed=50):
        duty = int(speed * 65535 / 100)
        if direction:
            self.M1.duty_u16(duty)
            self.M2.duty_u16(0)
        else:
            self.M1.duty_u16(0)
            self.M2.duty_u16(duty)
            
MotorLower1 = Motor(14, 27, 32, 39)
MotorUpper2 = Motor(13, 12, 26, 25)

# -------------------
# Servo helper
# -------------------
"""class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50)

    def write_angle(self, angle):
        min_us = 500
        max_us = 2500
        us = min_us + (angle / 180) * (max_us - min_us)
        duty = int(us / 20000 * 65535)
        self.pwm.duty_u16(duty)
"""
# -------------------
# Hardware setup
# -------------------

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
accel = lis3dh.H3LIS331DL(sda_pin=21, scl_pin=22)



# -------------------
# Dataset
# -------------------
dataset = []

# -------------------
# Play mode flag
# -------------------
play_mode = False

def toggle_play(pin):
    global play_mode
    play_mode = not play_mode
    print("Play mode:", play_mode)

button_play.irq(trigger=Pin.IRQ_FALLING, handler=toggle_play)

# -------------------
# Training mode function
# -------------------
def training_mode():
    print("=== Training Mode ===")
    while not play_mode:
        motor_lower_pos = MotorLower1.pos()
        motor_upper_pos = MotorUpper2.pos()

        servo_lower_angle = round((motor_lower_pos % 3840/21.33)) // 30 * 30
        servo_upper_angle = round((motor_upper_pos % 3840/21.33)) // 30 * 30

        ServoLower1.write_angle(servo_lower_angle)
        ServoUpper2.write_angle(servo_upper_angle)

        if button_record.value() == 0:
            #time.sleep(0.1)  # debounce
            if button_record.value() == 0:
                
                accl_g = accel.read_accl_g()
                accel_x = accl_g['x']
                accel_z = accl_g['z']
                data_point = (servo_lower_angle, servo_upper_angle, accel_x, accel_z)
                dataset.append(data_point)
                print("Data recorded:", data_point)
                print("Dataset length:", len(dataset))

                #while button_record.value() == 1:
                time.sleep(0.05)
        time.sleep(0.05)

# -------------------
# Play mode function
# -------------------
i2c = I2C(0, scl=Pin(22), sda=Pin(21))
accel = lis3dh.H3LIS331DL(sda_pin=21, scl_pin=22)

def nearest_neighbor(accel_x, accel_z, dataset):
    if not dataset:
        return None
    return min(
        dataset,
        key=lambda d: (d[2] - accel_x) ** 2 + (d[3] - accel_z) ** 2
        )

def play_mode_loop():
    print("=== Play Mode ===")
    while play_mode:
        accl_g = accel.read_accl_g()
        accel_x = accl_g['x']
        accel_z = accl_g['z']
        print("Accel x, z:", accel_x, accel_z)
        
        nearest = nearest_neighbor(accel_x, accel_z, dataset)

        #if dataset:
        """nearest = min(dataset, key=lambda d: abs(d[2]-accel_x) + abs(d[3]-accel_z))
        servo_lower_angle = nearest[0]
        servo_upper_angle = nearest[1]
        ServoLower1.write_angle(servo_lower_angle)
        ServoUpper2.write_angle(servo_upper_angle)
        print("Nearest servo angles:", servo_lower_angle, servo_upper_angle)"""
        servo_lower_angle, servo_upper_angle = nearest[0], nearest[1]
        ServoLower1.write_angle(servo_lower_angle)
        ServoUpper2.write_angle(servo_upper_angle)
        print("Nearest servo angles:", servo_lower_angle, servo_upper_angle)

        #else:
            #print("No training data available!")

        time.sleep(0.1)  # sample every 5 seconds
# -------------------
# Main loop
# -------------------
print("Press D34 to record training data, D35 to toggle play mode")

while True:
    if play_mode:
        play_mode_loop()
    else:
        training_mode()
