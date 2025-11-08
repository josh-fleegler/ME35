import network
import time
from umqtt.simple import MQTTClient
import ssl
import secrets
import neopixel
from machine import Pin, PWM

class MQTTDevice:
    def __init__(self):
        # WiFi credentials
        self.SSID = secrets.SSID
        self.PASSWORD = secrets.PWD

        # NeoPixel: 2 LEDs on pin 15
        self.np = neopixel.NeoPixel(Pin(15), 2)

        # Buzzer setup (optional)
        self.buz = Pin(23, Pin.OUT)
        self.buz.value(0)

        # MQTT setup (TLS)
        self.MQTT_BROKER = secrets.mqtt_url
        self.MQTT_PORT = 8883
        self.MQTT_USERNAME = secrets.mqtt_username
        self.MQTT_PASSWORD = secrets.mqtt_password
        self.CLIENT_ID = "Liam.higgins"
        self.TOPIC_SUB = b"/ME35/17"   # matches the camera publisher topic
        self.TOPIC_PUB = b"/ME35/18"   # optional if you want to send messages back

        # Motor pins (optional — fill these if you have a driver)
        self.L_IN1 = PWM(Pin(12), freq=20000, duty=0)
        self.L_IN2 = PWM(Pin(13), freq=20000, duty=0)
        self.R_IN1 = PWM(Pin(27), freq=20000, duty=0)
        self.R_IN2 = PWM(Pin(14), freq=20000, duty=0)
        self.SPEED = 300

    # ---------- MQTT Setup ----------
    def connect_wifi(self):
        wlan = network.WLAN(network.STA_IF)
        wlan.active(True)
        if not wlan.isconnected():
            print("Connecting to WiFi...")
            wlan.connect(self.SSID, self.PASSWORD)
            timeout = 10
            while not wlan.isconnected() and timeout > 0:
                time.sleep(1)
                timeout -= 1
        if wlan.isconnected():
            print("WiFi Connected:", wlan.ifconfig()[0])
            return True
        print("WiFi failed.")
        return False

    def mqtt_connect(self):
        try:
            self.client = MQTTClient(
                client_id=self.CLIENT_ID,
                server=self.MQTT_BROKER,
                port=self.MQTT_PORT,
                user=self.MQTT_USERNAME,
                password=self.MQTT_PASSWORD,
                ssl=True,
                ssl_params={'server_hostname': self.MQTT_BROKER}  # SNI
            )
            self.client.set_callback(self.sub_cb)
            self.client.connect()
            print("Connected to MQTT broker!")
            self.client.subscribe(self.TOPIC_SUB)
            print("Subscribed to:", self.TOPIC_SUB)
            return True
        except Exception as e:
            print("MQTT connect failed:", e)
            return False

    # ---------- Direction Handling ----------
    def sub_cb(self, topic, msg):
        """Handle messages like LEFT, RIGHT, CENTER, NONE."""
        cmd = msg.decode().strip().upper()
        print("Received:", cmd)

        # Set LED color feedback
        if cmd == "LEFT":
            self.set_led((255, 0, 0))   # blue = left
            #self.buzzer_beep()
            self.spin_left()
        elif cmd == "RIGHT":
            self.set_led((255, 0, 0))   # red = right
            #self.buzzer_beep()
            self.spin_right()
        elif cmd == "CENTER":
            self.set_led((0, 255, 0))   # green = center
            self.forward()
        elif cmd == "NONE":
            self.set_led((0, 0, 0))     # off
            self.stop()
        else:
            print("Unknown command:", cmd)

    def set_led(self, color):
        """Set both NeoPixels to given RGB tuple."""
        for i in range(2):
            self.np[i] = color
        self.np.write()

    def buzzer_beep(self, duration=0.1):
        self.buz.value(1)
        time.sleep(duration)
        self.buz.value(0)

    # Optional motor controls if you hook up a driver:
    def spin_left(self):
        self.L_IN1.duty(0); self.L_IN2.duty(self.SPEED)
        self.R_IN1.duty(self.SPEED); self.R_IN2.duty(0)
    def spin_right(self):
        self.L_IN1.duty(self.SPEED); self.L_IN2.duty(0)
        self.R_IN1.duty(0); self.R_IN2.duty(self.SPEED)
    def forward(self):
        self.L_IN1.duty(0); self.L_IN2.duty(0)
        self.R_IN1.duty(0); self.R_IN2.duty(0)
    def stop(self):
        self.L_IN1.duty(0); self.L_IN2.duty(0)
        self.R_IN1.duty(0); self.R_IN2.duty(0)

# ---------- MAIN LOOP ----------
mqtt_obj = MQTTDevice()

if mqtt_obj.connect_wifi() and mqtt_obj.mqtt_connect():
    while True:
        try:
            mqtt_obj.client.check_msg()  # non-blocking
            time.sleep(0.05)
        except Exception as e:
            print("Error in loop:", e)
            time.sleep(1)
            
            
