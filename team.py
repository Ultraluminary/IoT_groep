import time
from bmp280 import BMP280
from smbus2 import SMBus, i2c_msg
from datetime import datetime, timedelta
import paho.mqtt.client as mqtt
import wiringpi
import requests

# Create an I2C bus object for BMP280 and BH1750
bus = SMBus(0)

# BMP280 setup
bmp280_address = 0x77
bmp280 = BMP280(i2c_addr=bmp280_address, i2c_dev=bus)
interval = 15  # Sample period in seconds

# BH1750 setup
bh1750_address = 0x23
bus.write_byte(bh1750_address, 0x10)

# MQTT settings
MQTT_CLIENT_ID = "KRMcKzkpOBcZIi4VEzAZIjA"  # Vervang door jouw ThingSpeak MQTT Client ID
MQTT_USER = "KRMcKzkpOBcZIi4VEzAZIjA"        # Vervang door jouw ThingSpeak MQTT Username
MQTT_PWD = "8uzNOWNKQrd1w7LqR0ft1VNt"        # Vervang door jouw ThingSpeak MQTT Password
MQTT_HOST = "mqtt3.thingspeak.com"
MQTT_PORT = 1883
MQTT_TOPIC_MEASURED = "channels/2792379/publish"  # Kanaal-ID voor gemeten waarden
MQTT_TOPIC_DESIRED = "channels/2792381/publish"  # Kanaal-ID voor gewenste waarden

# ThingSpeak settings
THINGSPEAK_CHANNEL_ID_READ = "2792381"  # Kanaal-ID voor gewenste waarden
THINGSPEAK_CHANNEL_ID_WRITE = "2792379"  # Kanaal-ID voor gemeten waarden
THINGSPEAK_READ_API_KEY = "94U2IKT6YRPREMPN"
THINGSPEAK_WRITE_API_KEY_MEASURED = "LKPW521GXHAU6G31"
THINGSPEAK_WRITE_API_KEY_DESIRED = "2HBKEUX8ENIXKAGX"

# GPIO setup for LED and buttons
wiringpi.wiringPiSetup()
LED_PIN = 2  # LED pin
BUTTON_DEC_PIN = 3  # Wiring 3 -> Physical Pin 8
BUTTON_INC_PIN = 4  # Wiring 4 -> Physical Pin 10
BUTTON_LED_TOGGLE_PIN = 6  # Wiring 6 -> Physical Pin 12

wiringpi.pinMode(LED_PIN, wiringpi.OUTPUT)
wiringpi.pinMode(BUTTON_DEC_PIN, wiringpi.INPUT)
wiringpi.pullUpDnControl(BUTTON_DEC_PIN, wiringpi.PUD_UP)
wiringpi.pinMode(BUTTON_INC_PIN, wiringpi.INPUT)
wiringpi.pullUpDnControl(BUTTON_INC_PIN, wiringpi.PUD_UP)
wiringpi.pinMode(BUTTON_LED_TOGGLE_PIN, wiringpi.INPUT)
wiringpi.pullUpDnControl(BUTTON_LED_TOGGLE_PIN, wiringpi.PUD_UP)

# Default values (fallbacks)
bmp280_temperature = 2000  # Default temperature (°C)
TEMPERATURE_GOAL = 2500     # Default temperature goal (°C)
lux = 100                 # Default light intensity (Lux)
LED_BRIGHTNESS = 50       # Default LED brightness
LUX_GOAL = 200            # Default lux goal for LED
bmp280_pressure = 1013    # Default pressure (hPa)
manual_led_override = False  # Default to automatic mode
manual_led_status = False    # Default LED state is OFF

# Button state variables
previous_button_state = {
    BUTTON_DEC_PIN: wiringpi.HIGH,
    BUTTON_INC_PIN: wiringpi.HIGH,
    BUTTON_LED_TOGGLE_PIN: wiringpi.HIGH,
}

# Functions for MQTT
def on_connect(client, userdata, flags, rc):
    print("Connected OK" if rc == 0 else f"Bad connection: {rc}")

def on_disconnect(client, userdata, flags, rc=0):
    print(f"Disconnected: {rc}")

# Function to fetch the last entry from ThingSpeak
def fetch_last_entry():
    global LUX_GOAL
    api_url = f"https://api.thingspeak.com/channels/{THINGSPEAK_CHANNEL_ID_READ}/feeds/last.json"
    params = {'api_key': THINGSPEAK_READ_API_KEY}
    try:
        response = requests.get(api_url, params=params)
        if response.status_code == 200:
            data = response.json()

            # Fetch only the required field
            LUX_GOAL = int(float(data.get('field7', LUX_GOAL)))  # Field 7: LED Light Goal
            raw_timestamp = data.get('created_at', 'Unknown Time')

            # Convert timestamp to local time (GMT+1)
            if raw_timestamp != 'Unknown Time':
                utc_time = datetime.strptime(raw_timestamp, "%Y-%m-%dT%H:%M:%SZ")
                local_time = utc_time + timedelta(hours=1)  # Adjust to GMT+1
                date = local_time.strftime("%Y-%m-%d")
                time = local_time.strftime("%H:%M:%S")
            else:
                date, time = "Unknown Date", "Unknown Time"

            # Print fetched values for debugging
            print("==========================================")
            print("Initial Values Fetched from ThingSpeak:")
            print("------------------------------------------")
            print(f"LED Light Goal: {LUX_GOAL} Lux")
            print(f"Date: {date}")
            print(f"Time: {time}")
            print("==========================================")
        else:
            print(f"Failed to fetch last entry from ThingSpeak. Status code: {response.status_code}")
    except Exception as e:
        print(f"Error fetching last entry: {e}")

# Fetch the last entry from ThingSpeak before starting the main loop
print("Fetching initial values from ThingSpeak...")
fetch_last_entry()

# Function to get light value from BH1750
def get_light_value(bus, address):
    write = i2c_msg.write(address, [0x10])
    read = i2c_msg.read(address, 2)
    bus.i2c_rdwr(write, read)
    bytes_read = list(read)
    return int((((bytes_read[0] & 3) << 8) + bytes_read[1]) / 1.2)

# MQTT Client setup
client = mqtt.Client(client_id=MQTT_CLIENT_ID)
client.username_pw_set(MQTT_USER, MQTT_PWD)
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.loop_start()
client.connect(MQTT_HOST, MQTT_PORT)

# Initialize variables for LED brightness
LED_BRIGHTNESS = 100  # Default brightness
BRIGHTNESS_STEP = 20  # Step size for brightness adjustment
MAX_BRIGHTNESS = 100  # Maximum brightness
MIN_BRIGHTNESS = 0    # Minimum brightness

# Initialize softPWM on LED_PIN
wiringpi.softPwmCreate(LED_PIN, 0, MAX_BRIGHTNESS)  # Range is 0-100
wiringpi.softPwmWrite(LED_PIN, LED_BRIGHTNESS)      # Start with full brightness

# Function to send data to ThingSpeak
def send_data_to_thingspeak(topic, write_key, value):
    MQTT_DATA = f"field7={value}&status=MQTTPUBLISH"
    
    print(f"Publishing to MQTT topic {topic} with write key {write_key}: {MQTT_DATA}")
    try:
        client.publish(topic, payload=MQTT_DATA, qos=0, retain=False)
    except OSError:
        client.reconnect()
        print("Reconnecting to MQTT broker...")

# Main loop
last_publish_time = time.time()
last_fetch_time = time.time()  # Initialize last fetch time

while True:
    # Check button presses
    for pin in [BUTTON_DEC_PIN, BUTTON_INC_PIN, BUTTON_LED_TOGGLE_PIN]:
        current_state = wiringpi.digitalRead(pin)
        if current_state == wiringpi.LOW and previous_button_state[pin] == wiringpi.HIGH:
            if pin == BUTTON_DEC_PIN:
                LUX_GOAL = max(0, LUX_GOAL - BRIGHTNESS_STEP)  # Decrease LUX_GOAL
                print(f"Button on pin {pin} Pressed! Decreased LUX_GOAL to {LUX_GOAL}")
                send_data_to_thingspeak(MQTT_TOPIC_DESIRED, THINGSPEAK_WRITE_API_KEY_DESIRED, LUX_GOAL)
            elif pin == BUTTON_INC_PIN:
                LUX_GOAL = min(1000, LUX_GOAL + BRIGHTNESS_STEP)  # Increase LUX_GOAL
                print(f"Button on pin {pin} Pressed! Increased LUX_GOAL to {LUX_GOAL}")
                send_data_to_thingspeak(MQTT_TOPIC_DESIRED, THINGSPEAK_WRITE_API_KEY_DESIRED, LUX_GOAL)
            elif pin == BUTTON_LED_TOGGLE_PIN:
                # Toggle LED ON/OFF
                manual_led_status = not manual_led_status  # Change the toggle state
                if manual_led_status:
                    # Turn LED ON immediately and enable automatic control
                    manual_led_override = False
                    wiringpi.softPwmWrite(LED_PIN, LED_BRIGHTNESS)  # Turn on LED immediately
                    print("Manual toggle ON: LED turned ON and automatic mode activated.")
                else:
                    # Turn LED OFF immediately and disable automatic control
                    manual_led_override = True
                    wiringpi.softPwmWrite(LED_PIN, 0)  # Turn off LED
                    print("Manual toggle OFF: LED turned OFF and automatic mode deactivated.")

        previous_button_state[pin] = current_state

    # Fetch data from ThingSpeak every 15 seconds
    if time.time() - last_fetch_time >= 15:
        fetch_last_entry()
        # Update LED_BRIGHTNESS based on the fetched LUX_GOAL only if not manually overridden
        if not manual_led_override:
            LED_BRIGHTNESS = LUX_GOAL
            wiringpi.softPwmWrite(LED_PIN, LED_BRIGHTNESS)
        last_fetch_time = time.time()

    # Publish sensor data every 15 seconds
    if time.time() - last_publish_time >= interval:
        bmp280_temperature = int(bmp280.get_temperature())
        bmp280_pressure = int(bmp280.get_pressure())
        lux = get_light_value(bus, bh1750_address)

        # Lux-Based Automatic Control
        if not manual_led_override:  # Automatic control is active
            if lux < 40:  # Turn LED ON if lux is less than 40
                wiringpi.softPwmWrite(LED_PIN, LED_BRIGHTNESS)  # Use current brightness level
                print("Low light detected, LED is ON with brightness:", LED_BRIGHTNESS)
            else:  # Turn LED OFF if lux is 40 or higher
                wiringpi.softPwmWrite(LED_PIN, 0)  # Turn off LED by setting brightness to 0
                print("Sufficient light detected (lux >= 40), LED is OFF")

        # Print all relevant data
        print(f"Temperature: {bmp280_temperature}°C, Pressure: {bmp280_pressure} hPa, Light: {lux} Lux")
        print(f"Temperature Goal: {TEMPERATURE_GOAL}°C, LED Light Goal: {LUX_GOAL}, LED Brightness: {LED_BRIGHTNESS}%, LED Status: {'ON' if manual_led_status else 'OFF'}")

        send_data_to_thingspeak(MQTT_TOPIC_MEASURED, THINGSPEAK_WRITE_API_KEY_MEASURED, lux)
        last_publish_time = time.time()

    time.sleep(0.1)  # Small delay for loop stability