import time
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt
import wiringpi  # Voor GPIO-aansturing op de Orange Pi

# BH1750 setup
bus = SMBus(0)
bh1750_address = 0x23
bus.write_byte(bh1750_address, 0x10)
interval = 15

# GPIO-instellingen voor de LED
LED_PIN = 2  # Pas dit aan naar de juiste GPIO-pin
wiringpi.wiringPiSetup()
wiringpi.pinMode(LED_PIN, wiringpi.OUTPUT)
wiringpi.digitalWrite(LED_PIN, wiringpi.LOW)  # Zorg dat LED standaard uit staat

# Lichtdrempel voor LED
LIGHT_THRESHOLD = 20  # LED gaat aan bij lichtwaarde onder deze drempel

# MQTT-instellingen
MQTT_CLIENT_ID = "KRMcKzkpOBcZIi4VEzAZIjA"  # Vervang door jouw ThingSpeak MQTT Client ID
MQTT_USER = "KRMcKzkpOBcZIi4VEzAZIjA"        # Vervang door jouw ThingSpeak MQTT Username
MQTT_PWD = "8uzNOWNKQrd1w7LqR0ft1VNt"        # Vervang door jouw ThingSpeak MQTT Password
MQTT_HOST = "mqtt3.thingspeak.com"
MQTT_PORT = 1883
MQTT_TOPIC_MEASURED = "channels/2792379/publish"  # Vervang door jouw kanaal-ID voor gemeten waarden
MQTT_TOPIC_DESIRED = "channels/2792381/publish"  # Vervang door jouw kanaal-ID voor gewenste waarden

# Functie om lichtwaarden te lezen van de BH1750-sensor
def get_light_value(bus, address):
    write = i2c_msg.write(address, [0x10])  # 1lx resolutie, continu modus
    read = i2c_msg.read(address, 2)
    bus.i2c_rdwr(write, read)
    bytes_read = list(read)
    return int((((bytes_read[0] & 3) << 8) + bytes_read[1]) / 1.2)  # Converteer naar lux

# Callback-functies voor MQTT
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Status: Connected OK")
    elif rc == 4:
        print("MQTT Status: Connection refused – bad username or password")
    else:
        print(f"MQTT Status: Connection failed with result code {rc}")

def on_disconnect(client, userdata, flags, rc=0):
    print(f"MQTT Status: Disconnected with result code {rc}")

# MQTT Client configuratie
client = mqtt.Client(client_id=MQTT_CLIENT_ID)
client.username_pw_set(MQTT_USER, MQTT_PWD)
client.on_connect = on_connect
client.on_disconnect = on_disconnect

# Verbind met ThingSpeak
print("\n[System] Connecting to ThingSpeak...")
client.connect(MQTT_HOST, MQTT_PORT)
client.loop_start()

while True:
    try:
        # Meet lichtwaarden
        lux = get_light_value(bus, bh1750_address)
        
        # Reguleer de LED op basis van lichtwaarde
        led_status = "OFF"
        if lux < LIGHT_THRESHOLD:
            wiringpi.digitalWrite(LED_PIN, wiringpi.HIGH)  # LED aan
            led_status = "ON"
        else:
            wiringpi.digitalWrite(LED_PIN, wiringpi.LOW)   # LED uit

        # Print meetresultaten naar terminal
        print("\n===== SENSOR DATA =====")
        print(f"Measured light: {lux} Lux")
        print(f"LED Status: {led_status} (Threshold: {LIGHT_THRESHOLD} Lux)")

        # Publiceer gemeten waarde naar ThingSpeak
        MQTT_DATA_MEASURED = f"field7={lux}&status=MQTTPUBLISH"
        print("\nPublishing to ThingSpeak:")
        print(f"  - Measured Value: {MQTT_DATA_MEASURED}")
        client.publish(MQTT_TOPIC_MEASURED, payload=MQTT_DATA_MEASURED, qos=0, retain=False)
        
        # Publiceer gewenste waarde naar ThingSpeak
        desired_value = LIGHT_THRESHOLD  # Drempelwaarde als voorbeeld
        MQTT_DATA_DESIRED = f"field7={desired_value}&status=MQTTPUBLISH"
        print(f"  - Desired Value: {MQTT_DATA_DESIRED}")
        client.publish(MQTT_TOPIC_DESIRED, payload=MQTT_DATA_DESIRED, qos=0, retain=False)

        # Wacht tot de volgende meting
        print("\n[System] Waiting for next measurement...\n")
        time.sleep(interval)

    except OSError as e:
        print(f"[Error] {e}. Reconnecting...")
        client.reconnect()

    except KeyboardInterrupt:
        print("\n[System] Exiting program...")
        wiringpi.digitalWrite(LED_PIN, wiringpi.LOW)  # Zet de LED uit
        client.loop_stop()
        client.disconnect()
        break