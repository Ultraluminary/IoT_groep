import time
from smbus2 import SMBus, i2c_msg
import paho.mqtt.client as mqtt

# BH1750 setup
bus = SMBus(0)
bh1750_address = 0x23
bus.write_byte(bh1750_address, 0x10)
interval = 15

# MQTT instellingen voor gemeten waarden
CHANNEL_ID_MEASURED = "2792379"
WRITE_API_MEASURED = "LKPW521GXHAU6G31"

# MQTT instellingen voor gewenste waarden
CHANNEL_ID_DESIRED = "2792381"
WRITE_API_DESIRED = "2HBKEUX8ENIXKAGX"

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
        print("Connected OK")
    elif rc == 4:
        print("Connection refused – bad username or password")
    else:
        print(f"Connection failed with result code {rc}")

def on_disconnect(client, userdata, flags, rc=0):
    print(f"Disconnected with result code {rc}")

# MQTT Client configuratie voor gemeten waarden
client_measured = mqtt.Client(client_id="MeasuredValuesPublisher")
client_measured.username_pw_set(CHANNEL_ID_MEASURED, WRITE_API_MEASURED)
client_measured.on_connect = on_connect
client_measured.on_disconnect = on_disconnect

# MQTT Client configuratie voor gewenste waarden
client_desired = mqtt.Client(client_id="DesiredValuesPublisher")
client_desired.username_pw_set(CHANNEL_ID_DESIRED, WRITE_API_DESIRED)
client_desired.on_connect = on_connect
client_desired.on_disconnect = on_disconnect

# Verbind met ThingSpeak voor beide clients
print("Connecting to ThingSpeak...")
client_measured.connect("mqtt3.thingspeak.com", 1883)
client_desired.connect("mqtt3.thingspeak.com", 1883)
client_measured.loop_start()
client_desired.loop_start()

while True:
    try:
        # Meet lichtwaarden
        lux = get_light_value(bus, bh1750_address)
        print(f"Measured light: {lux} Lux")
        
        # Publiceer gemeten waarde naar kanaal 2792379
        MQTT_DATA_MEASURED = f"field7={lux}&status=MQTTPUBLISH"
        print(f"Publishing measured value to ThingSpeak: {MQTT_DATA_MEASURED}")
        client_measured.publish(f"channels/{CHANNEL_ID_MEASURED}/publish", payload=MQTT_DATA_MEASURED, qos=0, retain=False)
        
        # Publiceer gewenste waarde naar kanaal 2792381
        desired_value = 50  # Voorbeeld gewenste waarde
        MQTT_DATA_DESIRED = f"field7={desired_value}&status=MQTTPUBLISH"
        print(f"Publishing desired value to ThingSpeak: {MQTT_DATA_DESIRED}")
        client_desired.publish(f"channels/{CHANNEL_ID_DESIRED}/publish", payload=MQTT_DATA_DESIRED, qos=0, retain=False)
        
    except OSError as e:
        print(f"Error: {e}. Reconnecting...")
        client_measured.reconnect()
        client_desired.reconnect()
    
    # Wacht tot de volgende meting
    time.sleep(interval)