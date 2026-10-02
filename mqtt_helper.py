import os
import paho.mqtt.client as mqtt

from dotenv import load_dotenv

load_dotenv()


MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")

MQTT_BROKER = os.getenv(
    "MQTT_BROKER",
    "app.galerna.eus"
)

MQTT_PORT = int(
    os.getenv(
        "MQTT_PORT",
        8883
    )
)


class MQTTBridge:

    def __init__(self):

        self.client = mqtt.Client()

        self.client.tls_set()

        if MQTT_USERNAME and MQTT_PASSWORD:
            self.client.username_pw_set(
                MQTT_USERNAME,
                MQTT_PASSWORD
            )

    def connect(self):

        self.client.connect(
            MQTT_BROKER,
            MQTT_PORT,
            60
        )

        self.client.loop_start()

        print(
            f"[MQTT] Connected to "
            f"{MQTT_BROKER}:{MQTT_PORT}"
        )

    def publish(self, field, value):

        topic = f"urpekari/telem/{field}"

        self.client.publish(
            topic,
            str(value),
            retain=True
        )

    def disconnect(self):

        self.client.loop_stop()

        self.client.disconnect()


def publish_telemetry(mqtt, payload):

    mqtt.publish(
        "VEH-LAT",
        round(payload.lat, 7)
    )

    mqtt.publish(
        "VEH-LON",
        round(payload.lon, 7)
    )

    mqtt.publish(
        "VEH-ALT",
        payload.alt
    )

    mqtt.publish(
        "VEH-HDG",
        payload.heading
    )

    mqtt.publish(
        "VEH-ROLL",
        round(payload.roll, 2)
    )

    mqtt.publish(
        "VEH-PITCH",
        round(payload.pitch, 2)
    )

    #
    # Fake speed for demo, this should be a rolling average from GPS calculations
    #

    mqtt.publish(
        "VEH-SPD",
        0
    )

def publish_datalink(mqtt, payload):

    mqtt.publish(
        "RX-RSSI",
        payload.RSSI
    )

    mqtt.publish(
        "RX-SNR",
        payload.SNR
    )

    mqtt.publish(
        "RX-RTT",
        payload.RTT
    )

    mqtt.publish(
        "RX-PKTS",
        payload.SENT_PKTS
    )

def bool_array_to_mode(boolean_array):

    names = [
        "ARM",
        "AUTO",
        "STAB",
        "NAVLIGHT",
        "STROBE",
        "LAND",
        "",
        "",
        "",
        "",
        "",
        "",
        "COMLOSSBY",
        "LOITER",
        "RTH",
        ""
    ]

    active = []

    for index, flag in enumerate(boolean_array.bits):

        if (
            flag
            and index < len(names)
            and names[index]
        ):
            active.append(
                names[index]
            )

    if len(active) == 0:
        return "NONE"

    return ",".join(active)

def publish_direct_command(mqtt, payload):

    mqtt.publish(
        "VEH-MODE",
        bool_array_to_mode(payload.booleanArray)
    )