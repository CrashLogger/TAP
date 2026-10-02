from mqtt_helper import MQTTBridge
from uart_bridge import UARTBridge


UART_PORT = "/dev/ttyUSB0"
UART_BAUD = 9600


def main():

    print("")
    print("====================")
    print(" TAP MQTT BRIDGE")
    print("====================")
    print("")

    #
    # MQTT
    #

    mqtt = MQTTBridge()

    mqtt.connect()

    mqtt.publish(
        "RX-STATE",
        "STARTING"
    )

    #
    # UART
    #

    bridge = UARTBridge(
        port=UART_PORT,
        baudrate=UART_BAUD
    )

    mqtt.publish(
        "RX-STATE",
        "CONNECTED"
    )

    #
    # Main loop
    #

    bridge.run(mqtt)


if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print(
            "\n[SYS] Shutdown requested"
        )

    except Exception as e:

        print(
            f"\n[FATAL] {e}"
        )