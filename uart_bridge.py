import serial

from TAP import TAP_message
from TAP import TELEMETRY
from TAP import TELEMETRY_DATALINK
from TAP import DIRECT_COMMAND

from mqtt_helper import (
    publish_telemetry,
    publish_datalink,
    publish_direct_command
)


class UARTBridge:

    FRAME_SIZE = 32

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=9600
    ):

        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1
        )

        print(
            f"[UART] Connected to "
            f"{port} @ {baudrate}"
        )

    def read_frame(self):

        frame = self.serial.read(
            self.FRAME_SIZE
        )

        if len(frame) != self.FRAME_SIZE:
            return None

        return frame

    def run(self, mqtt_client):

        while True:

            frame = self.read_frame()

            if frame is None:
                continue

            print(
                "[UART RX]",
                frame.hex(" ")
            )

            try:

                message = TAP_message.unpack(
                    frame
                )

                msg_type = (
                    message.header.messageType
                )

                #
                # Vehicle telemetry
                #
                if msg_type == TELEMETRY:

                    print(
                        "[TAP] TELEMETRY"
                    )

                    publish_telemetry(
                        mqtt_client,
                        message.payload
                    )

                    mqtt_client.publish(
                        "RX-STATE",
                        "CONNECTED"
                    )

                #
                # Datalink telemetry
                #
                elif msg_type == TELEMETRY_DATALINK:

                    print(
                        "[TAP] TELEMETRY_DATALINK"
                    )

                    publish_datalink(
                        mqtt_client,
                        message.payload
                    )

                    mqtt_client.publish(
                        "RX-STATE",
                        "CONNECTED"
                    )

                #
                # Direct commands
                #
                elif msg_type == DIRECT_COMMAND:

                    print(
                        "[TAP] DIRECT_COMMAND"
                    )

                    publish_direct_command(
                        mqtt_client,
                        message.payload
                    )

                    mqtt_client.publish(
                        "RX-STATE",
                        "CONNECTED"
                    )

                else:

                    print(
                        f"[TAP] Ignoring type "
                        f"{hex(msg_type)}"
                    )

            except Exception as e:

                print(
                    "[ERROR]",
                    str(e)
                )

                mqtt_client.publish(
                    "RX-STATE",
                    "ERROR"
                )