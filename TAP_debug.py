import serial
import time
import logging
import struct
import TAP

class TAP_CLI:
    def __init__(self, port, baudrate, timeout):
        if port is None:
            self.serial = None
        else:
            self.serial = serial.Serial(port, baudrate, timeout)

    def read_TAP_message(self):
        buffer = bytearray()

        while True:
            if self.serial.in_waiting > 0:
                chunk_size = min(255,self.serial.in_waiting)
                chunk = self.serial.read(chunk_size)
                buffer.extend(chunk)

            if len(buffer) >2 and buffer[-2:] == b'\xAA\x55':
                break

            time.sleep(0.001)

        return TAP.TAP_message.unpack(buffer)
    
    def send_TAP_message(self, packed_bytes):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port not connected")
        self.serial.write(packed_bytes)
        print(f"Sent {len(packed_bytes)} bytes")
        


        

def main():

    #Print top ASCII
    print("---------------------- TAP Debug Tool v0.1 ----------------------")
    
    serial_device = input("Select serial device - Default: /dev/ttyACM0:")
    if serial_device == "":
        serial_device = "/dev/ttyACM0"
    
    baudrate_str = input("Select serial baud rate - Default: 9600:")
    if baudrate_str == "":
        baudrate = 9600
    else:
        baudrate = int(baudrate_str)
    
    timeout_str = input("Select serial timeout - Default: 1s:")
    if timeout_str == "":
        timeout = 1
    else:
        timeout = int(timeout_str)
    
    selection = input("Select function:\n\nSend(s)\nMonitor(m)\n\nYour Selection:")
    
    match selection:
        case "s":
            send(serial_device,baudrate,timeout)
        case "m":
            monitor(serial_device,baudrate,timeout)
        case _: 
            print("That is not an option, so funny man!")
            return



def send(serial_device,baudrate,timeout):
    print("Send mode selected")

    print(f"Sending through device '{serial_device}', baudrate:{baudrate}, timeout: {timeout} :")

    #Init serial device
    TAP_CLI = TAP_CLI(port=serial_device,baudrate=baudrate,timeout=timeout)
    
    tap_payload = TAP.TelemetryPayload(43.323228,-3.017115,0xAA55,245,90,90)
    tap_message = TAP.TAP_message(0x02,0x01,TAP.TELEMETRY,tap_payload)
    full_packet = tap_message.pack_message()

    TAP_CLI.send_TAP_message(full_packet)


def monitor(serial_device,baudrate,timeout):
    print("Monitor mode selected")
    
    #Init serial device
    TAP_CLI = TAP_CLI(port=serial_device,baudrate=baudrate,timeout=timeout)
   
    #Infinite loop to monitor incoming messages
    while True:
        try:
            TAP_msg = TAP_CLI.read_TAP_message()
            TAP_msg.string()

        except Exception as e:
            print(f"Read error: {e}")
            continue
    



if __name__ == '__main__':
    main()  