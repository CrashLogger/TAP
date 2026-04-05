import struct, logging
from crc import Calculator,Crc16

ACK = 0x00
DIRECT_COMMAND = 0x01
INDIRECT_COMMAND = 0x02
TELEMETRY = 0x10
NEGOTIATE_DATALINK = 0xFE
TELEMETRY_DATALINK = 0xFF

def message_type_hex_to_str(type_hex):
    match type_hex:
        case 0x00:
            return "ACK"
        case 0x01:
            return "DIRECT_COMMAND"
        case 0x02:
            return "INDIRECT_COMMAND"
        case 0x10:
            return "TELEMETRY"
        case 0xFE:
            return "NEGOTIATE_DATALINK"
        case 0xFF:
            return "TELEMETRY_DATALINK"
        case _:
            return "Unknown payload type"


class TAP_message:
    def __init__(self, tID, sID, messageType, payload=None):
        self.header = TAP_header(tID,sID,messageType,0)
        self.payload = payload
        self.trailer = TAP_trailer()
        self.length = None
        self.packed_message = None
        self.packed_header = None
        self.packed_payload = None
        self.packed_trailer = None
        self.pack_message()

    def string(self):
        print("")
        print(f"Length: {len(self.packed_message)}")
        print(f"HEADER({len(self.packed_header)}):",' '.join(f'{b:02x}' for b in self.packed_header))
        if self.payload is not None:
            print(f"PAYLOAD({len(self.packed_payload)}):",' '.join(f'{b:02x}' for b in self.packed_payload))
        print(f"TRAILER({len(self.packed_trailer)}):",' '.join(f'{b:02x}' for b in self.packed_trailer))
        print("")


    def object_string(self):
        self.header.object_string()
        print("")
        if self.payload is not None:
            self.payload.object_string()
        print("")
        self.trailer.object_string()
        print("")

    def calculate_COBS(self):
        SOF_word_big_endian = 0xAA55
        msg = bytearray(self.packed_message)
        last_cobs_pos = 0x0006
        print(last_cobs_pos)
        for i in range(2, len(msg)-2):
            if(msg[i]<< 8 | msg[i+1]) == SOF_word_big_endian:
                print(f"FOUND ONE, i:{i}")
                msg[last_cobs_pos] = i >> 8
                msg[last_cobs_pos+1] = i 

                last_cobs_pos = i

            msg[last_cobs_pos] = 0x00
            msg[last_cobs_pos+1] = 0x00    

        self.packed_message = msg
        self.packed_header = self.packed_message[0:8]
        if self.header.messageType != ACK:
            self.packed_payload = self.packed_message[8:-4]
        self.packed_trailer = self.packed_message[-4:]

    def pack_message(self):
        if self.header.messageType == ACK:
            self.packed_payload = None
            self.header.messageLength = 0X00
        else:
            self.packed_payload = self.payload.pack_payload()
            self.header.messageLength = len(self.packed_payload)
        

        self.packed_header = self.header.pack_header()
        

        self.trailer.calculate_CRC16(self.packed_header, self.packed_payload)
        

        self.packed_trailer = self.trailer.pack_trailer()
        if self.header.messageType == ACK:
            self.packed_message =  self.packed_header + self.packed_trailer
        else:
            self.packed_message =  self.packed_header + self.packed_payload + self.packed_trailer
        self.calculate_COBS()
        return self.packed_message
    
    def decode_COBS(self,data):
        msg = bytearray(data)
        current_cobs_pos = 0x0006
        if (msg[current_cobs_pos] << 8 | msg[current_cobs_pos+1]) != 0x0000:
            print("There is COBS in this message!!")
            # First one is COBS itself, turn back to 0x0000
            next_cobs_pos = (msg[current_cobs_pos] << 8 | msg[current_cobs_pos+1])
            msg[current_cobs_pos] = 0x00
            msg[current_cobs_pos+1] = 0x00
            current_cobs_pos = next_cobs_pos
            while True:
                print(f"COBS in position:{current_cobs_pos}")
                next_cobs_pos = (msg[current_cobs_pos] << 8 | msg[current_cobs_pos+1])
                #These ones have to be turned back to 0xAA55
                msg[current_cobs_pos] = 0xAA
                msg[current_cobs_pos+1] = 0x55
                if next_cobs_pos == 0x0000:
                    print("No more COBS")
                    break
                current_cobs_pos=next_cobs_pos
                
        else:
            print("No COBS, nothing to see here!")

        return msg    

    @classmethod    
    def unpack(cls,data):
        COBSless_data = cls.decode_COBS(cls,data)
        
        packed_header = COBSless_data[0:8]
        packed_payload = COBSless_data[8:-4]
        packed_trailer = COBSless_data[-4:]
        message = cls(None,None,None,None)
        message.packed_header = packed_header
        message.packed_payload = packed_payload
        message.packed_trailer = packed_trailer
        message.packed_message =  COBSless_data
        message.length = len(COBSless_data)

        message.header = TAP_header.unpack_header(packed_header)
        message.trailer = TAP_trailer.unpack_trailer(packed_trailer)
        assert message.trailer.check_CRC16(message.packed_header,message.packed_payload)
        
        #TODO Add missing payload types (Negotiate Datalink)
        if message.header.messageType == DIRECT_COMMAND:
            message.payload = DirectCommandPayload.unpack_payload(packed_payload)
        elif message.header.messageType == INDIRECT_COMMAND:
            message.payload = IndirectCommandPayload.unpack_payload(packed_payload)
        elif message.header.messageType == TELEMETRY:
            message.payload = TelemetryPayload.unpack_payload(packed_payload)
        elif message.header.messageType == TELEMETRY_DATALINK:
            message.payload = TelemetryDatalinkPayload.unpack_payload(packed_payload)
        else:
            message.payload = None
                
        return message



      
class TAP_header:
    def __init__(self, tID, sID, messageType, messageLength):
        self.SOF = 0xAA55
        self.tID = tID
        self.sID = sID
        if messageType == DIRECT_COMMAND:
            self.messageLength = messageLength
        else:
            self.messageLength = 0x00

        self.messageType = messageType
        self.COBS = 0x0000

    def pack_header(self):

        return struct.pack('>HBBBBH',
            self.SOF,          
            self.tID,          
            self.sID,         
            self.messageLength,
            self.messageType,  
            self.COBS          
        )
    @classmethod
    def unpack_header(cls, packed_data):
        
        sof, tID, sID, messageLength, messageType, COBS = struct.unpack(
            '>HBBBBH', packed_data
        )
        return cls(tID,sID,messageType,messageLength)  
    
    def object_string(self):
        print("TAP header:")
        print(f"SOF:{hex(self.SOF)}")
        print(f"tID:0x{self.tID:02x}({self.tID})")
        print(f"sID:0x{self.sID:02x}({self.sID})")
        print(f"Message Type:0x{self.messageType:02x}({message_type_hex_to_str(self.messageType)})")
        print(f"Message Length:{hex(self.messageLength)}")
        print(f"COBS:0x{self.COBS:02x}")



class TAP_trailer:
    def __init__(self,CRC16=None):
        self.CRC16 = CRC16
        self.EOF = 0xAA55

    def calculate_CRC16(self, header_bytes, payload_bytes):
        #logging.DEBUG("Calculating CRC-16")
        
        #For ACKs, there is no payload
        if payload_bytes is None:
            payload_bytes = b'' 
        data = header_bytes + payload_bytes
        calculator = Calculator(Crc16.MODBUS)
        self.CRC16 = calculator.checksum(data)

    def check_CRC16(self,header_bytes,payload_bytes):
        if payload_bytes is None:
            payload_bytes = bytearray()
        data = header_bytes + payload_bytes
        calculator = Calculator(Crc16.MODBUS)
        calculated_crc = calculator.checksum(data)
    
        if self.CRC16 != calculated_crc:
            raise ValueError(f"CRC mismatch! Expected {calculated_crc:04x}, got {self.CRC16:04x}")
    
        print(f"CRC16 verified: {calculated_crc:04x}")
        return True
    
    def pack_trailer(self):

        return struct.pack('>HH',
            self.CRC16,
            self.EOF
        )
    
    @classmethod
    def unpack_trailer(cls, packed_data):
        
        CRC16, EOF = struct.unpack(
            '>HH', packed_data
        )
        return cls(CRC16)  
    
    def object_string(self):
        print("TAP trailer:")
        print(f"CRC:{hex(self.CRC16)}")
        print(f"EOF:{hex(self.EOF)}")
        


class TelemetryPayload:
    def __init__(self, lat, lon, alt, heading, roll, pitch):
        self.lat = lat
        self.lon = lon
        self.alt = alt
        self.heading = heading
        self.roll = roll
        self.pitch = pitch
    
    def pack_payload(self):
        return struct.pack('>ffHhff',
            self.lat,        
            self.lon,      
            self.alt,         
            self.heading, 
            self.roll,  
            self.pitch          
        )
    
    @classmethod
    def unpack_payload(cls, packed_data):
        
        lat, lon, alt, heading, roll, pitch = struct.unpack(
            '>ffHhff', packed_data
        )
        return cls(lat, lon, alt, heading, roll, pitch)  
    
    def object_string(self):
        print("Telemetry Payload:")
        print(f"Latitude: 0x{struct.pack('>f', self.lat).hex()} ({self.lat})")
        print(f"Longitude: 0x{struct.pack('>f', self.lon).hex()} ({self.lon})")
        print(f"Altitude: 0x{struct.pack('>H', self.alt).hex()} ({self.alt})")
        print(f"Heading: 0x{struct.pack('>h', self.heading).hex()} ({self.heading})")
        print(f"Roll: 0x{struct.pack('>f', self.roll).hex()} ({self.roll})")
        print(f"Pitch: 0x{struct.pack('>f', self.pitch).hex()} ({self.pitch})")

class BooleanArray:
    def __init__(self):
        self.bits = None #16 bit array
        self.bools = None #Value that array takes as an uint16

    #Setting either one cascades in the other
    def setBits(self, bits):
        self.bits = bits
        self.bools = sum(bit << (15 - i) for i, bit in enumerate(bits))

    def setBools(self, bools):
        self.bools = bools
        self.bits = [(bools >> (15 - i)) & 1 for i in range(16)]

    def print_array(self):
        flag_names = ['ARM', 'AUTO', 'STAB', 'NAVLIGHT', 'STROBE', 'LAND', 
                    '', '', '', '', '', '', 'COMLOSSBY', 'LOITER', 'RTH']
        
        for i, name in enumerate(flag_names):
            if name:  # Only print named flags
                status = 'y' if self.bits[i] else 'n'
                print(f"{name:<10}: {status} (bit {i})")



class DirectCommandPayload:
    MAX_SIGNAL_COUNT = 8
    def __init__(self, bools, channels):
        if channels is None:
            channels = []
        if len(channels) > self.MAX_SIGNAL_COUNT:
            raise ValueError(f"Max {self.MAX_SIGNAL_COUNT} signals allowed")
        self.channels = channels 
        for i in range(len(self.channels),self.MAX_SIGNAL_COUNT):
            self.channels.append(0x0000)
        self.booleanArray = BooleanArray() 
        self.booleanArray.setBools(bools)

    
    def pack_payload(self):
        count = len(self.channels)
        format = f'>HH{"H"*count}'
        return struct.pack(format,
            self.booleanArray.bools,
            0x0000,
            *self.channels
            )
    
    def unpack_payload(cls,packed_data):
        if len(packed_data) < 1:
            raise ValueError("Payload too short")
        count = len(packed_data)/2
        fmt = f'>{"H"*count}'
        bools = struct.unpack('>H', packed_data[:2])
        channels = list(struct.unpack(fmt, packed_data[4:]))
        return cls(bools, channels)
    
    def object_string(self):
        print("Direct Command Payload:")
        print(f"Flags: 0x{self.booleanArray.bools:04x}({self.booleanArray.bools})")
        self.booleanArray.print_array()
        for i in range(0,len(self.channels)):
            print(f"Channel {i} command: {hex(self.channels[i])}({self.channels[i]})")
        
class IndirectCommandPayload:
    def __init__(self, bools, gps_lat, gps_lon, alt, rad):
        self.gps_lat = gps_lat
        self.gps_lon = gps_lon
        self.alt = alt
        self.rad = rad

        #Same procedure as direct command
        self.booleanArray = BooleanArray() 
        self.booleanArray.setBools(bools)

    def pack_payload(self):
        return struct.pack('>HHffHH',
            self.booleanArray.bools,
            0x0000,
            self.gps_lat,
            self.gps_lon,
            self.alt,
            self.rad
            )
    
    def unpack_payload(cls,packed_data):
        if len(packed_data) < 1:
            raise ValueError("Payload too short")

        bools = struct.unpack('>H', packed_data[:2])
        gps_lat, gps_lon, alt, rad = list(struct.unpack('>ffHH', packed_data[4:]))
        return cls(bools, gps_lat, gps_lon, alt, rad)
    
    def object_string(self):
        print("Indirect Command Payload:")
        print(f"Flags: 0x{self.booleanArray.bools:04x}({self.booleanArray.bools})")
        self.booleanArray.print_array()
        print(f"Waypoint Latitude: 0x{struct.pack('>f', self.gps_lat).hex()} ({self.gps_lat})")
        print(f"Waypoint Longitude: 0x{struct.pack('>f', self.gps_lon).hex()} ({self.gps_lon})")
        print(f"Waypoint Altitude/Depth: 0x{struct.pack('>H', self.alt).hex()} ({self.alt})")
        print(f"Waypoint Precission Radius: 0x{struct.pack('>H', self.rad).hex()} ({self.rad})")



class TelemetryDatalinkPayload:
    def __init__(self, RSSI, SNR, RTT, SENT_PKTS, DELTA_T):
        self.RSSI = RSSI
        self.SNR = SNR
        self.RTT = RTT
        self.SENT_PKTS = SENT_PKTS
        self.DELTA_T = DELTA_T
        self.reserved = 0x0000

    def pack_payload(self):
        return struct.pack('>hHHHHH',
            self.RSSI,
            self.SNR,
            self.RTT,
            self.SENT_PKTS,
            self.DELTA_T,
            self.reserved
        )
    
    @classmethod
    def unpack_payload(cls, packed_data):
        
        RSSI, SNR, RTT, SENT_PKTS, DELTA_T, reserved = struct.unpack(
            '>hHHHHH', packed_data
        )
        return cls(RSSI,SNR,RTT,SENT_PKTS,DELTA_T,reserved)  
    
    def object_string(self):
        print("Telemetry Datalink Payload:")
        print(f"RSSI:0x{struct.pack('>h', self.RSSI).hex()}({self.RSSI})") #Signed int, simple method simply doesn't cut it
        print(f"SNR:0x{self.SNR:04x}({self.SNR})")
        print(f"RTT:0x{self.RTT:04x}({self.RTT})")
        print(f"SENT_PKTS:0x{self.SENT_PKTS:04x}({self.SENT_PKTS})")
        print(f"DELTA_T:0x{self.DELTA_T:04x}({self.DELTA_T})")
        print(f"Reserved:0x{self.reserved:04x}({self.reserved})")
