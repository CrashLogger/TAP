import struct, logging
from crc import Calculator,Crc16

ACK = 0x00
DIRECT_COMMAND = 0x01
INDIRECT_COMMAND = 0x02
TELEMETRY = 0x10
NEGOTIATE_DATALINK = 0xFE
TELEMETRY_DATALINK = 0xFF


class TAP_message:
    def __init__(self, tID, sID, messageType, payload):
        self.header = TAP_header(tID,sID,messageType,0)
        self.payload = payload
        self.trailer = TAP_trailer()
        self.length = None
        self.packed_message = None
        self.packed_header = None
        self.packed_payload = None
        self.packed_trailer = None

    def string(self):
        print(f"Length: {len(self.packed_message)}")
        print(f"HEADER({len(self.packed_header)}):",' '.join(f'{b:02x}' for b in self.packed_header))
        print(f"PAYLOAD({len(self.packed_payload)}):",' '.join(f'{b:02x}' for b in self.packed_payload))
        print(f"TRAILER({len(self.packed_trailer)}):",' '.join(f'{b:02x}' for b in self.packed_trailer))

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
        self.packed_payload = self.packed_message[8:-4]
        self.packed_trailer = self.packed_message[-4:]

    def pack_message(self):
        self.packed_payload = self.payload.pack_payload()
        
        self.header.messageLength = len(self.packed_payload)
        

        self.packed_header = self.header.pack_header()
        

        self.trailer.calculate_CRC16(self.packed_header, self.packed_payload)
        

        self.packed_trailer = self.trailer.pack_trailer()
        
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
        
        if message.header.messageType == TELEMETRY:
            message.payload = TelemetryPayload.unpack_payload(packed_payload)
        elif message.header.messageType == TELEMETRY_DATALINK:
            message.payload = TelemetryDatalink.unpack_payload(packed_payload)
        #TODO: Add missing payload types
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

class TAP_trailer:
    def __init__(self,CRC16=None):
        self.CRC16 = CRC16
        self.EOF = 0xAA55

    def calculate_CRC16(self, header_bytes, payload_bytes):
        #logging.DEBUG("Calculating CRC-16")
        data = header_bytes + payload_bytes
        calculator = Calculator(Crc16.MODBUS)
        self.CRC16 = calculator.checksum(data)

    def check_CRC16(self,header_bytes,payload_bytes):
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

class TelemetryDatalink:
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

                

# THIS IS TURBO AI SLOP
CRC16 = 0xA001  # Assuming standard CRC-16 poly, adjust if different

def crc_16(message, message_len):
    out = 0
    bits_read = 0
    bit_flag = 0
    
    # Sanity check:
    if message is None or len(message) == 0:
        return 0
    
    # Convert message_len to actual bytes if it's a bytes object
    if isinstance(message, bytes):
        message_len = len(message)
    
    i = 0  # Byte index
    while message_len > 0:
        bit_flag = (out >> 15) & 1
        
        # Get next bit: work from LSB (item a)
        out = (out << 1) & 0xFFFF
        out |= (message[i] >> bits_read) & 1
        
        # Increment bit counter
        bits_read += 1
        if bits_read > 7:
            bits_read = 0
            i += 1
            message_len -= 1
        
        # Cycle check
        if bit_flag:
            out ^= CRC16
    
    # "Push out" the last 16 bits (item b)
    for _ in range(16):
        bit_flag = out >> 15
        out = (out << 1) & 0xFFFF
        if bit_flag:
            out ^= CRC16
    
    # Reverse the bits (item c)
    crc = 0
    bit_mask = 0x8000
    bit_pos = 0x0001
    while bit_mask != 0:
        if out & bit_mask:
            crc |= bit_pos
        bit_mask >>= 1
        bit_pos <<= 1
    
    return crc