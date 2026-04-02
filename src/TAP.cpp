// TAP.cpp

//If picotool needs sudo, run:
///home/[username]/.pico-sdk/picotool/2.2.0/picotool/picotool load /home/[path_to_file]/build/blink.elf -fx 
//For instance:
///home/crash/.pico-sdk/picotool/2.2.0/picotool/picotool load /home/crash/Documents/Galerna/rpi-tap-tests/build/blink.elf -fx

#include "pico/stdlib.h"
#include <stdio.h>
#include "TAP.h" 
#include <cstring>

// Constructor

    TAP::TAP() {
        messages_since_last_datalink_telem = 0;
    }

    uint8_t TAP::serialize(const TAP_ADDRESS_HEADER *header, const void *payload, TAP_TRAILER *trailer, uint8_t *buffer, uint8_t max_len) {
        if (header->message_len + sizeof(TAP_ADDRESS_HEADER) + sizeof(TAP_TRAILER) > max_len) {
            //Too big for the allowed size!
            return TAP_ERROR_INVALID_LENGTH;
        }
        TAP_ADDRESS_HEADER header_copy = *header;
        header_copy.cobs = 0x0000;
        // Serialize header
        memcpy(buffer, &header_copy, sizeof(TAP_ADDRESS_HEADER));
        uint8_t offset = sizeof(TAP_ADDRESS_HEADER);

        // Serialize payload
        memcpy(buffer + offset, payload, header_copy.message_len);
        offset += header->message_len;

        //We run the CRC check for the header and the payload, but not the trailer.
        //I mean... obviously. It is impossible to CRC check a CRC value. It would change every time we run CRC on it!
        trailer->crc_16 = crc_16(buffer, offset);

        // Serialize trailer, if one was provided (it should be!)
        if (trailer) {
            memcpy(buffer + offset, trailer, sizeof(TAP_TRAILER));
        }

        uint16_t full_byte_length = offset + sizeof(TAP_TRAILER);

        // It's only really necessary in unmanaged links, like... UART, CAN and Bit-Banged radios
        // Still, better safe than sorry
        tapCobs(buffer, full_byte_length);

        return (full_byte_length);
    }
    
    uint16_t TAP::crc_16(uint8_t* message, uint16_t message_len){

        /* DEBUGGING - THIS WAS A TOTAL PAIN IN THE ASS
        printf("CRC intake:\n");
        for(int i = 0; i<message_len; i++){
            printf("%02X ", message[i]);
        }
        printf("\n");
        */

        uint16_t crc = 0xFFFF;
        char i = 0;

        while(message_len--)
        {
            crc ^= (*message++);

            for(i = 0; i < 8; i++)
            {
                if( crc & 1 )
                {
                    crc >>= 1;
                    crc ^= 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return __builtin_bswap16(crc);
    }

    uint8_t TAP::tapCobs(uint8_t* message, uint16_t message_len){
        //COBS
        uint16_t cobs_last_filed_pos = 0x0006;
        //We don't let it check the first or last two bytes of the message because WE KNOW those have the SOF_WORD and we WANT them to have the SOF_WORD
        for(uint16_t i = 2; i<(message_len-2); i++){
            //Remember: network variables (like the ones we use for protocols) are Big Endian, while memory variables are Little Endian
            //That makes a lot of sense and does not become confusing at any point.
            //Debug line
            //printf("Checking %02X %02X\n", message[i], message[i+1]);
            if ((message[i] << 8 | message[i+1]) == TAP_SOF_WORD){
                //Debug lines
                //printf("FOUND ONE!\n");
                //printf("Buffer bytes: %d and %d should be referenced in: %04X\n", i, i+1, cobs_last_filed_pos);
                message[cobs_last_filed_pos+1] = (uint8_t)i;
                message[cobs_last_filed_pos] = (uint8_t)i >> 8;
                
                //See, not confusing at all
                cobs_last_filed_pos = (i);

                //printf("Found one of the bastards!\n");
            }
            // If there are no more TAP_SOF_WORD usages, we set the pointer filed to 0
            message[cobs_last_filed_pos+1] = (uint8_t)0x00;
            message[cobs_last_filed_pos] = (uint8_t)0x00;
        }
        return TAP_OK;
    }
    
    uint8_t TAP::tapSendTelem(const TAP_TELEMETRY &telemetry) {
        uint8_t buffer[255];
        TAP::TAP_ADDRESS_HEADER header;
        TAP::TAP_TRAILER trailer;
        header.target_id = TAP::target_id;
        header.source_id = TAP::source_id;
        header.sof_word = __builtin_bswap16(TAP_SOF_WORD);
        header.message_type = TELEMETRY;
        header.message_len = sizeof(TAP_TELEMETRY);

        TAP_TELEMETRY telemetry_copy;

        //About making the floats big endian
        //We have to trick the __builtint_bswap32 function into thinking these are integers
        telemetry_copy.lat = flipFloatEndianness(telemetry.lat);
        telemetry_copy.lon = flipFloatEndianness(telemetry.lon);
        telemetry_copy.alt = __builtin_bswap16(telemetry.alt);
        telemetry_copy.heading = __builtin_bswap16(telemetry.heading);
        telemetry_copy.pitch = flipFloatEndianness(telemetry.pitch);
        telemetry_copy.roll = flipFloatEndianness(telemetry.roll);

        trailer.eof_word = __builtin_bswap16(TAP_SOF_WORD);

        uint8_t len = serialize(&header, &telemetry_copy, &trailer, buffer, sizeof(buffer));
        if (len == 0) return TAP::TAP_ERROR_INVALID_LENGTH;

        for (uint8_t i = 0; i < len; i++) {

            printf("%02X ", buffer[i]);

            //Keep in mind, uart1 is for the GPS module!
            //Also remember to set the UART port instead of the USB port in the CmakeLists.txt file!
            //uart_putc_raw(uart0, buffer[i]);

            //Printf should obviously be replaced with uart_putc_raw()
            //But this is just easier because using real uart requires another piece of hardware
            //And like, I'd have to go to my pile of ESPs to get a free one, then get another USB cable...
            
        }
        printf("\t\n");

        messages_since_last_datalink_telem ++;
        return TAP_OK;
    }


    //Puts a float into a uint32
    //Flips the endianness
    //Then puts it back into a float
    //Yes, I hate it too
    float TAP::flipFloatEndianness(float f) {
        uint32_t u;
        memcpy(&u, &f, sizeof(float));
        u = __builtin_bswap32(u);
        memcpy(&f, &u, sizeof(float));
        return f;
    }

    //This should be able to fail to detect a correct struct after unraveling the header
    //TODO: Implement COBS decoding!!
    uint8_t TAP::deserialize(uint8_t *raw_message, uint8_t message_len){
        //First we determine what kind of header we've received
        TAP::TAP_ADDRESS_HEADER received_header;
        memcpy(&received_header, raw_message, sizeof(TAP::TAP_ADDRESS_HEADER));
        printf("Header type:%d\n", received_header.message_type);
        switch(received_header.message_type){
            case 0:
                printf("Acknowledgement!\n");
                break;
            case 1:
                printf("Direct command!\n");
                break;
            case 2:
                printf("Indirect command!\n");
                break;
            case 16:
                //"But this is the thing that creates the telemetry data why can it dissect other telemetry"
                //Because it's the only message we've finished so far. Screw you.
                printf("Telemetry!\n");
                printf("Header + Payload length: %d\n", received_header.message_len);
                TAP_TELEMETRY received_telemetry;
                memcpy(&received_telemetry, raw_message+sizeof(TAP_ADDRESS_HEADER), received_header.message_len - sizeof(TAP_ADDRESS_HEADER));
                printf("GPS: %f, %f\n", flipFloatEndianness(received_telemetry.lat), flipFloatEndianness(received_telemetry.lon));
                break;
            case 254:
                printf("Datalink negotiation!\n");
                break;
            case 255:
                printf("Datalink telemetry!\n");
                break;
            default:
                return(TAP_ERROR_UNSUPPORTED_HEADER);
        }
        return(TAP::TAP_OK);
    }