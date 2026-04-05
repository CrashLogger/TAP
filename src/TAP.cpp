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
        header_copy.sof_word = TAP::TAP_SOF_WORD;
        header_copy.cobs = 0x0000;
        // Serialize header
        memcpy(buffer, &header_copy, sizeof(TAP_ADDRESS_HEADER));
        uint8_t offset = sizeof(TAP_ADDRESS_HEADER);

        // Serialize payload
        memcpy(buffer + offset, payload, header_copy.message_len);
        offset += header->message_len;

        //We run the CRC check for the header and the payload, but not the trailer.
        //I mean... obviously. It is impossible to CRC check a CRC value. It would change every time we run CRC on it!
        //The crc_16 function returns the crc_16 value in BIG ENDIAN already!!
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

/*         printf("TapCobs received:\n");
        for(uint16_t i = 0; i<message_len; i++){
            printf("%02X ", message[i]);
        }
        printf("\n"); */

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

        //The CRC field gets handled by the serializer!!
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
    uint8_t TAP::deserialize(uint8_t *raw_message, uint8_t message_len, uint8_t* payload_type_out, uint8_t* payload_len_out, uint8_t* payload_buffer_out){
        TAP::TAP_ADDRESS_HEADER received_header;
        TAP::TAP_TRAILER received_trailer;

        memcpy(&received_header, raw_message, sizeof(TAP::TAP_ADDRESS_HEADER));
        memcpy(&received_trailer, raw_message + sizeof(TAP::TAP_ADDRESS_HEADER) + received_header.message_len, sizeof(TAP::TAP_TRAILER));

        //This will take the payload out of the full message, and give it back separately.
        //It also gives back the payload length and the payload type separately
        //The controller is in charge of identifying the fields within the payload

        *payload_type_out = received_header.message_type;
        *payload_len_out = received_header.message_len-sizeof(TAP_ADDRESS_HEADER);

        memcpy(payload_buffer_out, raw_message + sizeof(TAP_ADDRESS_HEADER), *payload_len_out);

        tapUnCobs(raw_message, (sizeof(TAP_ADDRESS_HEADER) + received_header.message_len));

        uint16_t check_crc_16 = crc_16(raw_message, (sizeof(TAP_ADDRESS_HEADER) + received_header.message_len));
        if(received_trailer.crc_16 != check_crc_16){
            return(TAP::TAP_ERROR_CRC_MISMATCH);
        }
        return(TAP::TAP_OK);
    }

    uint8_t TAP::tapUnCobs(uint8_t* message, uint16_t message_len){
        
        uint16_t cobs_pos = 6;
        uint16_t cobs_check = 0;

        //The first cobs field is always in the header, and therefore, an arbitrary position
        memcpy(&cobs_check, message + cobs_pos, sizeof(uint16_t));
        cobs_check = __builtin_bswap16(cobs_check);
        if(cobs_check > 0){
            message[6] = 0x00;
            message[7] = 0x00;
            uint16_t tap_sof_word = __builtin_bswap16(TAP::TAP_SOF_WORD);
            cobs_pos = cobs_check;
            //The following cobs fields are in the payload, and their pointers are counted from the start of the message!!
            while(cobs_pos > 0 && cobs_pos < message_len){
                memcpy(&cobs_check, message + cobs_pos, sizeof(uint16_t));
                printf("Here! - cobs_check = %d\n", cobs_check);
                memcpy(message + cobs_pos, &tap_sof_word, sizeof(uint16_t));
                cobs_pos = __builtin_bswap16(cobs_check);
            }
        }
        return(TAP::TAP_OK);
    }
