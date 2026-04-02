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
        // Serialize header
        memcpy(buffer, header, sizeof(TAP_ADDRESS_HEADER));
        uint8_t offset = sizeof(TAP_ADDRESS_HEADER);

        // Serialize payload
        memcpy(buffer + offset, payload, header->message_len);
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
    
    // From: https://stackoverflow.com/questions/10564491/function-to-calculate-a-crc16-checksum
    uint16_t crc_16(uint8_t* message, uint16_t message_len){
        uint16_t out = 0;
        int bits_read = 0, bit_flag;

        /* Sanity check: */
        if(message == NULL)
            return 0;

        while(message_len > 0)
        {
            bit_flag = out >> 15;

            /* Get next bit: */
            out <<= 1;
            out |= (*message >> bits_read) & 1; // item a) work from the least significant bits

            /* Increment bit counter: */
            bits_read++;
            if(bits_read > 7)
            {
                bits_read = 0;
                message++;
                message_len--;
            }

            /* Cycle check: */
            if(bit_flag)
                out ^= CRC16;
        }

        // item b) "push out" the last 16 bits
        int i;
        for (i = 0; i < 16; ++i) {
            bit_flag = out >> 15;
            out <<= 1;
            if(bit_flag)
                out ^= CRC16;
        }

        // item c) reverse the bits
        uint16_t crc = 0;
        i = 0x8000;
        int j = 0x0001;
        for (; i != 0; i >>=1, j <<= 1) {
            if (i & out) crc |= j;
        }
        return crc;
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
            if ((message[i] << 8 | message[i+1]) == __builtin_bswap16(TAP_SOF_WORD)){
                //Debug lines
                //printf("FOUND ONE!\n");
                //printf("Buffer bytes: %d and %d should be referenced in: %04X\n", i, i+1, cobs_last_filed_pos);
                message[cobs_last_filed_pos] = (uint8_t)i;
                message[cobs_last_filed_pos+1] = (uint8_t)i >> 8;
                
                //See, not confusing at all
                cobs_last_filed_pos = (i);

                //printf("Found one of the bastards!\n");
            }
            // If there are no more TAP_SOF_WORD usages, we set the pointer filed to 0
            message[cobs_last_filed_pos] = (uint8_t)0x00;
            message[cobs_last_filed_pos+1] = (uint8_t)0x00;
        }
        return TAP_OK;
    }
    
    uint8_t TAP::tapSendTelem(const TAP_TELEMETRY &telemetry) {
        uint8_t buffer[255];
        TAP::TAP_ADDRESS_HEADER header;
        TAP::TAP_TRAILER trailer;
        header.sof_word = TAP_SOF_WORD;
        header.message_type = TELEMETRY;
        header.message_len = sizeof(TAP_TELEMETRY);

        trailer.eof_word = TAP_SOF_WORD;

        uint8_t len = serialize(&header, &telemetry, &trailer, buffer, sizeof(buffer));
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


    //This should be able to fail to detect a correct struct after unraveling the header
    //TODO: Implement COBS decoding!!
    uint8_t* deserialize(uint8_t *raw_message, uint8_t* message);