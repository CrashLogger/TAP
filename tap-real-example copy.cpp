#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>
#include "string.h"
#include "TAP.h"

//===== DEFINITIONS

//Various buffers
//Dirty buffer
uint8_t tap_uart_rx_buffer_len = 0;
uint8_t tap_uart_rx_buffer[255];

//Clean last message buffer
uint8_t tap_uart_rx_last_message_len = 0;
uint8_t tap_uart_rx_last_message[255];
bool tap_rx_buffers_sem;

//Simple I/O
#define TAIL_LIGHT 13                   //COMMON CONTROL: NAV_LIGHTS
#define STARBOARD_LIGHT 12              //COMMON CONTROL: NAV_LIGHTS
#define PORT_LIGHT 11                   //COMMON CONTROL: NAV_LIGHTS
#define NAV_LIGHTS 16                   //TAIL, STARBOARD AND PORT LIGHTS
#define STROBES 17                      //ALL AROUND STROBE LIGHTS
#define RADIO_LINK_LOSS_INDICATOR 14    //UNUSED IN CURRENT PROTOTYPE

//===== GLOBAL VARS =====

//SCHEDULING
uint32_t ms_last_change = 0;
uint32_t ms_last_tap = 0;
uint32_t ms_last_rx = 0;

// ================================================================================================
// TAP PROTOCOL
    // Creating a global object is a good idea and definitely not frowned upon.
    TAP tap(01, 02);

    //The transmission header should be pretty constant
    TAP::TAP_ADDRESS_HEADER our_tx_header;
    TAP::TAP_TRAILER our_tx_trailer;

    //The uart port used by TAP
    #define TAP_UART_ID uart0
    #define DATA_BITS 8
    #define STOP_BITS 1
    #define PARITY    UART_PARITY_NONE

// ================================================================================================
//Although the change is quick enough not to break 99.9999% of the time, this could be atomic-ised.
//Semaphore UP is blocking!
uint8_t semaphore_up(bool sem){
    if(!sem){
        sem = true;
        return(0);
    }
    return(1);
}

//Semaphore DOWN is NOT blocking!
uint8_t semaphore_down(bool sem){
    if(sem){
        sem = false;
        return(0);
    }
    return(1);
}


uint8_t tap_telemetry(){

    /*
        This is where we'd get the telemetry data we want from the sensor, in this case, a GPS module
        However, I am silly and make the numbers up. Good demo this one.
    */

    //Fun fact if you don't use the delay we get a race condition!!! Wooo!!! :3
    //I had mutex implemented in an older, makeshift version, for reading from the GPS thing I think?
    //Maybe we should do that properly.
    if(to_ms_since_boot(get_absolute_time()) - ms_last_tap >= 250){
        ms_last_tap = to_ms_since_boot(get_absolute_time());

        TAP::TAP_TELEMETRY telem;

        telem.lat = 43.323228;
        telem.lon = -3.017115;

        // Use these for somewhat realistic values!
        //telem.alt = 50;
        //telem.heading = 200;
        telem.pitch = 90.0;
        telem.roll = 45.0;

        // Use these to check if COBS works!
        telem.alt = 0xAA55;
        telem.heading = 0xAA55;

        tap.tapSendTelem(telem);
        return(0);
    }
    return(1);
}

uint8_t direct_command_worker(TAP::TAP_DIRECT_COMMAND command){
    
    //There HAS got to be a better way of doing this
    command.bools = __builtin_bswap16(command.bools);

    command.channel_0 = __builtin_bswap16(command.channel_0);
    command.channel_1 = __builtin_bswap16(command.channel_1);
    command.channel_2 = __builtin_bswap16(command.channel_2);
    command.channel_3 = __builtin_bswap16(command.channel_3);
    command.channel_4 = __builtin_bswap16(command.channel_4);
    command.channel_5 = __builtin_bswap16(command.channel_5);
    command.channel_6 = __builtin_bswap16(command.channel_6);
    command.channel_7 = __builtin_bswap16(command.channel_7);
/* 
    //Check if the armed bit is high
    if(command.bools > 0x8000){
        //printf("Armed!\n");
        //printf("Motor(s) going BRRRRRRRRR at: %d speed!!\n", command.channel_0);
    }
    else{
        //printf("Command bools:%d\n", (command.bools));
        //printf("NOT armed!! Safe!! :3\n");
    } */
    return(0);
}

void tap_rx_irq(){
    uint8_t ch = 0;
    while(uart_is_readable(TAP_UART_ID)){
        ch = uart_getc(TAP_UART_ID);
        if(semaphore_up(tap_rx_buffers_sem)){
            tap_uart_rx_buffer[tap_uart_rx_buffer_len] = ch;
            tap_uart_rx_buffer_len++;
        }
        semaphore_down(tap_rx_buffers_sem);
        

        //Check if we should extract a packet from the diryt buffer
        if(tap_uart_rx_buffer_len>8){
            if(__builtin_bswap16((uint16_t)tap_uart_rx_buffer[tap_uart_rx_buffer_len]) == TAP::TAP_SOF_WORD){
                if(semaphore_up(tap_rx_buffers_sem)){
                    //We copy the entire packet to the clean buffer
                    memcpy(tap_uart_rx_last_message, tap_uart_rx_buffer, tap_uart_rx_buffer_len);
                    tap_uart_rx_last_message_len = tap_uart_rx_buffer_len;

                    //And then we clear the dirty buffer
                    memset(tap_uart_rx_buffer, 0, tap_uart_rx_buffer_len);
                    tap_uart_rx_buffer_len = 0;
                }
                semaphore_down(tap_rx_buffers_sem);
            }
        }
    }

}

uint8_t tap_rx_process(){

    /*
        This is where we process received messages.
        As you can tell, we are faking it by feeding it the message the telemetry function sends.
    */

    if(to_ms_since_boot(get_absolute_time()) - ms_last_rx >= 1000){
        ms_last_rx = to_ms_since_boot(get_absolute_time());
/* 
        printf("Still not used but the clean buffer looks like so:\n");

        for(uint16_t i = 0; i<tap_uart_rx_last_message_len; i++){
            printf("%02X ", tap_uart_rx_last_message[i]);
        }
        printf("\n"); */

        //Direct command example
        //uint8_t test_buffer[32] = {0xAA, 0x55, 0x02, 0x01, 0x1C, 0x01, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xAA, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x91, 0xB7, 0xAA, 0x55};
        
        //Direct command example from Gaizka's code
        uint8_t test_buffer[32] = {0xAA, 0x55, 0x21, 0x2C, 0x14, 0x01, 0x00, 0x00, 0xE0, 0x1C, 0x00, 0x00, 0x38, 0x51, 0x54, 0x7A, 0x70, 0xA3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0x12, 0xAA, 0x55};
        //Telemetry example
        //uint8_t test_buffer[32] = {0xAA, 0x55, 0x02, 0x01, 0x14, 0x10, 0x00, 0x10, 0x42, 0x2D, 0x2E, 0x63, 0xC0, 0x3E, 0x55, 0x8F, 0x00, 0x00, 0x00, 0xF5, 0x42, 0xB4, 0x00, 0x00, 0x42, 0xB4, 0x00, 0x00, 0x2F, 0xEF, 0xAA, 0x55};
        
        //Telemetry example, single COBS field
        //uint8_t test_buffer[32] = {0xAA, 0x55, 0x02, 0x01, 0x14, 0x10, 0x00, 0x10, 0x42, 0x2D, 0x4A, 0xFC, 0xC0, 0x41, 0x18, 0x6A, 0x00, 0x00, 0x00, 0xC8, 0x42, 0x34, 0x00, 0x00, 0x42, 0xB4, 0x00, 0x00, 0x76, 0x62, 0xAA, 0x55};

        //Telemetry example, two COBS fields
        //uint8_t test_buffer[32] = {0xAA, 0x55, 0x02, 0x01, 0x14, 0x10, 0x00, 0x10, 0x42, 0x2D, 0x4A, 0xFC, 0xC0, 0x41, 0x18, 0x6A, 0x00, 0x12, 0x00, 0x00, 0x42, 0x34, 0x00, 0x00, 0x42, 0xB4, 0x00, 0x00, 0x42, 0xB0, 0xAA, 0x55};

        uint8_t rx_type = 0;
        uint8_t rx_len = 0;
        uint8_t rx_buffer[32]; // Allocate memory!

        uint8_t deserialize_result = tap.deserialize(test_buffer, 32, &rx_type, &rx_len, rx_buffer);
        if(deserialize_result != tap.TAP_OK){
            printf("OI YOU WERE NOT SUPPOSED TO DO THAT!\n");
            return(1);
        }

        switch(rx_type) {
            case TAP::DIRECT_COMMAND: {

                TAP::TAP_DIRECT_COMMAND received_direct_command;
                memcpy(&received_direct_command, rx_buffer, sizeof(TAP::TAP_DIRECT_COMMAND));

                direct_command_worker(received_direct_command);
                break;
            }
            default:
                break;
        }
        return 0;
    }
    return 1;
}


// GPIO Initialisation should all happen here pls thx
int pico_gpio_init(void) {

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    //UART0 - TAP COMMUNICATION
    // ==================================================================================== //
    uart_init(TAP_UART_ID, 115200);
    gpio_set_function(0, UART_FUNCSEL_NUM(TAP_UART_ID, 0));
    gpio_set_function(1, UART_FUNCSEL_NUM(TAP_UART_ID, 1));

    //Uart parity, fifo, format... settings
    uart_set_hw_flow(TAP_UART_ID, false, false);
    uart_set_format(TAP_UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(TAP_UART_ID, false);

    
    //UART receiving causes interrupts
    int UART_IRQ_0 = TAP_UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ_0, tap_rx_irq);
    irq_set_enabled(UART_IRQ_0, true);
    uart_set_irq_enables(TAP_UART_ID, true, false);
    return PICO_OK;
}

// Decor LED, it lets you know (by blinking twice) that the whole thing has not died
uint8_t pico_set_led() {
    if(to_ms_since_boot(get_absolute_time()) - ms_last_change <= 1000){
        if((to_ms_since_boot(get_absolute_time()) - ms_last_change >= 200 && to_ms_since_boot(get_absolute_time()) - ms_last_change <= 275)||(to_ms_since_boot(get_absolute_time()) - ms_last_change >= 325 && to_ms_since_boot(get_absolute_time()) - ms_last_change <= 400)){
            gpio_put(PICO_DEFAULT_LED_PIN, true);
        }
        else{
            gpio_put(PICO_DEFAULT_LED_PIN,false);
        }
    }
    else{
        ms_last_change = to_ms_since_boot(get_absolute_time());
    }
    return(0);
}

int main() {
    stdio_init_all();
    sleep_ms(1000);
    int rc = pico_gpio_init();
    hard_assert(rc == PICO_OK);

    sleep_ms(100);
    pico_set_led();

    char testByte = (char)0;
    
    while (true) {
        
        //Lighting effects, internally scheduled. To be improved.
        pico_set_led();

        //Sends a TAP message over USB Uart with GPS data
        tap_telemetry();

        //Decodes a TAP message
        tap_rx_process();

        //Reads from USB, polling

    }
    return(0);
}
