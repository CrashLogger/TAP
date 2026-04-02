#include "pico/stdlib.h"
#include <stdio.h>
#include "string.h"
#include "TAP.h"

//===== DEFINITIONS

//Simple I/O
#define TAIL_LIGHT 13                   //COMMON CONTROL: NAV_LIGHTS
#define STARBOARD_LIGHT 12              //COMMON CONTROL: NAV_LIGHTS
#define PORT_LIGHT 11                   //COMMON CONTROL: NAV_LIGHTS
#define NAV_LIGHTS 16                   //TAIL, STARBOARD AND PORT LIGHTS
#define STROBES 17                      //ALL AROUND STROBE LIGHTS
#define RADIO_LINK_LOSS_INDICATOR 14    //UNUSED IN CURRENT PROTOTYPE

//PWM IO, 0 is left side and 1 is right side
#define ESC_PIN 22
#define AIL_0_PIN 17
#define AIL_1_PIN 16
#define RUD_0_PIN 19
#define RUD_1_PIN 18

//GPS NEO6M DEFINTIONS
#define UART_ID uart1
#define TAP_UART_ID uart0
#define BAUD_RATE 9600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 4
#define UART_RX_PIN 5

//DEFINITIONS FOR CONTROL SURFACES
#define VTAIL_GAIN 0.5

//DEFINITIONS FOR TELEMETRY SCALING
#define ALTIMETER_TELEM_PRECISSION 100
#define HEADING_TELEM_PRECISSION 10

//===== GLOBAL VARS =====

//SCHEDULING
uint32_t ms_last_change = 0;
uint32_t ms_last_tap = 0;

// ================================================================================================
// TAP PROTOCOL
// Creating a global object is a good idea and definitely not frowned upon.
TAP tap;

    //The transmission header should be pretty constant
    TAP::TAP_ADDRESS_HEADER our_tx_header;
    TAP::TAP_TRAILER our_tx_trailer;

    //TAP DEFAULTS!
    uint16_t tap_default_sof = 0xAA55;
    uint16_t tap_default_eof = 0x55AA;

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
        telem.alt = 50;
        //telem.heading = 30;

        // Use these to check if COBS works!
        //telem.alt = 0xAA55;
        telem.heading = 0xAA55;

        tap.tapSendTelem(telem);
        return(0);
    }
}

// GPIO Initialisation should all happen here pls thx
int pico_gpio_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
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
    printf("Hello world!");
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

    }
    return(0);
}
