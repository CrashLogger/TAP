#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "string.h"

#ifndef __GPS_H_
#define __GPS_H_

struct gps_data{
    char protocol[15];
    char time[15];
    char status;
    double latitude;
    char NSIndicator;
    double longitude;
    char EWIndicator;
    double speed;
    double course;
    char date[15]; //only uses 6 characters, one by one
    char magneticVar[15];
    char EWdegreeIndicator[15];
    char mode[15];
    char checksum[15];
};

typedef enum inputBuses{
    gps_bus_none = 0,
    gps_bus_uart = 1,
    gps_bus_i2c = 2,
    gps_bus_tap_uart = 3
}inputBus_t;

class GPS{

    private:
    //VARS
        inputBus_t inputBus = gps_bus_none;
        const uint8_t addr = 0x10;
        const int max_read = 250;
        char buffer[250];
        size_t buffer_cursor = 0;        

    //FUNCS
        /**
         * @brief Initializes I²C based GPS devices
         * @param i2c i2c bus pointer to use
         * @param freq frequency at which the bus is to operate
         */
        void i2c_init(i2c_inst* i2c, int32_t freq);
        void write_commmand(char command[], int command_length);
        void i2c_read();
        
        /**
         * @brief Parses a GNSS string to produce a full gps_data struct
         */
        struct gps_data parse_string();

        /**
         * @brief Internal interrupt handler
         */
        void uart_irq_handler();

    public:
    //VARS
        uart_inst_t* uart_id;
        gps_data output_gdata;

    //FUNCS
        /**
         * @brief Initializes a GPS object to read from I²C, such as the adafruit devices
         * @param i2c_inst i2c bus pointer to use
         * @param init_extern if the i2c bus is also used by other things, set this to true, it won't be re-initialized
         * @param SCL SCL pin to use
         * @param SDA SDA pin to use
         * 
         * You're in charge of using the right SCL and SDA for your hardware!
         */
        GPS(i2c_inst* i2c, bool init_extern, uint8_t SCL, uint8_t SDA);
        /**
         * @brief
         * Initializes a GPS object to read from UART, such as the uBlox NEO6M series
         * 
         * @param uart UART bus pointer to use
         * @param init_extern if the UART bus is also used by other things, set this to true, it won't be re-initialized
         * @param tx TX pin for the UART bus
         * @param rx RX pin for the UART bus
         * 
         * You're in charge of using the right TX and RX for your hardware! Well, mostly the RX as GPS is not even duplex.
         */
        GPS(uart_inst_t* uart, bool init_extern, uint8_t tx, uint8_t rx);
        ~GPS();

        /**
         * @brief Returns the latest available gps reading
         * On advanced devices, it re-polls to get more updated information.
         * On simpler devices, it will simply provide the latest data available, whatever that may be.
         */
        gps_data pollGPS();

        /**
         * @brief Method that takes a byte from UART interrupt handling
         * 
         * @param byte byte taken from UART
         */
        void uart_feeder(uint8_t byte);

};


#endif