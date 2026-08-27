#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "string.h"
#include "GPS.h"

GPS::~GPS(){
}
GPS::GPS(i2c_inst* i2c, bool init_extern, uint8_t SCL, uint8_t SDA){

    inputBus = gps_bus_i2c;

    if(!init_extern){
        i2c_init(i2c, 400*1000);
    }
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_pull_up(SCL);
    gpio_pull_up(SDA);
}

GPS::GPS(uart_inst_t* uart, bool init_extern, uint8_t tx, uint8_t rx){
    printf("[DEBUG] GPS says hi :P\n");
    inputBus = gps_bus_uart;
    this->uart_id = uart;
    gpio_set_function(tx, GPIO_FUNC_UART);
    gpio_set_function(rx, GPIO_FUNC_UART);

    // Initialize UART if it hasn't been externally initialised
    // You are still expected to create the interrupt handler yourself!
    if(!init_extern){
        uart_init(uart, 9600);
        gpio_set_function(tx, GPIO_FUNC_UART);
        gpio_set_function(rx, GPIO_FUNC_UART);
        uart_set_hw_flow(uart, false, false);
        uart_set_format(uart, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(uart, false);
    }
    printf("[DEBUG] GPS says bosh!! :D\n");
}


void GPS::i2c_init(i2c_inst* i2c, int32_t freq){
    char init_command[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";
    printf("Initialising GPS module...\n");
    write_commmand(init_command,sizeof(init_command));
}

void GPS::write_commmand(char command[], int command_length){
    uint8_t command_in_byte;

    for(int i=0;i<command_length;i++){
        command_in_byte = command[i];
        i2c_write_blocking(i2c0,addr,&command_in_byte,1,true);
    }
}
void GPS::i2c_read(){
    uint8_t buffer[max_read];
 
    int i = 0;
    bool complete = false;

    i2c_read_blocking(i2c_default, addr, buffer, max_read, false);

    // Convert bytes to characters
    while (i < max_read && complete == false) {
        GPS::buffer[i] = buffer[i];
        // Stop converting at end of message 
        if (buffer[i] == 10 && buffer[i + 1] == 10) {
            complete = true;
        }
        i++;
    }
}

void GPS::uart_irq_handler() {
    // Get the GPS instance
    extern GPS *gps_instance;

    if (uart_is_readable(gps_instance->uart_id)) {
        uint8_t ch = uart_getc(gps_instance->uart_id);
        gps_instance->uart_feeder(ch);
    }
}

void GPS::uart_feeder(uint8_t byte){
    if(byte == '$'){
        printf("[DEBUG] [GNSS SENTENCE]:");
        for(size_t i = 0; i<buffer_cursor; i++){
            printf("%02x", buffer[i]);
        }
        printf("\n");
        output_gdata = parse_string();
        buffer_cursor = 0;
        memset(buffer, 0, sizeof(buffer));
    }
    buffer[buffer_cursor] = byte;
    buffer_cursor++;
}

struct gps_data GPS::parse_string(){
    
    char protocol[]="GNRMC";
    // Finds location of protocol message in output
    char *com_index = strstr(buffer, protocol);
    int p = com_index - buffer;
 
    // Splits components of output sentence into array
    #define NO_OF_FIELDS 14
    #define MAX_LEN 15
 
    int n = 0;
    int m = 0;
 
    char gps_data[NO_OF_FIELDS][MAX_LEN];
    memset(gps_data, 0, sizeof(gps_data));
 
    bool complete = false;
    while (buffer[p] != '$' && n < MAX_LEN && complete == false) {
        if (buffer[p] == ',' || buffer[p] == '*') {
            n += 1;
            m = 0;
        } else {
            gps_data[n][m] = buffer[p];
            // Checks if sentence is complete
            if (m < NO_OF_FIELDS) {
                m++;
            } else {
                complete = true;
            }
        }
        p++;
    }
    
    struct gps_data gdata;
      

    sprintf(gdata.protocol,"%s",gps_data[0]);
    sprintf(gdata.time,"%s", gps_data[1]);
    gdata.status = gps_data[2][0];
    char lat_deg[3], lat_min[8], lon_deg[4], lon_min[8];
    
    lat_deg[0] = gps_data[3][0];
    lat_deg[1] = gps_data[3][1];
    lat_deg[2] = '\0';

    lat_min[0] = gps_data[3][2];
    lat_min[1] = gps_data[3][3];
    lat_min[2] = gps_data[3][4];
    lat_min[3] = gps_data[3][5];
    lat_min[4] = gps_data[3][6];
    lat_min[5] = gps_data[3][7];
    lat_min[6] = gps_data[3][8];
    lat_min[7] = '\0';

    lon_deg[0] = gps_data[5][0];
    lon_deg[1] = gps_data[5][1];
    lon_deg[2] = gps_data[5][2];    
    lon_deg[3] = '\0';

    lon_min[0] = gps_data[5][3];
    lon_min[1] = gps_data[5][4];
    lon_min[2] = gps_data[5][5];
    lon_min[3] = gps_data[5][6];
    lon_min[4] = gps_data[5][7];
    lon_min[5] = gps_data[5][8];
    lon_min[6] = gps_data[5][9];
    lon_min[7] = '\0';

    double temp_lat_deg = atof (lat_deg);
    double temp_lat_min = atof (lat_min);
    gdata.latitude = temp_lat_deg + (temp_lat_min/60);
    gdata.NSIndicator = gps_data[4][0];
    double temp_lon_deg = atof (lon_deg);
    double temp_lon_min = atof (lon_min);
    gdata.longitude = temp_lon_deg + (temp_lon_min/60);
    gdata.EWIndicator = gps_data[6][0];
    gdata.speed = atof(gps_data[7]);
    gdata.course = atof(gps_data[8]);
    sprintf(gdata.date,"%s", gps_data[9]);

    return gdata;
}

gps_data GPS::pollGPS(){
    if(inputBus == gps_bus_uart){
        return(output_gdata);
    }
    else{
        //TODO: I2C POLL
        return(output_gdata);
    }
}