/**
 * @file
 * @author
 * @brief I2C Master module
 */

//XKY Biblio
#include <xky.h>
#include <bare.h>
#include <xky_printf.h> 
#include <i2c_driver.h>
#include "gpio_v2.h"

#define BUFFER_LEN                          (100)

void i2c_init(unsigned int dev_id, unsigned int address);
int i2c_master_send(unsigned int dev_id, unsigned int slave_addr, char *data, int len);
int i2c_master_receive(unsigned int dev_id, unsigned int slave_addr, char *data, int nbyte);
float MCP9808_temp_calculation();
int message_sent_verification(int bytes_sent, int size);
void MPU6505_accelerometer_calculation();


char own_addr = 0x40;                           //BBB address
char temp_sensor_addr = 0x18;                   //MCP9808 default address
char read_temp_register[] = {0x05};             //Temperature access register of MCP9808
int i2c_instance = 2;                           //IC2-2
char accelerometer_sensor_addr = 0x68;          //MPU6505 default address
char read_accelerometer_register[] = {0x3B};    //Accelerometer values access register of MCP9808
char accelerometer_config_register = 0x6B;      //PWR_MGMT_1 register. Allows to configure the power mode of the MPU.

/**
 * @brief Partition Entry point
 * @return Ignored
 */
int entry_point() {

    xky_printf("    :: Enabling I2C as master... "); 

    //Enable interrupts
    xky_syscall_arm_enable_interrupts();

    //Initializes bus of I2C and gives address ox40 to this device
    i2c_init(i2c_instance,own_addr); 

    xky_printf("DONE\n");

    while (1) {
        //Starts MCP9808 temperature calculations
        float temperature = MCP9808_temp_calculation();

        //Prints de value of the temperature read by MCP9808 
        xky_printf("Temperature = %f\n",temperature);

        //Starts MPU6505 calculation
        MPU6505_accelerometer_calculation();

        //Partition in sleep mode until next widnow 
        bare_wake_in_next_window();             
    }

 return 0;
}

/**
 * @brief Verifies if the message was sent successfully from the master
 * @return Ignored
 */
int message_sent_verification(int bytes_sent,int size){

    if(bytes_sent != size){
         xky_printf("[ERROR] Expected to send 1 byte, sent %d instead\n", bytes_sent);
         return -1;
    }
    else{
        xky_printf("[OK] Message sent successfully. Reading data...\n");
        return 1;
    }
}

/**
 * @brief MCP9808 data calculations
 * @return value of the calculated temperature
 */
float MCP9808_temp_calculation(){

    int size_of_bytes_sent = 1;
    float temperature;
    int is_negative;
    int bytes_sent = i2c_master_send(i2c_instance,temp_sensor_addr,read_temp_register,size_of_bytes_sent);
    int size_of_bytes_received = 2;
    char data_received[] = {0x00,0x00};
    int bytes_received = i2c_master_receive(i2c_instance,temp_sensor_addr,data_received,size_of_bytes_received);

    //if the message was sent successfully from the master it starts the calculation of the data
    if(message_sent_verification(bytes_sent,size_of_bytes_sent) == 1 ){

        //Prints the hexadecimal value of the data received (Byte 1 and 2)
        xky_printf("Data_received[0] = %02x\n",data_received[0]);
        xky_printf("Data_received[1] = %02x\n",data_received[1]);

        //fetches the signal bit (bit 12) to check if it corresponds to negative temperature values
        is_negative = data_received[0] & 0x10;

        //clear threshold flags and signal bits (bit 15 to 12)
        data_received[0] = data_received[0] & 0x0F;

        //Positive temperature values calculation
        temperature = (data_received[0] * 16.0) + (data_received[1]/16.0);

        //Negative temperature values calculation
        if(is_negative == 1){
            temperature = 256 - temperature; 
        }

    }

return temperature;
            
}

/**
 * @brief MPU6505 initialization
 * @return 
 */
void MPU6505_initialization(){

    int size_of_bytes_sent = 1;

    //Telling the MPU which register we are writing to (PWR_MGMT_1 register)  
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,accelerometer_config_register,size_of_bytes_sent);

    //Configuration of the Gyroscope. According to page 14 of the register map, the register holding the GYRO_CONFIG settings is located at 0x1B
    //Bit0 - Reserved, set to 0
    //Bit1 - Reserved, set to 0
    //Bit2 - Reserved, set to 0
    //Bit3 - FS_SEL
    //Bit4 - FS_SEL  -  This sets the full scale range of the gyroscope. 
    //Bit5 - ZG_ST - Activates self test of gyroscope Z axis if set to 1. 
    //Bit6 - YG_ST - same as ZG_ST but with Y axis
    //Bit7 - XG_ST - same as ZG_ST but with X axis
    //According to those values, we want to send the value 0 0 0 1 0 0 0 0, which is hex 0x10
    char config_data[] = {0x10};
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,config_data,size_of_bytes_sent);

}

/**
 * @brief MPU6505 data calculations
 * @return 
 */
void MPU6505_accelerometer_calculation(){

    //Initializes MPU6505 sensor
    void MPU6505_initialization();

    int size_of_bytes_sent = 1;
    int bytes_sent = i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register,size_of_bytes_sent);

    int size_of_bytes_received = 2;
    char data_received[] = {0x00,0x00};
    int bytes_received = i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received,size_of_bytes_received);

    if(message_sent_verification(bytes_sent,size_of_bytes_sent) == 1 ){

        char XAxis_H = data_received[0];
        char XAxis_L = data_received[1];

        xky_printf("XAxis = %02x\n",XAxis_H);
        xky_printf("XAxis = %02x\n",XAxis_L);

        //Turn our 8 bit Bytes into a 16 bit integer
        char XAxis_FULL = ((XAxis_H & 0xFF) * 256) | XAxis_L;

        xky_printf("XAxis_FULL = %x\n",XAxis_L);
        
    }

}

   
