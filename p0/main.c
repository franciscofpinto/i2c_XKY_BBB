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
char read_accelerometer_register_XH[] = {0x3B};    //Accelerometer values access register of MPU6505
char read_accelerometer_register_XL[] = {0x3C}; 
char read_accelerometer_register_YH[] = {0x3D};
char read_accelerometer_register_YL[] = {0x3E};
char read_accelerometer_register_ZH[] = {0x3F};
char read_accelerometer_register_ZL[] = {0x40};
char gyro_address[] = {0x43};

char acc_reset[] = {0x6B,0x00};

char gyro_config[] = {0x1B,0x08};               //Gyro config address and set FS_SEL to the +-1000°/s 
char acc_config[] = {0x1C,0x00};                 //Acc config address
char enable_fifo[] = {0x23,0x08};               //Enable FIFO

char  gyroscope_measurements_X[] = {0x43};


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
        // //Starts MCP9808 temperature calculations
        // float temperature = MCP9808_temp_calculation();

        // //Prints de value of the temperature read by MCP9808 
        // xky_printf("Temperature = %f\n",temperature);

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

        //fetches the signal bit (bit 12) to check if it corresponds to negative or positive temperature values
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

    
    //gyro config
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_config,2);

    //acc config
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,acc_config,2);

    //Enable FIFO 
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,enable_fifo,2);

}

/**
 * @brief MPU6505 data calculations
 * @return 
 */
void MPU6505_accelerometer_calculation(){


    //Identificação do endereço do sensor
    // char who_am_I[] = {0x75}; 
    // int size_of_bytes_sent = 1;
    // i2c_master_send(i2c_instance,accelerometer_sensor_addr,who_am_I,size_of_bytes_sent);
    // int size_of_bytes_received1 = 1;
    // char data_received2[] = {0x00};
    // int bytes_received = i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received2,size_of_bytes_received1);
    // xky_printf("Endereço do sensor = %02x\n",data_received2[0]);

    //Initializes MPU6505 sensor
     //void MPU6505_initialization();

    //Power reset do MPU6505
    int bytes_sent =i2c_master_send(i2c_instance,accelerometer_sensor_addr,acc_reset,2);

    //=== Read accelrometer data === //
    //Leitura eixo X_H
    int size_of_bytes_sent = 1;
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_XH,size_of_bytes_sent);
    int size_of_bytes_received = 1;
    char data_received[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received,size_of_bytes_received);
    char XAxis_H = data_received[0];
   
    //Leitura eixo X_L
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_XL,size_of_bytes_sent);
    char data_received1[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received1,size_of_bytes_received);     
    char XAxis_L = data_received1[0];
    float XAxisFull = ((XAxis_H << 8) | XAxis_L) / 16384.0;
     
    //Leitura eixo Y_H
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_YH,size_of_bytes_sent);
    char data_received2[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received2,size_of_bytes_received);
    char YAxis_H = data_received2[0];
    
    //Leitura eixo Y_L
     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_YL,size_of_bytes_sent);
    char data_received3[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received3,size_of_bytes_received);
    char YAxis_L = data_received3[0];
    float YAxisFull = ((YAxis_H << 8) | YAxis_L) / 16384.0;
    
    //Leitura eixo Z_H
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_ZH,size_of_bytes_sent);
    char data_received4[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received4,size_of_bytes_received);
    char ZAxis_H = data_received4[0];
    

    //Leitura eixo Z_H
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_ZL,size_of_bytes_sent);
    char data_received5[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received5,size_of_bytes_received);
    char ZAxis_L = data_received5[0];
    float ZAxisFull = ((ZAxis_H << 8) | ZAxis_L) / 16384.0 ;

    xky_printf("XAxis_FULL = %f\n",XAxisFull);
    xky_printf("YAxis_FULL = %f\n",YAxisFull);
    xky_printf("ZAxis_FULL = %f\n",ZAxisFull);

    // float accAngle_X = (atan(YAxisFull / sqrt(pow(XAxisFull, 2) + pow(ZAxisFull, 2))) * 180 / 3.14) - 0.58;
    // xky_printf("accAngle_X = %f\n",accAngle_X);

    //=== Read gyroscope data === //
     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_XH[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_XH,size_of_bytes_received);
    char GyroXH = data_received_gyro_XH[0];

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_XL[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_XL,size_of_bytes_received);
    char GyroXL= data_received_gyro_XL[0];
    float GyroXFull = ((GyroXH << 8) | GyroXL) / 131.0;

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_YH[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_YH,size_of_bytes_received);
    char GyroYH = data_received_gyro_YH[0];

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_YL[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_YL,size_of_bytes_received);
    char GyroYL = data_received_gyro_YL[0];
    float GyroYFull = ((GyroYH << 8) | GyroYL) / 131.0;

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_ZH[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_ZH,size_of_bytes_received);
    char GyroZH = data_received_gyro_ZH[0];

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_ZL[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_ZL,size_of_bytes_received);
    char GyroZL = data_received_gyro_ZL[0];
    float GyroZFull = ((GyroZH << 8) | GyroZL) / 131.0;

    xky_printf("GyroXFULL = %f\n",GyroXFull);
    xky_printf("GyroYFULL = %f\n",GyroYFull);
    xky_printf("GyroZFULL = %f\n",GyroZFull);




}

   
