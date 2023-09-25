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
#define IMU_register_resolution                      2047.97
#define acc_resolution 32

void i2c_init(unsigned int dev_id, unsigned int address);
int i2c_master_send(unsigned int dev_id, unsigned int slave_addr, char *data, int len);
int i2c_master_receive(unsigned int dev_id, unsigned int slave_addr, char *data, int nbyte);
float MCP9808_temp_calculation();
int message_sent_verification(int bytes_sent, int size);
void MPU6505_accelerometer_calculation();
void calculate_IMU_error();
void calculo_OFFSET();


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
char acc_config[] = {0x1C,0xE0};                 //Acc config address ---> +/-2g
char enable_fifo[] = {0x23,0x08,0x78};               //Enable FIFO
char  gyroscope_measurements_X[] = {0x43};

char x_reset[] = {0x0D};
char y_reset[] = {0x0E};
char z_reset[] = {0x0F};


float GyroError_X;
float GyroError_Y;
float GyroError_Z;
float AccError_X;
float AccError_Y;
float total_range = 4.0; //Para 4.0 para de +2g a -2g
int maximum_value = 2; //Valor máximo em G


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
    calculo_OFFSET();

    while (1) {
        // //Starts MCP9808 temperature calculations
        //float temperature = MCP9808_temp_calculation();

        // //Prints de value of the temperature read by MCP9808 
         //xky_printf("Temperature = %f\n",temperature);

        //Starts MPU6505 calculation
        MPU6505_accelerometer_calculation();

        //calculate_IMU_error();

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
    char data_received_T[] = {0x00,0x00};
    int bytes_received = i2c_master_receive(i2c_instance,temp_sensor_addr,data_received_T,size_of_bytes_received);

    //if the message was sent successfully from the master it starts the calculation of the data
    if(message_sent_verification(bytes_sent,size_of_bytes_sent) == 1 ){
        //Prints the hexadecimal value of the data received (Byte 1 and 2)
        xky_printf("Data_received[0] = %02x\n",data_received_T[0]);
        xky_printf("Data_received[1] = %02x\n",data_received_T[1]);

        //fetches the signal bit (bit 12) to check if it corresponds to negative or positive temperature values
        is_negative = data_received_T[0] & 0x10;

        //clear threshold flags and signal bits (bit 15 to 12)
        data_received_T[0] = data_received_T[0] & 0x0F;

        //Positive temperature values calculation
        temperature = (data_received_T[0] * 16.0) + (data_received_T[1]/16.0);

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

    //Enable FIFO 
    //i2c_master_send(i2c_instance,accelerometer_sensor_addr,enable_fifo,3);

    //Power reset do MPU6505
    int bytes_sent =i2c_master_send(i2c_instance,accelerometer_sensor_addr,acc_reset,2);

     //gyro config
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_config,2);

    //acc config
    i2c_master_send(i2c_instance,accelerometer_sensor_addr,acc_config,2);

    //self-test 
    // bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,x_reset,1);
    // bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,y_reset,1);
    // bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,z_reset,1);

    //// Call this function to get the IMU error values
    //calculate_IMU_error();
    

    
    

    //=== Read accelerometer data === //
    //X axis reading
    int size_of_bytes_sent = 1;
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_XH,size_of_bytes_sent);
    int size_of_bytes_received = 1;
    char data_received[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received,size_of_bytes_received);
    char XAxis_H = data_received[0];
    xky_printf("XAxis_H = %02x\n",XAxis_H);
   
    //Leitura eixo X_L
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_XL,size_of_bytes_sent);
    char data_received1[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received1,size_of_bytes_received);     
    char XAxis_L = data_received1[0];
    xky_printf("XAxis_L = %02x\n",XAxis_L);

    float XAxisFull = (((XAxis_H << 8) | XAxis_L)  * ( total_range / 65535.0)) - maximum_value;
     
    //Leitura eixo Y_H
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_YH,size_of_bytes_sent);
    char data_received2[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received2,size_of_bytes_received);
    char YAxis_H = data_received2[0];
    xky_printf("YAxis_H = %02x\n",YAxis_H);
    
    //Leitura eixo Y_L
     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_YL,size_of_bytes_sent);
    char data_received3[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received3,size_of_bytes_received);
    char YAxis_L = data_received3[0];
    xky_printf("YAxis_L = %02x\n",YAxis_L);
    float YAxisFull = (((YAxis_H << 8) | YAxis_L)  *( total_range / 65535.0)) - maximum_value;
    
    //Leitura eixo Z_H
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_ZH,size_of_bytes_sent);
    char data_received4[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received4,size_of_bytes_received);
    char ZAxis_H = data_received4[0];
    xky_printf("ZAxis_H = %02x\n",ZAxis_H);
    
    //Leitura eixo Z_H
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_ZL,size_of_bytes_sent);
    char data_received5[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received5,size_of_bytes_received);
    char ZAxis_L = data_received5[0];
    xky_printf("ZAxis_L = %02x\n",ZAxis_L);
    float ZAxisFull = (((ZAxis_H << 8) | ZAxis_L) *( total_range / 65535.0)) - maximum_value;

    xky_printf("XAxis_FULL = %f\n",XAxisFull);
    xky_printf("YAxis_FULL = %f\n",YAxisFull);
    xky_printf("ZAxis_FULL = %f\n",ZAxisFull);

    // float accAngle_X = (atan(YAxisFull / sqrt(pow(XAxisFull, 2) + pow(ZAxisFull, 2))) * 180 / 3.14) - AccError_X;
    // float accAngle_Y = (atan(-1 * YAxisFull / sqrt(pow(XAxisFull, 2) + pow(ZAxisFull, 2))) * 180 / 3.14) + AccError_Y; // AccErrorY ~(-1.58)

    // xky_printf("accAngle = %f\n",accAngle_X);
   

    //=== Read gyroscope data === //
     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_XH[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_XH,size_of_bytes_received);
    char GyroXH = data_received_gyro_XH[0];

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_XL[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_XL,size_of_bytes_received);
    char GyroXL= data_received_gyro_XL[0];
    float GyroXFull = (((GyroXH << 8) | GyroXL) / 131)-500;

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_YH[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_YH,size_of_bytes_received);
    char GyroYH = data_received_gyro_YH[0];

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_YL[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_YL,size_of_bytes_received);
    char GyroYL = data_received_gyro_YL[0];
    float GyroYFull = (((GyroYH << 8) | GyroYL) / 131) - 500;

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_ZH[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_ZH,size_of_bytes_received);
    char GyroZH = data_received_gyro_ZH[0];

     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
    char data_received_gyro_ZL[] = {0x00};
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_ZL,size_of_bytes_received);
    char GyroZL = data_received_gyro_ZL[0];
    float GyroZFull = (((GyroZH << 8) | GyroZL) / 131)-500;

    xky_printf("GyroXFULL = %f\n",GyroXFull);
    xky_printf("GyroYFULL = %f\n",GyroYFull);
    xky_printf("GyroZFULL = %f\n",GyroZFull);

}

// void calculate_IMU_error() {

//   // We can call this funtion to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial.
//   // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
//   // Read accelerometer values 200 times

//   //Power reset do MPU6505
//     int bytes_sent =i2c_master_send(i2c_instance,accelerometer_sensor_addr,acc_reset,2);
  
//   int c = 0;
//   while (c < 200) {

//      //=== Read accelrometer data === //
//     //Leitura eixo X_H
//     int size_of_bytes_sent = 1;
//     int bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_XH,size_of_bytes_sent);
//     int size_of_bytes_received = 1;
//     char data_received[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received,size_of_bytes_received);
//     char XAxis_H = data_received[0];
   
//     //Leitura eixo X_L
//     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_XL,size_of_bytes_sent);
//     char data_received1[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received1,size_of_bytes_received);     
//     char XAxis_L = data_received1[0];
//     float XAxisFull = ((XAxis_H << 8) | XAxis_L) / 16384.0;
     
//     //Leitura eixo Y_H
//     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_YH,size_of_bytes_sent);
//     char data_received2[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received2,size_of_bytes_received);
//     char YAxis_H = data_received2[0];
    
//     //Leitura eixo Y_L
//      bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_YL,size_of_bytes_sent);
//     char data_received3[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received3,size_of_bytes_received);
//     char YAxis_L = data_received3[0];
//     float YAxisFull = ((YAxis_H << 8) | YAxis_L) / 16384.0;
    
//     //Leitura eixo Z_H
//     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_ZH,size_of_bytes_sent);
//     char data_received4[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received4,size_of_bytes_received);
//     char ZAxis_H = data_received4[0];
    
//     //Leitura eixo Z_H
//     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,read_accelerometer_register_ZL,size_of_bytes_sent);
//     char data_received5[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received5,size_of_bytes_received);
//     char ZAxis_L = data_received5[0];
//     float ZAxisFull = ((ZAxis_H << 8) | ZAxis_L) / 16384.0 ;

//     // Sum all readings
//     AccError_X = AccError_X + (atan(YAxisFull / sqrt(pow(XAxisFull, 2) + pow(ZAxisFull, 2))) * 180 / 3.14);
//     AccError_Y = AccError_Y + ((atan(-1 * (XAxisFull) / sqrt(pow((YAxisFull), 2) + pow((ZAxisFull), 2))) * 180 / 3.14));
//     c++;

//     xky_printf("AccErrorX: %f\n",AccError_X);
//     xky_printf("AccErrorY: %f\n",AccError_Y);
//   }

//   //Divide the sum by 200 to get the error value
//   AccError_X = AccError_X / 200;
//   AccError_Y = AccError_Y / 200;
//   c = 0;

//   // Read gyro values 200 times
//   while (c < 200) {

//     //=== Read gyroscope data === //
//     int size_of_bytes_sent = 1;
//      i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
//     char data_received_gyro_XH[] = {0x00};
//     int size_of_bytes_received = 1;
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_XH,size_of_bytes_received);
//     char GyroXH = data_received_gyro_XH[0];

//     i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
//     char data_received_gyro_XL[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_XL,size_of_bytes_received);
//     char GyroXL= data_received_gyro_XL[0];
//     float GyroXFull = ((GyroXH << 8) | GyroXL) / 131.0;

//     i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
//     char data_received_gyro_YH[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_YH,size_of_bytes_received);
//     char GyroYH = data_received_gyro_YH[0];

//     i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
//     char data_received_gyro_YL[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_YL,size_of_bytes_received);
//     char GyroYL = data_received_gyro_YL[0];
//     float GyroYFull = ((GyroYH << 8) | GyroYL) / 131.0;

//     i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
//     char data_received_gyro_ZH[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_ZH,size_of_bytes_received);
//     char GyroZH = data_received_gyro_ZH[0];

//     i2c_master_send(i2c_instance,accelerometer_sensor_addr,gyro_address,size_of_bytes_sent);
//     char data_received_gyro_ZL[] = {0x00};
//     i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_gyro_ZL,size_of_bytes_received);
//     char GyroZL = data_received_gyro_ZL[0];
//     float GyroZFull = ((GyroZH << 8) | GyroZL) / 131.0;

//     // Sum all readings
//     GyroError_X = GyroError_X + (GyroXFull / 131.0);
//     GyroError_Y = GyroError_Y + (GyroYFull / 131.0);
//     GyroError_Z = GyroError_Z + (GyroZFull / 131.0);

//     c++;

//     xky_printf("GyroError_X: %f\n",GyroError_X);
//     xky_printf("GyroError_Y: %f\n",GyroError_Y);
//     xky_printf("GyroError_Z: %f\n",GyroError_Z);
//   }

//   //Divide the sum by 200 to get the error value
//   GyroError_X = GyroError_X / 200;
//   GyroError_Y = GyroError_Y / 200;
//   GyroError_Z = GyroError_Z / 200;

// // Print the error values on the Serial 
// xky_printf("AccErrorX: %f\n",AccError_X);
// xky_printf("AccErrorY: %f\n",AccError_Y);
// xky_printf("GyroError_X: %f\n",GyroError_X);
// xky_printf("GyroError_Y: %f\n",GyroError_Y);
// xky_printf("GyroError_Z: %f\n",GyroError_Z);
  
// }

void calculo_OFFSET(){


//***************************Verificação de valores de OFFSET************************************//
    int size_of_bytes_sent = 2;
    char XA_OFFSET_H[] = {0x06,0x7F};
    char endereco_XA_h[] = {0x06};
    int bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,XA_OFFSET_H,size_of_bytes_sent);
    int size_of_bytes_received = 1;
    char data_received_offset_1[] = {0x00};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,endereco_XA_h,1);
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_offset_1,size_of_bytes_received);     
    xky_printf("XA_OFFSET_H = %02x\n",data_received_offset_1[0]);

    char XA_OFFSET_L[] = {0x07,0xFF};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,XA_OFFSET_L,size_of_bytes_sent);
    size_of_bytes_received = 1;
    char data_received_offset_2[] = {0x00};
    char endereco_XA_l[] = {0x07};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,endereco_XA_l,1);
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_offset_2,size_of_bytes_received);     
    xky_printf("XA_OFFSET_L = %02x\n",data_received_offset_2[0]);


    char YA_OFFSET_H[] = {0x08,0X7F};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,YA_OFFSET_H,size_of_bytes_sent);
    size_of_bytes_received = 1;
    char data_received_offset_3[] = {0x00};
    char endereco_YA_h[] = {0x08};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,endereco_YA_h,1);
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_offset_2,size_of_bytes_received);     
    xky_printf("YA_OFFSET_H = %02x\n",data_received_offset_3[0]);

    char YA_OFFSET_L[] = {0x09,0XFF};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,YA_OFFSET_L,size_of_bytes_sent);
    size_of_bytes_received = 1;
    char data_received_offset_4[] = {0x00};
     char endereco_YA_L[] = {0x09};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,endereco_YA_L,1);
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_offset_2,size_of_bytes_received);     
    xky_printf("YA_OFFSET_L = %02x\n",data_received_offset_4[0]);

    char ZA_OFFSET_H[] = {0x0A,0X7F};
     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,ZA_OFFSET_H,size_of_bytes_sent);
     size_of_bytes_received = 1;
    char data_received_offset_5[] = {0x00};
     char endereco_ZA_h[] = {0x0A};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,endereco_ZA_h,1);
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_offset_5,size_of_bytes_received);     
    xky_printf("ZA_OFFSET_H = %02x\n",data_received_offset_5[0]);

    char ZA_OFFSET_L[] = {0x0B,0XFF};
     bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,ZA_OFFSET_L,size_of_bytes_sent);
     size_of_bytes_received = 1;
    char data_received_offset_6[] = {0x00};
    char endereco_ZA_l[] = {0x0B};
    bytes_sent=i2c_master_send(i2c_instance,accelerometer_sensor_addr,endereco_ZA_l,1);
    i2c_master_receive(i2c_instance,accelerometer_sensor_addr,data_received_offset_6,size_of_bytes_received);     
    xky_printf("ZA_OFFSET_H = %02x\n",data_received_offset_6[0]);


}
