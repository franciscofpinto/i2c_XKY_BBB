/**
 * @file
 * @author
 * @brief I2C Master module
 */

//Bibliotecas XKY
#include <xky.h>
#include <bare.h>
#include <xky_printf.h> //---->#include <pprintf.h>
#include <i2c_driver.h>

#include "gpio_v2.h"

// #include <clock_module.h>
// #include <control_module.h>
// #include <GPIO.h>
// #include <pad.h>

#define BUFFER_LEN                          (100)

//Funções utilizadas
void i2c_init(unsigned int dev_id, unsigned int address);
int i2c_master_send(unsigned int dev_id, unsigned int slave_addr, char *data, int len);
int i2c_master_receive(unsigned int dev_id, unsigned int slave_addr, char *data, int nbyte);

//Defnição de registos
char own_addr = 0x40;
char temp_sensor_addr = 0x18;
char read_temp_register[] = {0x30,0x05};

int i2c_instance = 1;


/**
 * @brief Partition Entry point
 * @return Ignored
 */
int entry_point() {

    xky_printf("    :: Enabling I2C as master... "); 

    //enable interrupts
    xky_syscall_arm_enable_interrupts();

    //Initializes bus of I2C and gives address ox40 to this device
    i2c_init(1,0x40); //Mudar eventualmente para identificação do endereço

    xky_printf("DONE\n");

    while (1) {

        while (1) {
            
            int size = 2;


            float temperature;
            //Print to warn about send
            int bytes_sent = i2c_master_send(i2c_instance,temp_sensor_addr,read_temp_register,size);
            
            //Verificação de envio
            if(bytes_sent != 1){
                xky_printf("[ERROR] Expected to send 1 byte, sent %d instead\n", bytes_sent);
            }
            else{
                xky_printf("[OK] Message sent successfully. Reading data...\n");

                char data_received[2];
              }  //int bytes_received = i2c_master_receive(i2c_instance,temp_sensor_addr,data_received,2);
            
            // if(bytes_received != 2){

            //         xky_printf("[ERROR] Expected to send 2 bytes, received %d instead\n", bytes_received);

                // }else{

                // xky_printf("[OK] Data was successfully receive. Calculating...\n");

                // //fetches the signal bit (bit 12)----> mascara para fazer a leitura dos bits 12 a 0, que são os correspondentes aos indicadores de temperatura
                // int is_negative = data_received[0] & 0x10;

                // //clear threshold flags and signal bits (bit 15 to 12)
                // data_received[0] = data_received[0] & 0x0F;
                // temperature = (data_received[0] * 16.0) + (data_received[1]/16.0);

                // if(is_negative == 1){
                //     temperature = 256 - temperature;
                // }
                //}
            //}

            //partição fica em modo sleep até a proxima janela
            bare_wake_in_next_window();

        }      
            
    }
           

    return 0;
}
