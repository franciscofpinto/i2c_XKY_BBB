/**
 * @file
 * @author pfnf
 * @brief I2C driver for the BeagleBone
 */

#include "soc_AM335x.h"
#include "beaglebone.h"
#include "hw_hsi2c.h"
#include "hw_types.h"
#include "hsi2c.h"

#include "i2c_driver.h"
#include <xky.h>
#include <xky_printf.h>
#include <bare.h>    //

#define I2C_TIMEOUT                             (10000)
#define I2C_STATUS_AERR                         (0x00000080u)
#define I2C_STATUS_ROVR                         (0x00000800u)
#define I2C_STATUS_NACK                         (0x00000002u)
#define I2C_STATUS_AL                           (0x00000001u)
#define I2C_STATUS_RRDY                         (0x00000008u)
#define I2C_STATUS_XRDY                         (0x00000010u)
#define I2C_STATUS_ARDY                         (0x00000004u)
#define I2C_STATUS_AAS                          (0x00000200u)
#define I2C_STATUS_ALL                          (0x00007FFFu)
#define I2C_STATUS_ERROR \
        (I2C_STATUS_AERR | I2C_STATUS_ROVR | I2C_STATUS_NACK | I2C_STATUS_AL)

static unsigned int __i2c_own_address = 0x0;

static unsigned int get_base_addr(unsigned int instance) {

    unsigned int ret = 0;

    xky_id_t memory_id_i2c0 = xky_syscall_get_memory_id("I2C0");
    xky_memory_status_t memory_status_i2c0;
    xky_syscall_get_memory_status(memory_id_i2c0, &memory_status_i2c0);
    xky_printf("I2C0 at %08x\n", memory_status_i2c0.address);

    xky_id_t memory_id_i2c1 = xky_syscall_get_memory_id("I2C1");
    xky_memory_status_t memory_status_i2c1;
    xky_syscall_get_memory_status(memory_id_i2c1, &memory_status_i2c1);
    xky_printf("I2C1 at %08x\n", memory_status_i2c1.address);

    xky_id_t memory_id_i2c2 = xky_syscall_get_memory_id("I2C2");
    xky_memory_status_t memory_status_i2c2;
    xky_syscall_get_memory_status(memory_id_i2c2, &memory_status_i2c2);
    xky_printf("I2C2 at %08x\n", memory_status_i2c2.address);

    switch (instance) {

        case 0:
            ret = memory_status_i2c0.address;
            break;
        case 1:
            ret = memory_status_i2c1.address;
            break;
        case 2:
            ret = memory_status_i2c2.address;
            break;
    }
    return ret;
}

/**
 * @brief Get the status of the I2C device
 * @param dev_addr I2C device base address
 * @return Status of I2C
 */
static inline unsigned int i2c_get_status(unsigned int dev_addr) {
    return HWREG(dev_addr + I2C_IRQSTATUS_RAW);
}

/**
 * @brief Clear IRQ status
 * @param dev_addr I2C instance base address
 * @param mask Mask to clear
 */
static inline void i2c_clear_status(unsigned int dev_addr, unsigned int mask) {
    HWREG(dev_addr + I2C_IRQSTATUS) = mask;
}

/**
 * @brief Polls the status of the I2C device
 * @param dev_addr I2C instance base address
 * @param mask Mask to poll
 * @return Status of I2C
 *
 * @note This function blocks if the status never changes
 */
static inline unsigned int i2c_status_poll(unsigned int dev_addr, unsigned int mask) {

    volatile int timeout = 0;
    unsigned int status = 0x0;
    while (((status = i2c_get_status(dev_addr)) & mask) == 0) {
        if (++timeout > I2C_TIMEOUT) {
            xky_printf("Time_out\n");
            return 0;
        }
    }
    return status;
}


static inline void i2c_write_byte(unsigned int dev_addr, char byte) {

    HWREG(dev_addr + I2C_DATA) = byte;
    xky_printf("Valor do i2c_write_byte = %x\n", byte);
}


static inline char i2c_read_byte(unsigned int dev_addr) {

    return HWREG(dev_addr + I2C_DATA) & 0xFF;
}

static inline void i2c_flush_rx_fifo(unsigned int dev_addr) {

    HWREG(dev_addr + I2C_BUF) = I2C_BUF_RXFIFO_CLR;
}

static inline void i2c_flush_tx_fifo(unsigned int dev_addr) {

    HWREG(dev_addr + I2C_BUF) = I2C_BUF_TXFIFO_CLR;
}

static void i2c_soft_reset(unsigned int dev_addr) {

    /* stop any pending operation */
    HWREG(dev_addr + I2C_CON) |= I2C_CON_STP;

    /* disable module */
    HWREG(dev_addr + I2C_CON) = 0x0;

    /* soft reset module */
    HWREG(dev_addr + I2C_SYSC) |= I2C_SYSC_SRST;

    /* disable auto idle */
    HWREG(dev_addr + I2C_SYSC) &= ~I2C_SYSC_AUTOIDLE;

    /* initialize BUS */
    I2CMasterInitExpClk(dev_addr,
            I2C_SYSTEM_CLOCK, I2C_INTERNAL_CLOCK, I2C_OUTPUT_CLOCK);

    /* disable all interrupts */
    HWREG(dev_addr + I2C_IRQENABLE_CLR) = I2C_STATUS_ALL;

    /*
     * According to NetBSD driver and u-boot, these are needed even
     * if just using polling (i.e. non-interrupt driver programming)
     */
    HWREG(dev_addr + I2C_IRQENABLE_SET) = \
            I2C_STATUS_ROVR | I2C_STATUS_AERR | I2C_STATUS_XRDY |
            I2C_STATUS_RRDY | I2C_STATUS_NACK | I2C_STATUS_AL |
            I2C_STATUS_ARDY | I2C_STATUS_AAS;

    /* set device address */
    HWREG(dev_addr + I2C_OA) = __i2c_own_address;


    HWREG(dev_addr + I2C_BUF) = 0x0;

    /* set mode and enable module */
    HWREG(dev_addr + I2C_CON) = I2C_CON_I2C_EN;

    /* wait for activation */
    while ((HWREG(dev_addr + I2C_SYSS) & I2C_SYSS_RDONE) != 0);
}

/**
 * @brief - Checks the state of the I2C bus. When the bit is 0, the bus is free,
 * while if it's 1, the bus is busy.
 * 
 * @param [in] dev_addr - Base address of the I2C instance.
 * 
 * @return - 1 if free; 0 is busy.
 */
static inline int i2c_is_bus_free(unsigned int dev_addr) {

    volatile int timeout = 0;
    xky_printf("checking bus ");
    while ((HWREG(dev_addr + I2C_IRQSTATUS_RAW) & I2C_IRQSTATUS_RAW_BB) != 0) {
        if (++timeout > I2C_TIMEOUT) {
            xky_printf("[BUSY]\n");
            return 0;
        }
    }
    xky_printf("[FREE]\n");
    return 1;
}

/**
 * @brief - Checks if I2C registers are ready to be accessed again. When set to 1 means
 * that the previous command was performed and status was updated. If the number of 
 * attempts passes I2C_TIMEOUT times, the call times out and we assume the device
 * is not ready.
 * 
 * @param [in] dev_addr - Base address of the I2C instance.
 * 
 * @return - 1 if ready; 0 if not ready.
 * 
 */
static inline int i2c_is_device_ready(unsigned int dev_addr) {

    volatile int timeout = 0;
    //xky_printf("is ready ");
    while ((HWREG(dev_addr + I2C_IRQSTATUS_RAW) & I2C_STATUS_ARDY) == 0) {
        if (++timeout > I2C_TIMEOUT) {
    //        xky_printf("[NO]\n");
            return 0;
        }
    }
    //xky_printf("[YES]\n");
    return 1;
}

/**
 * @brief - Initiates an instance of I2C, including configuring the Module clock
 * and respective pins, accroding to the given instance
 * 
 * @param [in] dev_id - I2C instance ID to be initated
 * @param [in] address - The address of the board
 */
void i2c_init(unsigned int dev_id, unsigned int address) {

    /* get base device address */
    unsigned int dev_addr = get_base_addr(dev_id);

    __i2c_own_address = address;

    /* Enable the clock for I2C1 and configure respective pins*/
    //xky_printf("\nConfigurating module clock\n");
    if(dev_id == 1){
        I2C1ModuleClkConfig();
    }else{
        I2C2ModuleClkConfig();
    }
    
    //xky_printf("\niniting pin mux\n");
    I2CPinMuxSetup(dev_id);
    //xky_printf("\ndoing soft reset\n");
    /* reset module */
    i2c_soft_reset(dev_addr);
}

/**
 * @brief - Sends a message with a given size to a specific slave adrees on the
 *  given I2C instance.
 */
int i2c_master_send(unsigned int dev_id, unsigned int slave_addr, char *data, int nbyte) {

    /* get base device address */
    unsigned int dev_addr = get_base_addr(dev_id);

    /* check if the bus is free */
    if (i2c_is_bus_free(dev_addr) == 0) {
        return -1;
    }

    i2c_flush_tx_fifo(dev_addr);

    /* set the data count and slave address */
    xky_printf("Set target address\n");
    HWREG(dev_addr + I2C_CNT) = (unsigned int)nbyte;

    HWREG(dev_addr + I2C_SA) = slave_addr;

    //HWREG(dev_addr + I2C_SA) = 0x30;
    

    /* clear i2c status; set enable, master, transmission mode, start condition and stop conditions) */
    xky_printf("Clear status\n");
    i2c_clear_status(dev_addr, I2C_STATUS_ALL);
    HWREG(dev_addr + I2C_CON) = \
       (I2C_CON_I2C_EN | I2C_CON_MST | I2C_CON_TRX | I2C_CON_STT | I2C_CON_STP);//enable 1 bits

    /* send full message loop */
    //xky_printf("Looping\n");
    int sent = 0;
    while (sent < nbyte) {

        /* poll status */
        //xky_printf("polling\n");
        unsigned int status = i2c_status_poll(dev_addr, I2C_STATUS_XRDY | I2C_STATUS_ERROR);

        
        /* check if there's error in bus */
        /* could be interesting to just check the NACK bit*/
        //xky_printf("check error in bus\n");
        if ((status & I2C_STATUS_ERROR) != 0) {
            xky_printf("Valor no status = %x\n",status);
            xky_printf("error in bus\n");
            i2c_soft_reset(dev_addr);
            return -1;
        }

        /* check if bus is ready to write */
       // xky_printf("check bus is ready to write\n");
        if ((status & I2C_STATUS_XRDY) != 0) {
            xky_printf("bus ready to write\n");

            i2c_write_byte(dev_addr, data[sent]);

            xky_printf(" Value of data = %08x\n",data[sent]);

            //unsigned int data_sent = HWREG(dev_addr + I2C_DATA);

            //HWREG(dev_addr + I2C_DATA) = 0;
            
            //xky_printf(" Register %08x\n",data_sent);
            //i2c_write_byte(((dev_addr<<1) & 0xFE), 0x05);
            sent++;
            
            xky_printf("clear status\n");
            i2c_clear_status(dev_addr, I2C_STATUS_XRDY);

            int bufstat = HWREG(dev_addr + I2C_BUFSTAT);
             xky_printf("Bufstat = %x\n", bufstat);

            int bytes_left =  HWREG(dev_addr + I2C_CNT);
                xky_printf("Poll do registo DCOUNT = %d\n",bytes_left);
        
            continue;
        }


        /* reset device */
        /* ideally this will happen and only happen if there's no errors and the message is not sent fully */
        xky_printf("reseting bus\n");
        i2c_soft_reset(dev_addr);
        return -1;
    }

    

    /*check if the operation is complete */
    
    if (i2c_is_device_ready(dev_addr) == 0) {
         xky_printf("device_not_ready\n");
         i2c_soft_reset(dev_addr);
         return -1;
     }

    /* clear slave address and finish transmission */
    i2c_clear_status(dev_addr, I2C_STATUS_ALL);
    return sent;
}

int i2c_master_receive(
        unsigned int dev_id, unsigned int slave_addr, char *data, int nbyte) {

    /* get base device address */
    unsigned int dev_addr = get_base_addr(dev_id);

    /* wait for the bus to be free */
    if (i2c_is_bus_free(dev_addr) == 0) {
        return -1;
    }

    /* set the slave address, set data count */
    HWREG(dev_addr + I2C_CNT) = (unsigned int)nbyte;
    HWREG(dev_addr + I2C_SA) = slave_addr;

    /* set master and reception mode (start and stop conditions)*/
    HWREG(dev_addr + I2C_CON) = \
        (I2C_CON_I2C_EN | I2C_CON_MST | I2C_CON_STT | I2C_CON_STP);

    int received = 0;
    while (received < nbyte) {

        /* poll status */
        unsigned int status = i2c_status_poll(
                dev_addr, I2C_STATUS_RRDY | I2C_STATUS_ERROR);

        /* ready to read */
        if ((status & I2C_STATUS_RRDY) != 0) {
            data[received++] = i2c_read_byte(dev_addr);
            i2c_clear_status(dev_addr, I2C_STATUS_RRDY);
            continue;
        }

        /* reset device */
        i2c_soft_reset(dev_addr);
        return -1;
    }

    /* wait for the operation to complete */
    if (i2c_is_device_ready(dev_addr) == 0) {
        i2c_soft_reset(dev_addr);
        return -1;
    }

    /* clear slave address and finish transmission */
    i2c_clear_status(dev_addr, I2C_STATUS_ALL);
    return received;
}



i2c_slave_action_e i2c_slave_get_action(unsigned int dev_id) {

    /* get base device address */
    unsigned int dev_addr = get_base_addr(dev_id);

    /* get status action */
    unsigned int status = i2c_get_status(dev_addr);

    /* check if a read was requested */
    if ((status & I2C_STATUS_RRDY) != 0) {
        return I2C_SLAVE_ACTION_READ;
    }

    /* check if a write was requested */
    if ((status & I2C_STATUS_XRDY) != 0) {
        return I2C_SLAVE_ACTION_WRITE;
    }

    /* no action necessary */
    return I2C_SLAVE_ACTION_NONE;
}

int i2c_slave_read(unsigned int dev_id, char *data, int nbyte) {

    /* get base device address */
    unsigned int dev_addr = get_base_addr(dev_id);

    int received = 0;
    while (received < nbyte) {

        /* poll status */
        unsigned int status = i2c_status_poll(
                dev_addr, I2C_STATUS_RRDY | I2C_STATUS_ARDY);

        /* operation completed */
        if ((status & I2C_STATUS_ARDY) != 0) {
            break;
        }

        /* ready to read */
        if ((status & I2C_STATUS_RRDY) != 0) {
            data[received++] = i2c_read_byte(dev_addr);
            i2c_clear_status(dev_addr, I2C_STATUS_RRDY);
            continue;
        }

        i2c_soft_reset(dev_addr);
        return -1;
    }

    /* clear slave address and finish reception */
    i2c_clear_status(dev_addr, I2C_STATUS_ALL);
    return received;
}

int i2c_slave_write(unsigned int dev_id, char *data, int nbyte) {

    /* get base device address */
    unsigned int dev_addr = get_base_addr(dev_id);

    /* send message */
    int sent = 0;
    while (sent < nbyte) {
        /* poll status */
        unsigned int status = i2c_status_poll(
                dev_addr, I2C_STATUS_XRDY | I2C_STATUS_ARDY);

        /* operation completed */
        if ((status & I2C_STATUS_ARDY) != 0) {
            break;
        }

        /* ready to write */
        if ((status & I2C_STATUS_XRDY) != 0) {
            i2c_write_byte(dev_addr, data[sent++]);
            
            i2c_clear_status(dev_addr, I2C_STATUS_XRDY);
            continue;
        }

        i2c_soft_reset(dev_addr);
        return -1;
    }

    /* clear slave address and finish transmission */
    i2c_clear_status(dev_addr, I2C_STATUS_ALL);
    return sent;
}
