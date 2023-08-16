/**
 * @file GPIO.c
 * @author Alexis Marquet
 * @date 03 Dec 2014
 * @brief Implementation concerning GPIO usage: TRM 9.3.51, TRM 25, Datasheet 4
 **/
//  adapted by : pesi


#include <GPIO.h>
#include <clock_module.h>
#include <pad.h>
#include <control_module.h>
#include <regs.h>

unsigned long Base_GPIOAddress[4];

void GPIO_initAddress(unsigned long GPIO_0_BASE, unsigned long GPIO_1_BASE, unsigned long GPIO_2_BASE, unsigned long GPIO_3_BASE) {

    Base_GPIOAddress[0] = GPIO_0_BASE;
    Base_GPIOAddress[1] = GPIO_1_BASE;
    Base_GPIOAddress[2] = GPIO_2_BASE;
    Base_GPIOAddress[3] = GPIO_3_BASE;
}

void GPIO_initPort(GPIO_t port) {
    if (GPIO_checkValidPortPin(port, 0)) // pin 0 always exist
            {
        unsigned int setting = (1 << 18) | (0x2 << 0); //enable functional clock & enable module, TRM 8.1.12.1.32
        switch (port) {
            case GPIO0:
                return; // GPIO0 doesnt have a clock module register, TRM 8.1.12.1
                break;
            case GPIO1:
                CKM_setCLKModuleRegister(CKM_PER_GPIO1_CLKCTRL, setting);
                while ((CKM_getCLKModuleRegister(CKM_PER_GPIO1_CLKCTRL) & (0x3 << 16)) != 0)
                    ;
                break;
            case GPIO2:
                CKM_setCLKModuleRegister(CKM_PER_GPIO2_CLKCTRL, setting);
                while ((CKM_getCLKModuleRegister(CKM_PER_GPIO2_CLKCTRL) & (0x3 << 16)) != 0)
                    ;
                break;
            case 3:
                CKM_setCLKModuleRegister(CKM_PER_GPIO3_CLKCTRL, setting);
                while ((CKM_getCLKModuleRegister(CKM_PER_GPIO3_CLKCTRL) & (0x3 << 16)) != 0)
                    ;
                break;
            default:
                // TODO: raise error (not possible, checked port before: /port)
                break;
        }
    }
}

bool GPIO_checkValidPortPin(GPIO_t port, pin_t pin) {
    if ((port < GPIO0) || (port > GPIO3)) {
        // TODO: Only GPIO0 is working at the moment
        return false;
    }
    if ((pin < 0) || (pin > 31)) {
        return false;
    }
    if (GPIO_CTRL_MODULE_ARRAY[pin][port] == -1) {
        return false;
    }
    return true;
}

void GPIO_initPin(GPIO_t port, pin_t pin) {
    if (GPIO_checkValidPortPin(port, pin)) {
        CONTROL_MODULE module = GPIO_CTRL_MODULE_ARRAY[pin][port]; // get conf <module> <pin> for port/pin combination
        PAD_setMode(module, MODE_7); //set mode to GPIO, Datasheet 4.3.2
        return;
    }
}

void GPIO_setDirection(GPIO_t port, pin_t pin, pin_direction_t dir) {

    if (GPIO_checkValidPortPin(port, pin)) {
        if (GPIO_CheckValidDirection(dir) && Base_GPIOAddress[port] != 0) {
            unsigned int addr_temp = Base_GPIOAddress[port] + GPIO_OE; // GPIOx base + output enable offset, TRM 2.1 & 25.4.1.16
            unsigned int val_temp = get32(addr_temp); // not overwriting previous port setting
            val_temp &= ~(1 << pin);
            val_temp |= (dir << pin);
            put32(addr_temp, val_temp);
        }
    }
}

bool GPIO_CheckValidDirection(pin_direction_t dir) {
    if ((dir != INPUT) && (dir != OUTPUT)) {
        // TODO: raise error (direction not valid: /dir)
        return false;
    }
    return true;
}

void GPIO_setPin(GPIO_t port, pin_t pin) {

    if (GPIO_checkValidPortPin(port, pin) && Base_GPIOAddress[port] != 0) {
        unsigned int addr_temp = Base_GPIOAddress[port] + GPIO_SETDATAOUT; // GPIOx base + set data out offset, TRM 2.1 & 25.4.1.26
        unsigned int val_temp = 1 << pin;
        put32(addr_temp, val_temp);
    }
}

void GPIO_clrPin(GPIO_t port, pin_t pin) {

    if (GPIO_checkValidPortPin(port, pin) && Base_GPIOAddress[port] != 0) {
        unsigned int addr_temp = Base_GPIOAddress[port] + GPIO_CLEARDATAOUT; // GPIOx base+clear data out offset, TRM 2.1 & 25.4.1.25
        unsigned int val_temp = 1 << pin;
        put32(addr_temp, val_temp);
    }
}
