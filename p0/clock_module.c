//
//  clock_module.c
//
//  Created by Alexis Marquet on 03/12/14.
//
//  adapted by : pesi

#include <GPIO.h>
#include <clock_module.h>
#include <pad.h>
#include <control_module.h>
#include <regs.h>

unsigned long Base_ClkAddress;

void CKM_iniCtrlAddress(unsigned long ClkAddress) {
    Base_ClkAddress = ClkAddress;
}

bool CKM_checkValidModule(CKM_MODULE_REG module) {

    if ((module > CKM_PER_CLK_24MHZ_CLKSTCTRL)) {
        // TODO: raise error (CKM_MODULE_REG too big: /module)
        return false;
    }
    return true;
}
void CKM_setCLKModuleRegister(CKM_MODULE_REG module, unsigned int value) {

    if (CKM_checkValidModule(module)) {
        unsigned int addr_temp = Base_ClkAddress + module; // clock module base + module offset, TRM 2.1 & 8.1.12.1
        put32(addr_temp, value);
    }

}
unsigned int CKM_getCLKModuleRegister(CKM_MODULE_REG module) {

    if (CKM_checkValidModule(module)) {
        unsigned int addr_temp = Base_ClkAddress + module; // clock module base + module offset, TRM 2.1 & 8.1.12.1
        return get32(addr_temp);
    }
    return 0;
}
