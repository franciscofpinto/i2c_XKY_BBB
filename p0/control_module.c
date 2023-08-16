//
//  control_module.c
//
//  Created by Alexis Marquet on 03/12/14.
//
//
//  adapted by : pesi

#include <GPIO.h>
#include <clock_module.h>
#include <pad.h>
#include <control_module.h>
#include <regs.h>

unsigned long Base_CtrlAddress;

void CM_iniCtrlAddress(unsigned long CtrlAddress) {

    Base_CtrlAddress = CtrlAddress;

}

void CM_setCtrlModule(CONTROL_MODULE module, unsigned int value) {

    put32((module + Base_CtrlAddress) , value);

}

unsigned int CM_getCtrlModule(CONTROL_MODULE module) {

   return get32(module + Base_CtrlAddress);

}
