//
//  pad.c
//
//  Created by Alexis Marquet on 03/12/14.
//
//
//  adapted by : pesi


#include <GPIO.h>
#include <clock_module.h>
#include <pad.h>
#include <control_module.h>

void PAD_setMode(CONTROL_MODULE module, pinmode_t mode)
{

   if((module <= CM_conf_usb1_drvvbus ) && (module >= CM_conf_gpmc_ad0))
   {
      unsigned int temp = CM_getCtrlModule(module);
      temp &= ~(0b111);
      temp |= mode;
      CM_setCtrlModule(module, temp);
   }
   else
   {
      // TODO: raise error (control module isnt a "conf <module> <pin>" register)
      return;
   }
}
