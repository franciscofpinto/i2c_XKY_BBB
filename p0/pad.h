//
//  pad.h
//
//  Created by Alexis Marquet on 03/12/14.
//
//
/**
 * @file pad.h
 * @author Alexis Marquet
 * @date 03 Dec 2014
 * @brief File containing types & function prototypes concerning pad usage: TRM 9.2.2.1, Datasheet 4
 **/
//  adapted by : pesi

#ifndef EXAMPLES_GPIO_P0_PAD_H_
#define EXAMPLES_GPIO_P0_PAD_H_

#include <control_module.h>

typedef enum
{
   MODE_0 = 0,
   MODE_1 = 1,
   MODE_2 = 2,
   MODE_3 = 3,
   MODE_4 = 4,
   MODE_5 = 5,
   MODE_6 = 6,
   MODE_7 = 7

}pinmode_t;

void PAD_setMode(CONTROL_MODULE module, pinmode_t mode);

#endif /* EXAMPLES_GPIO_P0_PAD_H_ */
