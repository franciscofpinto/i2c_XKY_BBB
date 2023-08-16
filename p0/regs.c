/*
 * regs.c
 *
 *  Created on: 28/02/2018
 *      Author: pesi
 */


//#define DEBUG

#ifdef DEBUG

#include <xky.h>
#include <xky_printf.h>

#endif

void put32(unsigned int address, unsigned int data) {
    unsigned int *ptr;
    ptr = (unsigned int*)address;

#ifdef DEBUG
    xky_printf("bw:%08x <= %08x  ver[%08x]\n",address , data,ptr[0]);
#endif


    ptr[0] = data;

}

unsigned int get32(unsigned int address) {
    unsigned int *ptr;
    ptr = (unsigned int*)address;


#ifdef DEBUG
    xky_printf("rd:%08x => %08x \n",address , ptr[0]);
#endif
    return ptr[0];
}
