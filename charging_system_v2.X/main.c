/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on December 27, 2023, 7:49 PM
 */


#include <xc.h>
#define F_CPU 8000000UL 
#include <util/delay.h>

#include "controller_io_utils.h"

void main(void) {
    // se configuran los puertos de entrada y salida
    configurePorts();

    while (1){
        nimhLedOn();
        lionLedOff();
        _delay_ms(1000);
        nimhLedOff();
        lionLedOn();
        _delay_ms(1000);
        //control de buck
        buckOff();
        _delay_ms(500);
        buckOn();
        _delay_ms(500);
        
        //control de carga
        chargeLionOn();
        chargeNimhOff();
        _delay_ms(1000);
        chargeLionOff();
        chargeNimhOn();
        _delay_ms(1000);
        
        //control de alimentacion
        supplyLionOn();
        supplyNimhOff();
        _delay_ms(1000);
        supplyLionOff();
        supplyNimhOn();
        _delay_ms(1000);
        

    }
    

    return;
}
