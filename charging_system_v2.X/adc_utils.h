/* 
 * File:   adc_utils.h
 * Author: Angel Orellana
 * Comments: API que facilita el uso del ADC, Habilita la interrupcion por
 * ADC. Para la lectura del ADC unicamente es necesario llamar a la funcion
 * readADC(channel) donde channel es el canal a leer, del buffer de lectura.
 * El buffer se actualiza con una frecuencia indicada en F_SAMPLE. El timer 
 * empleado para generar la interrupcion es el Timer2. si se desea realizar
 * un procesamiento adicional durante la interrupcion del timer, se debe
 * de llamar a la funcion setAdicionalProcessing(funcion(void)) donde el
 * parametro es la funcion que se desea ejecutar durante la interrupcion.
 * 
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef ADC_UTILS_H
#define	ADC_UTILS_H

#include <xc.h>
#include <stdint.h>
#include <avr/interrupt.h> // interrupciones para el timer


// -------- PARAMETROS MODIFICABLES POR USUARIO --------------------
#define F_OSC 8000000UL // frecuencia de oscilacion del microcontrolador
#define F_SAMPLE 1000 // frecuencia de muestreo del adc (para cada canal)
#define N_CHANNELS 5 // numero de canales a muestrear
#define N_CHANNELS_ADC 8 // numero de canales a muestrear
// valores posibles de prescaladores para timer2
#define N_PRESCALERS 7


void configureTimer2(void);
void configureADC(void);
void setAdicionalProcessing(void (*funcion)(void)) ;
uint16_t readADC(uint8_t channel);


#endif	/* XC_HEADER_TEMPLATE_H */

