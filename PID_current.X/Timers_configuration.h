/* 
 * File: Timer_configuration.h
 * Author: Angel Orellana
 * Comments: Funciones para la configuración de timers en ATMEGA328P
 * Revision history: 
 */

#ifndef TIMERS_CONFIG_E_H

#define	TIMERS_CONFIG_E_H
#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdint.h>

void configure_timer2();
void configure_pwm_timer1();
void configure_pwm_timer0();

#endif	/* XC_HEADER_TEMPLATE_H */

