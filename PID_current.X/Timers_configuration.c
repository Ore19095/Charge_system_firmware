/* 
 * File: Timer_configuration.c
 * Author: Angel Orellana
 * Comments: Funciones para la configuración de timers en ATMEGA328P
 * Revision history: 
 */

#include "Timers_configuration.h"

/**
 * Configuraci�n del timer 2, en modo CTC. Se activa la 
 * interrupci�n por <i>overflow</i> en el contador,  la
 * cual tiene una frecuencia de 1kHz. 
 **/
void configure_timer2(){
    
    TCCR2A |= (1 << WGM21); // WGM2[1:0] = 0b10
    TCCR2A &= ~(1 << WGM20);
    TCCR2B &= ~(1 << WGM22); // WGM2[2] = 0b0 modo CTC
    
    // Prescalador de 64
    TCCR2B |= (1 << CS22);
    TCCR2B &= ~(1 << CS21 | 1 << CS20);
    // se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);
    TIMSK2 |= (1 << OCIE2A);

    // Frecuencia de interrupcion = 1kHz
    OCR2A = 124;  // Fs = Fosc / (N * (1 + OCR2A)) ,N = 1024 (prescalador)
    
    return;
}


/** 
 * modulo de timer 1, sin preescalador, generará una señal pwm
 * variable, con una resolución de 10 bits, la señal pwm se genera
 * en el pin OC1A, el pin OC1B estará en modo normal (no pwm), se
 * configura el modulo como fast pwm de 10 bits 
 **/
void configure_pwm_timer1(){
    // COM1A[1:0] = 0b10, COM1B[1:0] = 0b00, WGM1[3:0] = 0b1110
    // COM1A[1:0] = 0b10, COM1B[1:0] = 0b00, WGM1[3:0] = 0b1110
    TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
    TCCR1A &= ~(1 << COM1A0);
    // WGM1[3:2] = 0b01, CS1[2:0] = 0b001
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    OCR1A = 0; // duty cycle = 0%
    return;
}

 /**
  * modulo timer 0 con un preescalador de 64, generando un pwm de
  * 50%, unicamente el pin OC0A sera como salida, el pin OC0B
  * estará en modo normal (no pwm). Se configuró el modulo como
  * fast pwm, siendo OC0A puesto en 1 al llegar al valor bottom, y
  * puesto en 0 cuando coincida con el valor del registro OCR0A. El valor
  * TOP para este modulo se definirá  */
void configure_pwm_timer0(){
    // WGM0[1:0] = 0b11 fast pwm, TOP = OCRA
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
        // WGM02 = 0b1 fast pwm, TOP = OCRA
    TCCR0B |= (1 << WGM02);
    // COM0A[1:0] = 0b10, COM0B[1:0] = 0b00
    TCCR0A |= (1 << COM0A0); // OC0A en modo pwm, toggle on compare match
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0B1 | 1 << COM0B0); // OC0B en modo normal
    // Prescalador de 64
    TCCR0B |= (1 << CS01) | (1 << CS00);
    TCCR0B &= ~(1 << CS02);
    // OCR0A = 7, duty cycle = 50%, Fs = 125 kHz
    OCR0A = 0;
}