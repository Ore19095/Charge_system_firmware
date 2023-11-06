/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on July 12, 2023, 4:58 PM
 */

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>


#define ADC_START 0x40 //set the start bit



uint16_t current = 123;

int16_t pwm;

int main(void) {
    // ----     IO CONFIG  ---------
    DDRB = 0b00000011; // PB[7:2] input, PB1 out, PB0 out
    
    PORTB = 0b0000000; //activar PB0
    // ----  ADC CONFIG ------------
    
    ADMUX = 0b01000001; // [7:6] REFS, reference is AVCC (5V)
                        //  [5] ADLAR  right adjusted
                        // [3:0] ADC channel 1

    ADCSRB = 0b00000110; // free runung auto triggered
    DIDR0 = 0b00111111; // Digital input disabled
    ADCSRA = 0b10000110; /*
                        * [7] ADEN , Adc anabled
                        * [5] ADATE,auto trigger enabled
                        * [2:0] ADPS, ADC clk = sys clk/64
                        */

    // ---- PWM configuration ------
    TCCR1A = 0b10100011;  //[7:6] COM1A1 [1:0] Clear OC1A on match
                            //[5:4] COM1B1 [1:0] Clear OC1B on match
                            //[1:0] WGM1 [1:0] Fast PWM, 10-bit
    TCCR1B = 0b00001101;  /*[7:5] no usado
                             *[4:3] WGM1 [3:2] Fast PWM, 10-bit
                             *[2:0] CS1  [2:0] clk/1  */
    OCR1A = 1023; //pwm 100%
    OCR1B = 512;
    pwm = 256;
    _delay_ms(50);
    // start ADC conversion
    ADCSRA |= ADC_START;
    
    while (1) {
        if((ADCSRA & ADC_START) == 0 ){
            current = ADC ;
            PORTD = (current&255);
            ADCSRA |= ADC_START;
        }
        if(current<800) pwm -= 1;
        else if(current>840) pwm += 1;
        
        if (pwm < 0) pwm = 0;
        if (pwm >1023) pwm = 1023;
        //_delay_ms(5);
        OCR1A = (uint16_t) 256;
    }
}
