/*
 * File:   main.c
 * Author: AngelOrellana
 *
 * Created on September 26, 2023, 9:53 AM
 */
#define F_CPU 8000000UL

#include <xc.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/io.h>

//------------------- constantes --------------------
#define SIGNAL_FREQ 10UL        /*frecuencia de la señal generada en el DAC*/
#define SINE                    // senoidal, si no es triangular
#define TIMER2_PREESCALER 256UL /*Valor del prescalador para el timer 2*/

#ifdef SINE
    #define DATA_FREQ 32UL /*multiplicador de la frecuencia   \
                       con la que se actualiza el DAC     \
                       con respecto a la frecuencia de la \
                        señal generada en el DAC*/
#else
    #define DATA_FREQ 1024UL
#endif

#define OCR2A_VALUE F_CPU / (DATA_FREQ * TIMER2_PREESCALER * SIGNAL_FREQ) - 1
//------------------- sine values --------------------
uint16_t sine[32] = {
    512, 615, 714, 804, 883, 947, 992, 1018,
    1023, 1008, 972, 917, 846, 760, 665, 564,
    460, 359, 264, 178, 107, 52, 16, 1,
    6, 32, 77, 141, 220, 310, 409, 512};
uint8_t dir = 0; // direccion de la señal triangular
uint16_t counter = 0;
int main(void){
    //--------- CONFIGURACION DE PUERTOS --------
    DDRB = 0b11111111; // PB0-7 como salidas
    DDRD = 0b11110010; // PD4-7,PD1 como salidas, PD2-3,PD0 como entrada
    // Colocar PD4 como salida
    PORTD &= ~(1 << PD4); // type:ignore
    //---------   CONFIGURACION PWM 0    --------
    /*modulo timer 0 con un preescalador de 64, generando un pwm de
    50%, unicamente el pin OC0A sera como salida, el pin OC0B
    estará en modo normal (no pwm). Se configuró el modulo como
    fast pwm, siendo OC0A puesto en 1 al llegar al valor bottom, y
    puesto en 0 cuando coincida con el valor del registro OCR0A. El valor
    TOP para este modulo se definirá  */
    // WGM0[1:0] = 0b11 fast pwm, TOP = OCRA
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    // COM0A[1:0] = 0b10, COM0B[1:0] = 0b00
    TCCR0A |= (1 << COM0A0); // OC0A en modo pwm, toggle on compare match
    TCCR0A &= ~(1 << COM0A1);
    TCCR0A &= ~(1 << COM0B1 | 1 << COM0B0); // OC0B en modo normal
    // CS0[2:0] = 0b011, preescalador = 64
    TCCR0B |= (1 << CS01) | (1 << CS00);
    TCCR0B &= ~(1 << CS02);
    // WGM02 = 0b1 fast pwm, TOP = OCRA
    TCCR0B |= (1 << WGM02);
    // OCR0A = 7, duty cycle = 50%, Fpwm = 7812.5 Hz
    OCR0A = 1;
    //-------- CONFIGURACION PWM 1 ---------
    /* modulo de timer 1, sin preescalador, generará una señal pwm
        variable, con una resolución de 10 bits, la señal pwm se genera
        en el pin OC1A, el pin OC1B estará en modo normal (no pwm), se
        configuró el modulo como fast pwm de 10 bits */
    // COM1A[1:0] = 0b10, COM1B[1:0] = 0b00, WGM1[3:0] = 0b1110
    TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
    TCCR1A &= ~(1 << COM1A0);
    // WGM1[3:2] = 0b01, CS1[2:0] = 0b001
    TCCR1B |= (1 << WGM12) | (1 << CS10);
    OCR1A = 1023; // duty cycle = 50%
    //-------- CONFIGURACION TIMER 2 ---------
    /*Se configura el modulo en modo de operacion CTC, con un
      preescalador de 64 para el reloj del timer, este
      temporizador se utiliza para el control de la
      frecuencia de la senal generada con el DAC
      */
    TCCR2A |= (1 << WGM21); // WGM2[1:0] = 0b10
    TCCR2A &= ~(1 << WGM20);
#if TIMER2_PREESCALER == 1024
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // CS2[2:0]=0b111,preescaler=1024
#elif TIMER2_PREESCALER == 256
    TCCR2B |= (1 << CS22) | (1 << CS21); // CS2[2:0] = 0b110, preescaler = 256
#elif TIMER2_PREESCALER == 128
    TCCR2B |= (1 << CS22) | (1 << CS20); // CS2[2:0] = 0b101, preescaler = 128
#elif TIMER2_PREESCALER == 64
    TCCR2B |= (1 << CS22); // CS2[2:0] = 0b100, preescaler = 64
#elif TIMER2_PREESCALER == 32
    TCCR2B |= (1 << CS21) | (1 << CS20); // CS2[2:0] = 0b011, preescaler = 32
#elif TIMER2_PREESCALER == 8
    TCCR2B |= (1 << CS21); // CS2[2:0] = 0b010, preescaler = 8
#elif TIMER2_PREESCALER == 1
    TCCR2B |= (1 << CS20); // CS2[2:0] = 0b001, preescaler = 1
#endif

    TCCR2B &= ~(1 << WGM22); // WGM2[2] = 0b0 modo CTC

    // se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);

    // frecuencia del timer
    OCR2A = (uint8_t)OCR2A_VALUE;

    sei(); // se habilitan las interrupciones globales

    while (1)
    {
        PORTB = 1;
    }
    return 0;
}
// ----------------- interrupciones -----------------
ISR(TIMER2_COMPA_vect){
    // se actualiza el valor del DAC
#ifdef SINE
    OCR1A = sine[counter % 32];
    counter++;
#else
    OCR1A = counter;
    if (counter == 1023) dir = 1;
    if (counter == 0) dir = 0;

    if (dir) counter--;
    else counter++;
#endif
}