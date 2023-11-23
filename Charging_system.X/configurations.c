#include "configurations.h"

void  conf_timer0(){
    /*
        Usado para generar una señal Cuadrada con una frecuencia de 
        62.5Khz, para el funcionamiento del charge pump que genera
    */
    // Temporizador en modo Fast PWM, TOP = OCRA WGM0[2:0] = 0b111
    TCCR0A |= (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << WGM02);
    // Prescalador de 64, CS0[2:0] = 0b010
    TCCR0B |= (1 << CS01) ;
    TCCR0B &= ~((1 << CS02) | (1 << CS00) );
    // Configuración de los pines OC0A como salida, COM0A[1:0] = 0b10
    TCCR0A |= (1 << COM0A0);
    TCCR0A &= ~(1 << COM0A1); // Toggle on compare match
    OCR0A = 16; // Fs = Fosc / (2*N * (1 + OCR0A)) ,N = 64 (prescalador)
    return;
}

void conf_timer1(){
    // Usado para generar una señal PWM de 10 bits, que controlará
    // la salida del convertidor buck
    // Temporizador en modo Fast PWM 10 bits WGM1[3:0] = 0b0111
    TCCR1A |= (1 << WGM11) | (1 << WGM10);
    TCCR1B &= ~(1 << WGM13);
    TCCR1B |= (1 << WGM12);
    // Prescalador de 1, CS1[2:0] = 0b001
    TCCR1B |= (1 << CS10);
    TCCR1B &= ~(1 << CS11 | 1 << CS12);
    // Configuración de los pines OC1A como salida, COM1A[1:0] = 0b10
    TCCR1A |= (1 << COM1A1);
    TCCR1A &= ~(1 << COM1A0);
    OCR1A = 1023; // duty cycle = 0%
    return;
}

void conf_timer2(){
    // Usado para muestrear el ADC a la frecuencia de 1kHz
    // Temporizador en modo CTC WGM2[2:0] = 0b010
    TCCR2A |= (1 << WGM21);
    TCCR2A &= ~(1 << WGM20);
    TCCR2B &= ~(1 << WGM22);
    // Prescalador de 64 CS2[2:0] = 0b011
    TCCR2B |= (1 << CS21) | (1 << CS20);
    TCCR2B &= ~(1 << CS22);
    // Se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);
    // Frecuencia de interrupcion = 1kHz
    OCR2A = 30;  // Fs = Fosc / (N * (1 + OCR2A)) ,N = 64 (prescalador)
    return;
}

void conf_adc(){
    // Referencia de voltaje en AVCC con capacitor en AREF
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    ADMUX |= ((1 << MUX0));
    // activar la interrupcion por ADC
    ADCSRA |= (1 << ADIE);
    // prescalador de 32 para el reloj del modulo
    ADCSRA |= (1 << ADPS2) | (1 << ADPS0);
    ADCSRA &= ~((1 << ADPS1));
    // Habilitar el ADC
    ADCSRA |= (1 << ADEN);
    //habilidar interrupcion por ADC
    ADCSRA |= (1 << ADIE);
    return;
}

void conf_uart(){
     UCSR0A |= 1<<U2X0; //UART en modo de alta velocidad
    
    UCSR0B |= 1<<RXEN0; // Habilitar receptor UART
    UCSR0B |= 1<<TXEN0; // Habilitar transmisor UART
   
    UBRR0 = 103; //1M baud rate con Fosc = 8MHz
}

void conf_ports(){
    // Deshabilita la funcion digital de los pines ADC0-ADC3
    DIDR0 = 0b00001111; 
    /*  PD0: RXD (Entrada)  PD4: PWR_LiON (Salida)
        PD1: TXD (Salida)   PD5: ON_Buck (Salida)
        PD2: N.C. (Entrada) PD6: PWM_PUMP (Salida)
        PD3: N.C. (Entrada) PD7: MUX_LiON (Salida)*/
    DDRD = 0b11110010;
    /*  PB0: MUX_NIMH (Salida)   PB4: MISO_PROG (Entrada)
        PB1: PWM_DAC (Salida)    PB5: SCK_PROG (Entrada)
        PB2: PWR_NIMH (Salida)   PB6: LED_NiMH (Salida)
        PB3: MOSI_PROG (Entrada) PB7: LED_LiON (Entrada)*/
    DDRB = 0b11000111;
    /*  PC0: ADC0 (Entrada) PC4: SDA (Entrada)
        PC1: ADC1 (Entrada) PC5: SCL (Entrada)
        PC2: ADC2 (Entrada) PC6: RESET (Entrada)
        PC3: ADC3 (Entrada) PC7: XTAL2 (Entrada)*/
    DDRC = 0b00000000;
}
