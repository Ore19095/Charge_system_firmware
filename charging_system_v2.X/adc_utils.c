#include "adc_utils.h"

uint8_t usableChannels[N_CHANNELS] = {1, 0, 2, 3, 7}; // canales a muestrear
uint16_t const prescalers[N_PRESCALERS] = {1, 8, 32, 64, 128, 256, 1024};
uint16_t adcValues[N_CHANNELS_ADC];
uint8_t counter = 0;

void emptyFunction(void);

void (*additionalProcessing)(void) = emptyFunction;
/**
 *  Se configura el Timer 2 para que genere una frecuencia de muestreo de 1kHz
 *  en cada canal del ADC. 
*/
void configureTimer2(){
    uint16_t ocraValue = 0;
    // se desconectan las salidas de los comparadores
    TCCR2A &= ~(1 << COM2A1);
    TCCR2A &= ~(1 << COM2A0);
    TCCR2A &= ~(1 << COM2B1);
    TCCR2A &= ~(1 << COM2B0);
    // Temporizador en modo CTC WGM2[2:0] = 0b010
    TCCR2A |= (1 << WGM21);
    TCCR2A &= ~(1 << WGM20);
    TCCR2B &= ~(1 << WGM22);
    // se configura el prescaler (beta, harcode si no funciona mañana)
    // for(int i = 0; i < N_PRESCALERS; i++){
    //     ocraValue = F_OSC/(prescalers[i]*F_SAMPLE*N_CHANNELS) - 1;
    //     // si el valor calculado es mayor a 255, se prueba con el siguiente prescaler
    //     if(ocraValue>255) continue;
    //     else break; // si el valor es menor o igual a 255, se sale del ciclo
    // }
    // Prescalador de 64 CS2[2:0] = 0b011
    TCCR2B |= (1 << CS21) | (1 << CS20);
    TCCR2B &= ~(1 << CS22);
    // se actualiza el registro OCR2A con el valor calculado
    OCR2A = (uint8_t) 123;
    // Se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);
    // se enciende el ADC
    return;
}

void configureADC(){

    // Deshabilita la funcion digital de los pines ADC0-ADC3
    DIDR0 = 0b10001111;
    // Referencia de voltaje en AVCC con capacitor en AREF
    ADMUX |= (1 << REFS0);
    ADMUX &= ~(1 << REFS1);
    // prescalador de 32 para el reloj del modulo
    ADCSRA |= (1 << ADPS2) | (1 << ADPS0);
    ADCSRA &= ~((1 << ADPS1));
    // Habilitar el ADC
    ADCSRA |= (1 << ADEN);
    //habilidar interrupcion por ADC
    ADCSRA |= (1 << ADIE);
    ADCSRA |= (1 << ADSC);
    return;
}

// Funcion que permite configurar una funcion adicional que se ejecutara
void setAdicionalProcessing(void (*funcion)(void)){
    additionalProcessing = funcion;
    return;
}

// Funcion que permite leer el valor del ADC de un canal especifico
uint16_t readADC(uint8_t channel){
    return adcValues[channel];
}

/* Interrupcion del ADC*/
ISR(ADC_vect){
    // se guarda el valor d-el ADC en el buffer
    adcValues[ usableChannels[counter] ] = ADC;
    // se incrementa el contador
    counter = (counter+1)%N_CHANNELS;
    // se configura el canal del ADC
    switch (usableChannels[counter]){
    case 0:
        ADMUX &= ~(1 << MUX0);
        ADMUX &= ~(1 << MUX1);
        ADMUX &= ~(1 << MUX2);
        break;
    case 1:
        ADMUX |= (1 << MUX0);
        ADMUX &= ~(1 << MUX1);
        ADMUX &= ~(1 << MUX2);
        break;
    case 2:
        ADMUX &= ~(1 << MUX0);
        ADMUX |= (1 << MUX1);
        ADMUX &= ~(1 << MUX2);
        break;
    case 3:
        ADMUX |= (1 << MUX0);
        ADMUX |= (1 << MUX1);
        ADMUX &= ~(1 << MUX2);
        break;
    case 7:
        ADMUX |= (1 << MUX0);
        ADMUX |= (1 << MUX1);
        ADMUX |= (1 << MUX2);
        break;
    default:
        break;
    }

    return;
}

/* Interrupcion del Timer 2*/
ISR(TIMER2_COMPA_vect){
    // se inicia la conversion del ADC
    ADCSRA |= (1 << ADSC);
    // se ejecuta la funcion adicional
    //additionalProcessing();
    return;
}

void emptyFunction(){
    return;
}