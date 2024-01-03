#include "adc_utils.h"

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
    // temporizador en modo CTC
    TCCR2A |= (1 << WGM21);
    TCCR2A &= ~(1 << WGM20);
    TCCR2B &= ~(1 << WGM22);
    // se configura el prescaler (beta, harcode si no funciona mañana)
    for(int i = 0; i < N_PRESCALERS; i++){
        ocraValue = F_OSC/(prescalers[i]*F_SAMPLE*N_CHANNELS) - 1;
        // si el valor calculado es mayor a 255, se prueba con el siguiente prescaler
        if(ocraValue>255) continue;
        else break; // si el valor es menor o igual a 255, se sale del ciclo
    }
    // se actualiza el registro OCR2A con el valor calculado
    OCR2A = (uint8_t) ocraValue;
    // Se habilita la interrupcion por comparacion con OCR2A
    TIMSK2 |= (1 << OCIE2A);
    return;
}

void configureADC(){
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
    // variable local que no pierde su valor entre llamadas
    static uint8_t counter = 0;
    // se guarda el valor d-el ADC en el buffer
    adcValues[ADMUX & 0x07] = ADC;
    // bits que van en 1 en el registro ADMUX
    ADMUX |= (0x07 & usableChannels[counter]);
    //bits que vna en 0 en el registo ADMUX
    ADMUX &= ~(0x07 & usableChannels[counter]);
    // se incrementa el contador
    counter = (counter+1)%N_CHANNELS;
    return;
}

/* Interrupcion del Timer 2*/
ISR(TIMER2_COMPA_vect){
    // se inicia la conversion del ADC
    ADCSRA |= (1 << ADSC);
    // se ejecuta la funcion adicional
    additionalProcessing();
    return;
}

void emptyFunction(){
    return;
}