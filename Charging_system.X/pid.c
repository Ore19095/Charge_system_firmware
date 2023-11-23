#include "pid.h"


float error = 0;
static float error_prev = 0;
static float error_integral = 0;
float out = 0;

void reset_pid(){
    error = 0;
    error_prev = 0;
    error_integral = 0;
    return;
}

float pid_voltage(uint16_t input,uint16_t reference){
    error_prev = error; //actualizar error previo
    error = reference - input; //calcular error
    error_integral += error/PID_Fs; //calcular integral

    out = (PID_P*error + PID_I*error_integral +
            PID_D*(error - error_prev)*PID_Fs); //calcular salida

    if(out > 1023) out = 1023;
    else if(out < 0) out = 0;

    return out;
}

float pid_current(uint16_t input,uint16_t reference){
   // error_prev = error; //actualizar error previo
    error = reference - input; //calcular error
    error_integral += error/PID_Fs; //calcular integral

    out = (PID_P_CUR*error + PID_I_CUR*error_integral +
            PID_D_CUR*(error - error_prev)*PID_Fs); //calcular salida
    
    if(out > 1023) out = 1023;
    else if(out < 0) out = 0;

    return out;
}
