#ifndef PID_H
#define PID_H

#include <stdint.h>

#define PID_Fs 1000 // frecuencia de muestreo del PID

/*constantes para pid voltaje*/
#define PID_D -2.244e-4
#define PID_I -500.5
#define PID_P -1.2515

//PID for current control
#define PID_D_CUR 0//-4.244e-4/5
#define PID_I_CUR 0//-400.5/10
#define PID_P_CUR -1.2515/10

void reset_pid();
float pid_current(uint16_t input,uint16_t reference);
float pid_voltage(uint16_t input,uint16_t reference);

#endif // PID_CONSTANTS_H