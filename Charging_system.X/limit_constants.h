# ifndef LIMIT_CONSTANTS_H
# define LIMIT_CONSTANTS_H

#define V_IN_MIN 359 // detecta cuando hay 10V en la alimentacion 10*1024/5 = 359.3

#define V_LECT_MAX_LION 860 // 4.2V
#define V_LECT_MIN_LION 819 // 3V 3*1024/5 = 614.4

#define V_LECT_MAX_NIMH  922// V_NiMH = 6V, 6*(3/4)(1024/5) = 921.6
#define V_LECT_MIN_NIMH  614// V_NiMH = 4V, 4*(3/4)(1024/5) = 614.4

#define I_LION 614 // I = 1A, 1*3*(1024/5) = 614.4 //corriente maxima de carga LiON
#define I_NIMH 184 // I = 0.3A, 0.3*3*(1024/5) = 184.3 //corriente maxima de carga NiMH

#define I_LION_MIN 92 // I = 150mA, 0.15*3*(1024/5) = 92.16 //corriente minima de carga LiON

#endif