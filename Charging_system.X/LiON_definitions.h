/* 
 * File:   
 * Author: 
 * Comments: Contantes relacionadas con la bateria LiON, estos valores son los
 * que deberá de mostrar el ADC.
 * Revision history: 
 */

#ifndef LION_DEFINITIONS_H
#define	LION_DEFINITIONS_H

#define V_LION_FULL 778 // Voltaje de carga completa de la bateria LiON (4.2V)
#define V_LION_EMPTY 737// Voltaje de descarga completa de la bateria LiON (3.0V)

#define I_LION_CHARGE 819//1A*3*1024/5 Corriente de carga de la bateria LION (A)
#define I_LION_STOP_CHARGE 92//0.15A*3*1024/5 Corriente de carga de la bateria LION (A)

#endif	/* XC_HEADER_TEMPLATE_H */

