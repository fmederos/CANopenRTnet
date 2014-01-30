#ifndef drive_canopen_H_
#define drive_canopen_H_
#include <xs1.h>

#define _DEBUG_

#define N_PUERTOS_PWM   4
#define RES_PWM         1024                    // resolución de los puertos PWM
#define GRANO_PWM       5                     // granularidad en veintenas de nS del incremento de
                                                  // ancho de pulso de la señal PWM
                                                  // grano=0 -> incremento=10nS, grano=1 -> incremento 20nS, 2->40nS, 3->60nS, etc.
                                                // 4->80ns, 5->100ns

// módulo Quadrature-Encoder-Input configura su cantidad de encoders leídos con este define que
// en rigor debería llamarse "number_of_encoders"
#define NUMBER_OF_MOTORS 2

// En este modo (SD-Card) se puede leer el registro de STATUS del módulo WL directamente
// con una lectura del puerto SPI
#define SPI_MASTER_SD_CARD_COMPAT 1
// Selección de canal
#define NRF_CHANNEL     5
#define NRF_ADDRESS     0xB6,0x24,0xA6


#ifdef __XC__
// declaraciones de funciones

#endif
#endif /* sync_RTnet_H_ */
