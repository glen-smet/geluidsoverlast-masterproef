#ifndef __NBIOT
#define __NBIOT

#include <stdint.h>
#include <stdbool.h>
#include "bg96.h"
#include "vcom2.h"
#include "string.h"
#include "timeServer.h"

#include "energy.h"

extern BG96_Powerdown_t powerStatus;

										
// ---------------------------- NBIOT FUNCTIONS ---------------------------------
void initNBIoT(void);														// Init NB-IoT modem
void registerNBIoT(void);												// Register to NB-IoT network
void sendNBIoTc(void);
void sendNBIoTs(char * payload);
void sendNBIoTi(void);
int8_t sendNBIoT(char * payload);// Send NB-IoT data
#ifdef __cplusplus
}
#endif

#endif /* __NBIOT */
