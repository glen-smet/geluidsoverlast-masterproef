#ifndef SPIRAM_H_
#define SPIRAM_H_
//#include <stdint.h>
//#include <stdbool.h>
//#include <stdarg.h>
//#include <stddef.h>
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_gpio.h"
#include "utils_common.h"
#include "stm32l0xx_hal_spi.h"
//#include "hw.h"


#ifdef __cplusplus
extern "C" {
#endif

#define RAM_NSS_PORT  													GPIOB
#define RAM_NSS_PIN  														GPIO_PIN_7
#define RAM_MOSI_PORT                           GPIOB
#define RAM_MOSI_PIN                            GPIO_PIN_15
#define RAM_MISO_PORT                           GPIOB
#define RAM_MISO_PIN                            GPIO_PIN_14
#define RAM_SCLK_PORT                           GPIOB
#define RAM_SCLK_PIN                            GPIO_PIN_13

enum Error { OK=0, BADSIZ, BADPAG, BADADDR, BADOPT, ERR };
typedef enum{
	BYTE = 0x00,	//0b00000000,
	PAGE = 0x80,	//0b10000000,
	SEQ  = 0x40,	//0b01000000,
	NONE = 0xC0,	//0b11000000
}Mode;
typedef enum{
	READ  = 0x03,	//0b00000011,
	WRITE = 0x02, //0b00000010,
	EDIO  = 0x3B, //0b00111011,
	RSTIO = 0xFF,	//0b11111111,
	RMDR  = 0x05,	//0b00000101,
	WRMR  = 0x01,	//0b00000001
}Op;
void RAM_SPI_Init( void );
void RAM_SPI_DeInit( void );
void RAM_SPI_IoInit( void );
void RAM_SPI_IoDeInit( void );
void startOp(void);
void finishOp(void);
void begin(Mode mode);
void setMode(Mode mode);

void write(uint32_t address, uint16_t size, uint8_t *buffer);
void read(uint32_t address, uint16_t size, uint8_t *buffer);
uint8_t sendCommand(Op cmd, uint32_t addr);
uint8_t sendBuffer(uint16_t size, uint8_t *buffer);
uint8_t readBuffer(uint16_t size, uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif /* SPISRAM_H_ */
