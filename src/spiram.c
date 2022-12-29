#include "spiram.h"

static SPI_HandleTypeDef hspi2;

void RAM_SPI_Init( void )
{
	hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  //RAM_SPI_IoInit();
}

void RAM_SPI_DeInit( void )
{
  HAL_SPI_DeInit( &hspi2);
  __HAL_RCC_SPI2_FORCE_RESET();  
  __HAL_RCC_SPI2_RELEASE_RESET();
  RAM_SPI_IoDeInit( );
}

void RAM_SPI_IoInit( void )
{
  GPIO_InitTypeDef initStruct={0};
  initStruct.Mode =GPIO_MODE_AF_PP;
  initStruct.Pull = GPIO_PULLDOWN;
  initStruct.Speed = GPIO_SPEED_HIGH;
  initStruct.Alternate= SPI1_AF ;

  HW_GPIO_Init( RAM_SCLK_PORT, RAM_SCLK_PIN, &initStruct);
  HW_GPIO_Init( RAM_MISO_PORT, RAM_MISO_PIN, &initStruct);
  HW_GPIO_Init( RAM_MOSI_PORT, RAM_MOSI_PIN, &initStruct);

  initStruct.Mode = GPIO_MODE_OUTPUT_PP;
  initStruct.Pull = GPIO_PULLUP;

  HW_GPIO_Init(  RAM_NSS_PORT, RAM_NSS_PIN, &initStruct );

  HW_GPIO_Write ( RAM_NSS_PORT, RAM_NSS_PIN, 1 );
	PRINTF_LN("ioinint");
}



void RAM_SPI_IoDeInit( void )
{
  GPIO_InitTypeDef initStruct={0};

  initStruct.Mode =GPIO_MODE_OUTPUT_PP;

  initStruct.Pull =GPIO_PULLDOWN  ;
  HW_GPIO_Init ( RAM_MOSI_PORT, RAM_MOSI_PIN, &initStruct );
  HW_GPIO_Write( RAM_MOSI_PORT, RAM_MOSI_PIN, 0 );

  initStruct.Pull =GPIO_PULLDOWN; 
  HW_GPIO_Init ( RAM_MISO_PORT, RAM_MISO_PIN, &initStruct );
  HW_GPIO_Write( RAM_MISO_PORT, RAM_MISO_PIN, 0 );

  initStruct.Pull =GPIO_PULLDOWN  ; 
  HW_GPIO_Init ( RAM_SCLK_PORT, RAM_SCLK_PIN, &initStruct );
  HW_GPIO_Write(  RAM_SCLK_PORT, RAM_SCLK_PIN, 0 );

  initStruct.Pull = GPIO_PULLUP;
  HW_GPIO_Init ( RAM_NSS_PORT, RAM_NSS_PIN , &initStruct );
  HW_GPIO_Write( RAM_NSS_PORT, RAM_NSS_PIN , 1 );
}

void startOp( void ) {
	HW_GPIO_Write( RAM_NSS_PORT, RAM_NSS_PIN , 0 );
}

void finishOp( void ) {
	HW_GPIO_Write( RAM_NSS_PORT, RAM_NSS_PIN , 1 );
}

/*
 * Private method: sends a byte command followed by the three bytes (24 bits) of an address.
 * Please notice this method will not work with smalled devices which use 16 bit memory addresses.
 */
uint8_t sendCommand(Op cmd, uint32_t addr) {
	
	// Extract the three address bytes
	uint8_t a[4];
	a[0] =(uint8_t) cmd;
	a[1] = (addr & 0x00ff0000) >> 16;
	a[2] = (addr & 0x0000ff00) >> 8;
	a[3] = (addr & 0x000000ff);	
	return (uint8_t)HAL_SPI_Transmit(&hspi2,a,4,5000);
}

/*
 * Private method: send an array of bytes
 */
uint8_t sendBuffer(uint16_t size, uint8_t *buffer) {
	return (uint8_t)HAL_SPI_Transmit(&hspi2,buffer,size,5000);
}

/*
 * Private method: read an array of bytes
 */
uint8_t readBuffer(uint16_t size, uint8_t *buffer) {
	return (uint8_t)HAL_SPI_Receive(&hspi2, buffer, size, 5000);
}

/*
 * Set up the device and the CS pin.
 * It also sets the initial mode
 */
void begin(Mode mode) {

	RAM_SPI_Init();
	// Set up the initial mode
	startOp();
	uint8_t a[2];
	a[0] =(uint8_t) WRMR;
	a[1] = (uint8_t)mode;
	HAL_SPI_Transmit(&hspi2,a,2,5000);
	finishOp();
}

// Set the access mode. It's the same as begin() without the SPI initialization
// TO-DO: Refactor this method and begin()
void setMode(Mode mode) {
	// Set up the initial mode
	startOp();
	uint8_t a[2];
	a[0] =(uint8_t) WRMR;
	a[1] = (uint8_t)mode;
	HAL_SPI_Transmit(&hspi2,a,2,5000);
	finishOp();
}

/*
 * Write an array of bytes into the device
 */
void write(uint32_t address, uint16_t size, uint8_t *buffer) {
	startOp();
	sendCommand(WRITE,address);
	sendBuffer(size, buffer);
	finishOp();
}

/*
 * Read an array of bytes from the device
 * See the write() comments for details, since both methods are basically equal
 */
void read(uint32_t address, uint16_t size, uint8_t *buffer) {
	startOp();
	sendCommand(READ,address);
	readBuffer(size, buffer);
	finishOp();
}
