// -------------------------------- NBIOT FUNCTIONS -------------------------------------

#include "nbiot.h"

BG96_Powerdown_t powerStatus = BG96_POWERDOWN;

// Total payload size = Lookuptable + overhead 'P', deviceID, bootID, packageNumber and commas
const uint16_t nbiot_payloadsize_lt[] ={0,	
																				1,	
																				2,	
																				4,	
																				8,	
																				16,	
																				24,	
																				32,	
																				48,	
																				64,	
																				96,	
																				128,	
																				192,	
																				256,	
																				384,
																				512,
																				768,
																				1024,
																				1449 // Max payload for BG96=1460, minus max meta data=11
};


// Statics
static void onTimerWd(void *context);
void send_data_request_wd( void );
static void checkPowerDown(void);

static TimerEvent_t WdTimer;
	
void initNBIoT(void){
	#ifdef DEBUG
	PRINTF_LN("start NBiot init");
	#endif

	BG96_Init();
	PRINTF_LN("B96 init");
	BG96_PowerOn();
	
	powerStatus = BG96_ACTIVE; 
	PRINTF_LN("B96 ON");
	char buffer[30];
	memset(buffer, '\0', 30);
	
	// ---------- Configure NB-IoT ---------- 
	BG96_SendATCommandGetReply("ATX1\r\n", buffer, 300); // Important for edrx?
	BG96_SendATCommandGetReply("AT+QCFG=\"psm/urc\",1\r\n", buffer, 300);
	//BG96_SetErrorFormat(BG96_ERROR_VERBOSE);
	BG96_ConfigureURCIndication(BG96_URCINDICATION_USBAT);
	BG96_SetNetworkReporting(BG96_NETWORK_REPORTING_STAT);
	BG96_SetModemOptimization();
	#ifdef NBIOT_PROXIMUS
	BG96_SetPowerSavingMode(1, "", "", "\"00001010\"", "\"00001010\"");
	BG96_SetEDRXConfiguration(1,5,"\"0010\"");
	#ifdef DEBUG
	PRINTF_LN("PROX");
	#endif
	#else
	BG96_SetPowerSavingMode(1, "", "", "\"00001010\"", "\"00000000\""); // set tau timer to 1h, active timer to 30s (seems to work best in network)//20s
	BG96_SetEDRXConfiguration(0,5,"\"0000\"");
	#endif
	BG96_SetMode(BG96_NBIOT_MODE);
	
	#ifdef DEBUG
	PRINTF_LN("- Initialised");
	#endif
	
}

void registerNBIoT(void){

	// ---------- Configure/select network ---------- 
	BG96_ConfigureContext();
	#ifdef NBIOT_PROXIMUS
	BG96_SetPDPContext("\"m2minternet.proximus.be\""); // Orange: nbiot.iot, Proximus: m2minternet.proximus.be
	BG96_SelectNetwork(20601, BG96_NETWORK_NBIOT); // Proximus: 20601, Telenet: 20605, Orange: 20610, Base: 20620
	#endif
	//char pinbuffer[30];
	//memset(pinbuffer, '\0', 30);
	//BG96_SendATCommandGetReply("AT+CPIN?\r\n", pinbuffer, 1000);
	//char pinbuffer1[50];
	//BG96_SendATCommandGetReply("AT+COPS=?\r\n", pinbuffer1, 10000);
	
	#ifdef NBIOT_ORANGE
	BG96_SetPDPContext("\"nbiot.iot\""); // Orange: nbiot.iot, Proximus: m2minternet.proximus.be
	BG96_SelectNetwork(20610, BG96_NETWORK_NBIOT); // Proximus: 20601, Telenet: 20605, Orange: 20610, Base: 20620
	#endif
	
	
	#ifdef DEBUG
	PRINTF_LN("- Network selected");
	#endif
	
	// ---------- Connect to network ---------- 
	BG96_ConnectToOperator(90000);
	#ifdef DEBUG
	PRINTF_LN("- Connected to operator");
	#endif
	
	// ---------- Set power saving settings ---------- 
	#ifdef NBIOT_PROXIMUS
	BG96_SetPowerSavingMode(1, "", "", "\"00001010\"", "\"00001010\""); // set tau timer to 1h, active timer to 30s (seems to work best in network)//20s
	BG96_SetEDRXConfiguration(1,5,"\"0010\"");
	BG96_SetPowerSavingModeSettings(20,12);
	BG96_SetModemOptimization();
	#else
	BG96_SetPowerSavingMode(1, "", "", "\"00001010\"", "\"00000000\""); // set tau timer to 1h, active timer to 30s (seems to work best in network)//20s
	BG96_SetEDRXConfiguration(1,5,"\"0000\"");
	//BG96_SetEDRXConfiguration(0,0,"\"0000\"");
	#endif
	//
	//BG96_SetModemOptimization();
	#ifdef DEBUG
	PRINTF_LN("- Set power saving settings");
	#endif
	
	// ---------- Get various modem parameters ---------- 
//	uint8_t celevel; 
//	BG96_GetCELevel(&celevel);
//	#ifdef DEBUG
//	PRINTF_LN("- CE Level: %d", celevel);
//	#endif
//	
//	char* seperator = "|\0";
//	sprintf(energyStruct.nbiot_conditions, "%d|", celevel);
//	
//	char buffer[70];
//	BG96_SetErrorFormat(BG96_ERROR_VERBOSE);
//	BG96_GetNetworkStatus(buffer);
//	strcat(energyStruct.nbiot_conditions, buffer);
//	strcat(energyStruct.nbiot_conditions, seperator);
//	#ifdef DEBUG
//	PRINTF_LN("- Network status: %s", buffer);
//	#endif
//	
//	BG96_GetPowerSavingMode(buffer);
//	strcat(energyStruct.nbiot_conditions, buffer);
//	strcat(energyStruct.nbiot_conditions, seperator);
//	#ifdef DEBUG
//	PRINTF_LN("- Power saving mode settings: %s\r\n", buffer);
//	#endif
//	
//	BG96_GetEDRXConfiguration(buffer);
//	strcat(energyStruct.nbiot_conditions, buffer);
//	strcat(energyStruct.nbiot_conditions, seperator);
//	#ifdef DEBUG
//	PRINTF_LN("- Edrx mode settings: %s\r\n", buffer);
//	#endif
//	
//	BG96_GetSignalStength(buffer);
//	strcat(energyStruct.nbiot_conditions, buffer);
//	strcat(energyStruct.nbiot_conditions, seperator);
//	#ifdef DEBUG
//	PRINTF_LN("- Signal strength: %s\r\n", buffer);
//	#endif
//	
//	#ifdef DEBUG
//	PRINTF_LN("- Register procedure complete");
//	#endif

}

int8_t sendNBIoT(char * payload){
	#ifdef NBIOT_PROXIMUS
	PRINTF_LN("proximus");
	#endif	
	#ifdef NBIOT_ORANGE
	PRINTF_LN("orange");
	#endif
	
	PRINTF_LN("- Start sending NB-IoT");
		
	if(powerStatus == BG96_POWERDOWN){

		initNBIoT();
		registerNBIoT();

	}else{

		BG96_Init();

	}
	

	// ---------- Wake from psm and connect to network ---------- 
	if(BG96_WakeFromPSMToSend() == BG96_OK){
		// ---------- Get network info ---------- 

//		char bufferNetworkInfo[100];
//		BG96_GetNetworkInfo(bufferNetworkInfo);
//		PRINTF_LN("- Network info: %s", bufferNetworkInfo);

		
		// ---------- Connect to server ---------- 
		HAL_Delay(200);
		
		if(BG96_UDP_Start("94.224.9.123",8891) != BG96_OK){ //94.224.9.123  20.61.15.37
			PRINTF_LN("- UDP Start failed, going back to psm");
		}else{		

			BG96_UDP_SendData(payload, 30000);
			BG96_UDP_Stop();
			BG96_DeactivateContext();
			
			PRINTF_LN("- Done sending NB-IoT");
			BG96_SetPowerSavingModeImmediately();
			BG96_AbortRRC();
			BG96_SetPowerSavingModeImmediately();
			PRINTF_LN("- Waiting for PSM");
		}
	}
	
	// ---------- Wait for PSM ---------- 
	SCH_RegTask(WD_TASK, checkPowerDown);		  // Record send data task
	TimerInit(&WdTimer, onTimerWd);
	TimerSetValue(&WdTimer,  1000);
	TimerStart(&WdTimer); // Schedule next testing cycle
	
	SCH_ResumeTask(WD_TASK);
	
	uint32_t tick = HW_RTC_GetTimerValue();
	uint8_t counter = 0;
	while(!BG96_IsPoweredDown() && counter < 3){ 
		BG96_WaitForPowerDown(10000);
		PRINTF_LN("- PSM failed again, try again (%i).", counter);
		if(!BG96_IsPoweredDown()){
			BG96_SetPowerSavingModeImmediately();
			BG96_AbortRRC();
			BG96_SetPowerSavingModeImmediately();
		}
	  counter ++; 
	}
	if(!BG96_IsPoweredDown() && counter >= 3){
		PRINTF_LN("- Shutting down completely.");
		BG96_Powerdown_t pd = BG96_PowerDown();
		while(pd == BG96_POWERDOWN_ERROR && !BG96_IsPoweredDown()){
			pd = BG96_PowerDown();
			PRINTF_LN("- (PSM) Shutdown fail");
			powerStatus = BG96_ACTIVE; 
		}
		powerStatus = pd;
		
	}
	
	// ---------- Stop everything ----------
	BG96_DeInit();
	BG96_IoDeInit();
	TimerStop(&WdTimer);
	SCH_PauseTask(WD_TASK);
	
	// ---------- Increase packet number ----------

}
void sendNBIoTi(void){
		#ifdef NBIOT_PROXIMUS
	PRINTF_LN("proximus");
	#endif	
	#ifdef NBIOT_ORANGE
	PRINTF_LN("orange");
	#endif
	
	PRINTF_LN("- Start sending NB-IoT");
		
	if(powerStatus == BG96_POWERDOWN){

		initNBIoT();
		registerNBIoT();

	}else{

		BG96_Init();

	}
	

	// ---------- Wake from psm and connect to network ---------- 
	if(BG96_WakeFromPSMToSend() == BG96_OK){
		// ---------- Get network info ---------- 

//		char bufferNetworkInfo[100];
//		BG96_GetNetworkInfo(bufferNetworkInfo);
//		PRINTF_LN("- Network info: %s", bufferNetworkInfo);

		
		// ---------- Connect to server ---------- 
		HAL_Delay(200);
		
		if(BG96_UDP_Start("94.224.9.123",8891) != BG96_OK){ //94.224.9.123  20.61.15.37
			PRINTF_LN("- UDP Start failed, going back to psm");
		}
	}
}	

void sendNBIoTs(char * payload){	
	BG96_UDP_SendData(payload, 30000);
}
			
void sendNBIoTc(void){
	BG96_UDP_Stop();
	BG96_DeactivateContext();
	
	PRINTF_LN("- Done sending NB-IoT");
	BG96_SetPowerSavingModeImmediately();
	BG96_AbortRRC();
	BG96_SetPowerSavingModeImmediately();
	PRINTF_LN("- Waiting for PSM");	
	
	
	// ---------- Wait for PSM ---------- 
	SCH_RegTask(WD_TASK, checkPowerDown);		  // Record send data task
	TimerInit(&WdTimer, onTimerWd);
	TimerSetValue(&WdTimer,  1000);
	TimerStart(&WdTimer); // Schedule next testing cycle
	
	SCH_ResumeTask(WD_TASK);
	
	uint32_t tick = HW_RTC_GetTimerValue();
	uint8_t counter = 0;
	while(!BG96_IsPoweredDown() && counter < 3){ 
		BG96_WaitForPowerDown(10000);
		PRINTF_LN("- PSM failed again, try again (%i).", counter);
		if(!BG96_IsPoweredDown()){
			BG96_SetPowerSavingModeImmediately();
			BG96_AbortRRC();
			BG96_SetPowerSavingModeImmediately();
		}
	  counter ++; 
	}
	if(!BG96_IsPoweredDown() && counter >= 3){
		PRINTF_LN("- Shutting down completely.");
		BG96_Powerdown_t pd = BG96_PowerDown();
		while(pd == BG96_POWERDOWN_ERROR && !BG96_IsPoweredDown()){
			pd = BG96_PowerDown();
			PRINTF_LN("- (PSM) Shutdown fail");
			powerStatus = BG96_ACTIVE; 
		}
		powerStatus = pd;
		
	}
	
	// ---------- Stop everything ----------
	BG96_DeInit();
	BG96_IoDeInit();
	TimerStop(&WdTimer);
	SCH_PauseTask(WD_TASK);
	
	// ---------- Increase packet number ----------

}




//-------Timer testing stuff
void send_data_request_wd( void ){
  /* send task to background*/
  SCH_SetTask( WD_TASK );
}

static void onTimerWd(void *context){
	send_data_request_wd(); 
	
}

static void checkPowerDown(void){
	PRINTF_LN("WD");
	TimerStart(&WdTimer);
}
