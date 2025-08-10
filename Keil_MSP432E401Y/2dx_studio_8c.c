// 2DX4_Knowledge_Thread_3_Session_1
// This program illustrates the use of SysTick in the C language.
// Note the library headers asscoaited are PLL.h and SysTick.h,
// which define functions and variables used in PLL.c and SysTick.c.
// This program uses code directly from your course textbook.

//  Written by Mazen Maamon
//  March 15, 2022
//  Student Number: 400313173
//  MacID: maamonm
//  Assigned Bus Speed: 12
//  LED Status: PF0 

#include <stdint.h>
#include "tm4c1294ncpdt.h"
#include "vl53l1x_api.h"
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"




#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define MAXRETRIES              5           // number of receive attempts before giving up

//initialize the button as an input 
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	
  GPIO_PORTJ_DIR_R &= ~0x02;    										
  GPIO_PORTJ_DEN_R |= 0x02;     										
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								
	GPIO_PORTJ_AMSEL_R &= ~0x02;												
	GPIO_PORTJ_PUR_R |= 0x02;													
}

//initialize I2C protocol
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           												
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	
    GPIO_PORTB_ODR_R |= 0x08;             																	

    GPIO_PORTB_DEN_R |= 0x0C;             																	
//    GPIO_PORTB_AMSEL_R &= ~0x0C;          																

                                                                           
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       

        
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};   
    GPIO_PORTG_DIR_R &= 0x00;                                        
  GPIO_PORTG_AFSEL_R &= ~0x01;                                     
  GPIO_PORTG_DEN_R |= 0x01;                                        
                                                                                                    
  //GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  GPIO_PORTG_AMSEL_R &= ~0x01;                                     

    return;
}
//Port H is to initialize the pins for motor to control the stepper motor 
void PortH_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7; 
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){}; 
	GPIO_PORTH_DIR_R = 0b00001111; 
	GPIO_PORTH_DEN_R = 0b00001111; 
return;
}


void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        
    GPIO_PORTG_DATA_R &= 0b11111110;                                 
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                           
    
}
//This turns motor 45 degrees clockwise 
void spin(){
	int wait = 2;
	for(int i=0; i<64; i++){
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(wait);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(wait);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(wait);
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(wait);
	}
}
//This will turn the motor 360 degrees counter clockwise to go back to original position 
void spinccw(){
	for(int i=0; i<512; i++){
		GPIO_PORTH_DATA_R = 0b00001001;
		SysTick_Wait10ms(2);
		GPIO_PORTH_DATA_R = 0b00000011;
		SysTick_Wait10ms(2);
		GPIO_PORTH_DATA_R = 0b00000110;
		SysTick_Wait10ms(2);
		GPIO_PORTH_DATA_R = 0b00001100;
		SysTick_Wait10ms(2);
	}
}



//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status=0;

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;
	uint8_t id;
	uint8_t type;
	uint16_t both;

	//initialize
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortH_Init();
	PortJ_Init();


// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	
	
	status = VL53L1X_ClearInterrupt(dev); 
	
  
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

	


  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging


	// Get the Distance Measures 50 times
	int counter = 0;
	while(counter < 3)
	{
		if(GPIO_PORTJ_DATA_R == 0b00000000)
		{
			counter++;
			for(int i = 0; i < 8; i++) {
			
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			//read the data values from ToF sensor
			
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			FlashLED4(1);
	
			spin();

			status = VL53L1X_ClearInterrupt(dev); 
			
			// print the resulted readings to UART
			sprintf(printf_buffer,"%u\n",Distance);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(50);
			}
			spinccw();
		}
		else
		{
			SysTick_Wait10ms(50);
		}
		
	}
  
	VL53L1X_StopRanging(dev);
  while(1) {}

}

