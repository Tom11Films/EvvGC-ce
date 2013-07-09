#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_adc.h"
#include <stdio.h>
#include <math.h>

#define MPU6050_ADDRESS     0xD0   // Gyro address
#define EEPROM_ADDRESS      0xAE   //EEPROM address
#define MPU6050_I2C I2C2 //mpu6050 bus
#define LEDon  GPIO_WriteBit(GPIOB, GPIO_Pin_12,   Bit_SET) 
#define LEDoff GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_RESET) 



       GPIO_InitTypeDef        GPIO_InitStructure;
	    TIM_OCInitTypeDef       TIM_OCInitStructure;
TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_BDTRInitTypeDef 		TIM_BDTRInitStructure;
 USART_ClockInitTypeDef  USART_ClockInitStructure;
				ADC_InitTypeDef         ADC_InitStructure;
				I2C_InitTypeDef         I2C_InitStructure;



void Periph_clock_enable(void);
void GPIO_Config(void);
void Usart4Init(void);
void USART_PutChar(uint8_t ch);
void USART_PutString(uint8_t * str);
void I2C_Config(void);
void ADC_Config(void);
void Timer1_Config(void);
void Timer8_Config(void);
void Timer2_Config(void);
void Timer5_Config(void);
void Timer4_Config(void);
void MPU6050_Init(void);
void Delay_ms(uint32_t ms);
void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead);
void MPU6050_Gyro_get(void);
void MPU6050_ACC_get(void);
void pitch_PID(void);
void roll_PID(void);
void yaw_PID(void);
u16 readADC1(u8 channel);
void WriteToEEPROM(int addressToWrite, int DataToWrite);
void ReadFromEEPROM(u8 readAddr);
void NVIC_Configuration(void);
void UART4_IRQHandler();
void saveData(void);


int ConfigMode,  w, enable_writing;
int stop=0, i, YawPh1, YawPh2, YawPh3, EEPROM_REGISTER, EEPROM_DATA, EepromData, UART4_DATA;
float roll_amplitude, pitch_amplitude, yaw_amplitude, dt=0.002, ADC1Ch1_vid, sinus, cosinus, ROLL;
uint8_t XMSB,XLSB,YMSB,YLSB,ZMSB,ZLSB;
uint8_t out[] = {0,0,0,0,0,0};
short int gyroADC_PITCH, gyroADC_ROLL, gyroADC_YAW, accADC_ROLL, accADC_PITCH, accADC_YAW, printcounter;
float pitch, Gyro_Pitch_angle, pitch_output, pitch_setpoint=0, pitch_Error_current, pitch_Error_last, pitch_P, pitch_D, pitch_Kp, pitch_Kd, pitch_angle=0, pitch_angle_true, pitch_angle_correction;
float roll,   Gyro_Roll_angle,  roll_output,  roll_setpoint=0,  roll_Error_current,  roll_Error_last,  roll_P,  roll_D,  roll_Kp,  roll_Kd,  roll_angle=0,  roll_angle_correction;
float yaw,     Gyro_Yaw_angle,   yaw_output,   yaw_setpoint=0,   yaw_Error_current,   yaw_Error_last,   yaw_P,   yaw_D,   yaw_Kp,   yaw_Kd,   yaw_angle=0,   yaw_angle_correction;
float acc_pitch_angle, acc_roll_angle, accADC_x, accADC_y, accADC_z, gyroADC_x, gyroADC_y, gyroADC_z, fs, fssmall, acc_pitch_angle_vid, acc_roll_angle_vid;
uint8_t ACCread[6], GYROread[6];
int RXString[3];
char configData[9]={'1','1','1','1','1','1','1','1','1'};
float sinusas[91]={
0.000, 0.017, 0.035, 0.052, 0.070, 0.087, 0.105, 0.122, 0.139, 0.156,
0.174, 0.191, 0.208, 0.225, 0.242, 0.259, 0.276, 0.292, 0.309, 0.326, 
0.342, 0.358, 0.375, 0.391, 0.407, 0.423, 0.438, 0.454, 0.469, 0.485, 
0.500, 0.515, 0.530, 0.545, 0.559, 0.574, 0.588, 0.602, 0.616, 0.629, 
0.643, 0.656, 0.669, 0.682, 0.695, 0.707, 0.719, 0.731, 0.743, 0.755,
0.766, 0.777, 0.788, 0.799, 0.809, 0.819, 0.829, 0.839, 0.848, 0.857, 
0.866, 0.875, 0.883, 0.891, 0.899, 0.906, 0.914, 0.920, 0.927, 0.934, 
0.940, 0.945, 0.951, 0.956, 0.961, 0.966, 0.970, 0.974, 0.978, 0.982, 
0.985, 0.988, 0.990, 0.993, 0.995, 0.996, 0.998, 0.999, 0.999, 1.000,
1.000};

char buff[10];
int main(void)
{
	
	
	Periph_clock_enable(); 
	GPIO_Config();	
	Usart4Init();
	I2C_Config();
	ADC_Config();
	Delay_ms(100);
	MPU6050_Init();
	Timer1_Config();
	Timer8_Config();
	Timer2_Config();
	Timer5_Config();
	Timer4_Config();
	NVIC_Configuration();
	
	TIM_Cmd(TIM5, ENABLE);
	TIM_CtrlPWMOutputs(TIM5, ENABLE);
	for (i=1 ; i<1 ; i++) ;
	TIM_Cmd(TIM4, ENABLE);
	TIM_CtrlPWMOutputs(TIM4, ENABLE);
	
	LEDoff;
  
	
	

	Delay_ms(100);
	
			for(i=0; i<9;i++){
							ReadFromEEPROM(i);
							configData[i]=EepromData;							
							Delay_ms(5);							
							}
							 /* Enable Acknowledgement to be ready for another reception */
							I2C_AcknowledgeConfig(I2C2, ENABLE);
	Delay_ms(100);
	
pitch_Kp = 0.160;  //spring 
pitch_Kd = 0.80;//damping 

roll_Kp = 0.160;  //spring 
roll_Kd = 0.80;//damping 

yaw_Kp = 0.160;  //spring 
yaw_Kd = 0.80;//damping 


//pitch_amplitude=250; //500=100%
 //roll_amplitude=450; //500=100%
  //yaw_amplitude=450; //500=100%
 

while(1)
	{	
		LEDon;
		
		while(ConfigMode==1){
		LEDon;
		Delay_ms(100);
		LEDoff;
		Delay_ms(100);
		TIM4->CCR1=0;
		TIM4->CCR2=0;
		TIM4->CCR3=0;
		TIM5->CCR1=0;
		TIM5->CCR2=0;
		TIM5->CCR3=0;
		TIM1->CCR1=0;
		TIM1->CCR2=0;
		TIM1->CCR3=0;
		TIM8->CCR1=0;
		TIM8->CCR2=0;
		TIM8->CCR3=0;	
		}
		
		MPU6050_ACC_get();//Getting Accelerometer data
		
		acc_pitch_angle = -(atan2(accADC_x, accADC_z))-0.10;   //Calculating pitch ACC angle+callibration
		acc_roll_angle  = +(atan2(accADC_y, accADC_z));   //Calculating roll ACC angle		
		
		MPU6050_Gyro_get();//Getting Gyroscope data		
	
		acc_pitch_angle_vid=  ((acc_pitch_angle_vid*99.00)+acc_pitch_angle)/100.00;	//Averaging pitch ACC values
    acc_roll_angle_vid=   ((acc_roll_angle_vid* 99.00)+acc_roll_angle)/ 100.00; //Averaging roll ACC values		
		
		sinus   = sinusas[(int)(ADC1Ch1_vid*57.3)];      //Calculating sinus		
    cosinus = sinusas[90-(int)(ADC1Ch1_vid*57.3)];   //Calculating cosinus
		
		ROLL=-gyroADC_z*sinus+gyroADC_y*cosinus;
		roll_angle =(roll_angle + ROLL*dt)    + 0.0002*(acc_pitch_angle_vid-roll_angle); //Roll Horizon
		
		
		//ROLL=-gyroADC_z*sinus+gyroADC_y*cosinus;
		yaw_angle =(yaw_angle + gyroADC_z*dt); //Yaw
		
		
		
		pitch_angle_true=((pitch_angle_true  + gyroADC_x*dt) + 0.0002*(acc_roll_angle_vid-pitch_angle_true)); //Pitch Horizon
		
		ADC1Ch1_vid=  ((ADC1Ch1_vid*99.00)+(readADC1(1)/4000.00))/100.00;	//Averaging ADC values
		ADC1Ch1_vid=0.00;
		
		pitch_angle=pitch_angle_true-ADC1Ch1_vid;//Adding angle

		pitch_angle_correction=pitch_angle*150.0;
		if(pitch_angle_correction> 2.0){pitch_angle_correction= 2.0;}
		if(pitch_angle_correction<-2.0){pitch_angle_correction=-2.0;}
		pitch_setpoint=pitch_setpoint+pitch_angle_correction;//Pitch return to zero after collision
		
		roll_angle_correction=roll_angle*200.0;
		if(roll_angle_correction> 2.0){roll_angle_correction= 2.0;}
		if(roll_angle_correction<-2.0){roll_angle_correction=-2.0;}
		roll_setpoint=roll_setpoint+roll_angle_correction;//Roll return to zero after collision
		
		/*yaw_angle_correction=yaw_angle*150.0;
		if(yaw_angle_correction> 1.0){yaw_angle_correction= 1.0;}
		if(yaw_angle_correction<-1.0){yaw_angle_correction=-1.0;}
		yaw_setpoint=yaw_setpoint+yaw_angle_correction;//Roll return to zero after collision*/
		
    pitch_PID();//Pitch axis pid
		roll_PID(); //Roll axis pid
		yaw_PID(); //Yaw axis pid
		
		
		printcounter++; //Print data to UART
		if (printcounter>=100)
			{
				//sprintf (buff, " %d %d %c Labas\n\r", ACCread[0], ACCread[1], ACCread[2]);
				//sprintf (buff, " %x %x %x %x %x %x Labas\n\r", ACCread[0], ACCread[1], ACCread[2], ACCread[3], ACCread[4], ACCread[5]);
				//sprintf (buff, "Labas %d %d\n\r", ACCread[0], ACCread[1]);
				//sprintf (buff, "%3.1f %f\n\r", ADC1Ch1_vid*57.3, sinus);
				//sprintf (buff, "Labas %f %f %f \n\r", accADC_x, accADC_y, accADC_z);
				//sprintf (buff, "%3.1f %3.1f \n\r", acc_pitch_angle_vid*57.3,  acc_roll_angle_vid *57.3);
				//sprintf (buff, "%3.1f %3.1f \n\r", pitch_angle*57.3,  roll_angle*57.3);
				//sprintf (buff, "%3.3f\n\r", yaw_angle*57.3);
				//USART_PutString(buff);
				printcounter=0;
			}
		
    stop=0;
		LEDoff;	
		while(stop==0) {}//Closed loop waits for interrupt

		
	}
}

void Periph_clock_enable(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB  |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD  |
                         RCC_APB2Periph_GPIOE | 
												 RCC_APB2Periph_AFIO  | RCC_APB2Periph_ADC1	| RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5  | RCC_APB1Periph_TIM2 | RCC_APB1Periph_UART4   
												 | RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1,  ENABLE);
}

void GPIO_Config(void)
{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			GPIO_Init(GPIOB, &GPIO_InitStructure); 
	
			//CS_G
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			GPIO_Init(GPIOC, &GPIO_InitStructure); 
	
			//CS_G
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			GPIO_Init(GPIOA, &GPIO_InitStructure); 
}



void Usart4Init(void) 
{ 
			GPIO_InitTypeDef GPIO_InitStructure; 
			USART_InitTypeDef USART_InitStructure; 
			USART_ClockInitTypeDef USART_ClockInitStructure; 

			//Set USART2 Tx (PA.02) as AF push-pull 
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			GPIO_Init(GPIOC, &GPIO_InitStructure); 
			//Set USART2 Rx (PA.03) as input floating 
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; 
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
			GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	
			USART_ClockStructInit(&USART_ClockInitStructure);
			USART_ClockInit(UART4, &USART_ClockInitStructure); 
			USART_InitStructure.USART_BaudRate = 9600; 
			USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
			USART_InitStructure.USART_StopBits = USART_StopBits_1; 
			USART_InitStructure.USART_Parity = USART_Parity_No ; 
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; 
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
			//Write USART2 parameters 
			USART_Init(UART4, &USART_InitStructure); 
			//Enable UART4 Receive interrupt
			USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
			//Enable USART2 
			USART_Cmd(UART4, ENABLE); 
			}
			
			
			
void USART_PutChar(uint8_t ch)
{
  while(!(UART4->SR & USART_SR_TXE));
  UART4->DR = ch;
}

void USART_PutString(uint8_t * str)
{
while(*str != 0)
  {
  USART_PutChar(*str);
  str++;
}
}

void UART4_IRQHandler()//UART4 Interrupt handler implementation
{
    int eeRreg;
	  ConfigMode=1;
		while ( USART_GetFlagStatus(UART4, USART_FLAG_RXNE) == RESET);
    UART4_DATA=USART_ReceiveData(UART4);
LEDon;
		if(UART4_DATA==103){ //if "g"
							
							Delay_ms(100);
							sprintf (buff, "x");
							USART_PutString(buff);
							for(eeRreg=0; eeRreg<9;eeRreg++){
							ReadFromEEPROM(eeRreg);
							Delay_ms(5);
							sprintf (buff, "%c",EepromData);
							USART_PutString(buff);
							}
							 /* Enable Acknowledgement to be ready for another reception */
							I2C_AcknowledgeConfig(I2C2, ENABLE);
							}
							
							

							
		if(enable_writing==1){							
							configData[w]=(int)UART4_DATA;
							w++;
							if(w>8){w=0; enable_writing=0; saveData();}
							}
							
							
		if(UART4_DATA==104){ // if h (write to eeprom)
							
							enable_writing=1;
							}
		
							
		if(UART4_DATA==105){ 
							
							ConfigMode=1;
							}	
							
		if(UART4_DATA==106){
							
							ConfigMode=0;
							}								
			
		
		
	
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  
  /* Enable the USARTy Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}
void I2C_Config(void)
{
//Clock enable
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);	
//GPIO Setup
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  // Open Drain, I2C bus pulled high externally 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

//I2C Setup
	  I2C_Cmd(I2C2,ENABLE);		
	  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_ClockSpeed = 400000;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0xD0;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C2,&I2C_InitStructure);
    
	  I2C_AcknowledgeConfig(I2C2, ENABLE);

}


void MPU6050_Init(void)
{
	Delay_ms(5);
  I2C_GenerateSTART(I2C2, ENABLE); // Send I2C1 START condition 
  while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB));
  I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);// Send EEPROM slave Address for write  
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));// Test on I2C1 EV6 and clear it 
  I2C_SendData(I2C2, 0x19);// Send I2C1 EEPROM internal address //MPU6050_RA_SMPLRT_DIV
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Test on I2C1 EV8 and clear it   
  I2C_SendData(I2C2, 0x07);// Send I2C1 EEPROM data   
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));// Test on I2C1 EV8 and clear it  
  I2C_GenerateSTOP(I2C2, ENABLE);// Send I2C1 STOP Condition 
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
	
	I2C_GenerateSTART(I2C2, ENABLE); // Send I2C1 START condition 
  while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB));   
  I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);// Send EEPROM slave Address for write   
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));// Test on I2C1 EV6 and clear it  
  I2C_SendData(I2C2, 0x1A);// Send I2C1 EEPROM internal address 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //Test on I2C1 EV8 and clear it 
  I2C_SendData(I2C2, 0x02);// Send I2C1 EEPROM data LOW PASS..............................................................................
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));// Test on I2C1 EV8 and clear it 
  I2C_GenerateSTOP(I2C2, ENABLE);// Send I2C1 STOP Condition 
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
	
	I2C_GenerateSTART(I2C2, ENABLE); // Send I2C1 START condition 
  while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB));   
  I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);// Send EEPROM slave Address for write   
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));// Test on I2C1 EV6 and clear it  
  I2C_SendData(I2C2, 0x1B);// Send I2C1 EEPROM internal address 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //Test on I2C1 EV8 and clear it 
  I2C_SendData(I2C2, 0x08);// Send I2C1 EEPROM data 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));// Test on I2C1 EV8 and clear it 
  I2C_GenerateSTOP(I2C2, ENABLE);// Send I2C1 STOP Condition 
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
	
  I2C_GenerateSTART(I2C2, ENABLE); // Send I2C1 START condition 
  while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB));   
  I2C_Send7bitAddress(I2C2, MPU6050_ADDRESS, I2C_Direction_Transmitter);// Send EEPROM slave Address for write   
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));// Test on I2C1 EV6 and clear it  
  I2C_SendData(I2C2, 0x6B);// Send I2C1 EEPROM internal address 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); //Test on I2C1 EV8 and clear it 
  I2C_SendData(I2C2, 0x00);// Send I2C1 EEPROM data 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));// Test on I2C1 EV8 and clear it 
  I2C_GenerateSTOP(I2C2, ENABLE);// Send I2C1 STOP Condition 
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
	
	Delay_ms(5);

}

void ADC_Config(void)
{
// Configure ADC on ADC0_IN0    pin PA0
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
	

  /* PCLK2 is the APB2 clock */
  /* ADCCLK = PCLK2/6 = 72/6 = 12MHz*/
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  /* Enable ADC1 clock so that we can talk to it */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  /* Put everything back to power-on defaults */
  ADC_DeInit(ADC1);

  /* ADC1 Configuration ------------------------------------------------------*/
  /* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  /* Don't do contimuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 1;

  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

 
}

u16 readADC1(u8 channel)
{
  ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_28Cycles5);
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  // Get the conversion value
  return ADC_GetConversionValue(ADC1);
}

	void Delay_ms(uint32_t ms)
{
        volatile uint32_t nCount;
        RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq (&RCC_Clocks);

        nCount=(RCC_Clocks.HCLK_Frequency/10000)*ms;
        for (; nCount!=0; nCount--);
}

void MPU6050_I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{


/* While the bus is busy */
 while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY));

/* Send START condition */
 I2C_GenerateSTART(MPU6050_I2C, ENABLE);

/* Test on EV5 and clear it */
 while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

/* Send MPU6050 address for write */
 I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

 /* Test on EV6 and clear it */
 while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

/* Clear EV6 by setting again the PE bit */
 I2C_Cmd(MPU6050_I2C, ENABLE);

/* Send the MPU6050's internal address to write to */
 I2C_SendData(MPU6050_I2C, readAddr);

/* Test on EV8 and clear it */
 while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

/* Send STRAT condition a second time */
 I2C_GenerateSTART(MPU6050_I2C, ENABLE);

/* Test on EV5 and clear it */
 while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT));

/* Send MPU6050 address for read */
 I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);

/* Test on EV6 and clear it */
 while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

/* While there is data to be read */
 while(NumByteToRead)
 {
 if(NumByteToRead == 1)
 {
 /* Disable Acknowledgement */
 I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

/* Send STOP Condition */
 I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
 }

/* Test on EV7 and clear it */
 if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
 {
 /* Read a byte from the MPU6050 */
 *pBuffer = I2C_ReceiveData(MPU6050_I2C);

/* Point to the next location where the byte read will be saved */
 pBuffer++;

/* Decrement the read bytes counter */
 NumByteToRead--;
 }
 }

/* Enable Acknowledgement to be ready for another reception */
 I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);


}


void MPU6050_ACC_get(void)
{
	MPU6050_I2C_BufferRead(MPU6050_ADDRESS, ACCread, 0X3B, 6);
		accADC_ROLL  =  (((ACCread[0]<<8) | ACCread[1]));
		accADC_x  =(accADC_ROLL);
		accADC_PITCH =  (((ACCread[2]<<8) | ACCread[3]));
		accADC_y=(accADC_PITCH);
		accADC_YAW   =  (((ACCread[4]<<8) | ACCread[5]));
		accADC_z    =(accADC_YAW);
}	

void MPU6050_Gyro_get(void)
{
		MPU6050_I2C_BufferRead(MPU6050_ADDRESS, GYROread, 0X43, 6);
		gyroADC_ROLL  =  (((GYROread[0]<<8) | GYROread[1]));
		gyroADC_x=((float)gyroADC_ROLL+300)/8000.00;
		gyroADC_PITCH =  (((GYROread[2]<<8) | GYROread[3]));
		gyroADC_y=((float)gyroADC_PITCH+320)/8000.00;
		gyroADC_YAW   =  (((GYROread[4]<<8) | GYROread[5]));
		gyroADC_z=((float)gyroADC_YAW+350)/8000.00;
}
void WriteToEEPROM(int addressToWrite, int DataToWrite)//Write data to external EEPROM
{
	Delay_ms(5);
  I2C_GenerateSTART(I2C2, ENABLE); // Send I2C2 START condition 
  while(!I2C_GetFlagStatus(I2C2, I2C_FLAG_SB));
  I2C_Send7bitAddress(I2C2, EEPROM_ADDRESS, I2C_Direction_Transmitter);// Send EEPROM slave Address for write  
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));// Test on I2C2 EV6 and clear it 
  I2C_SendData(I2C2, addressToWrite);// Send I2C2 EEPROM internal address 
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Test on I2C2 EV8 and clear it   
  I2C_SendData(I2C2, DataToWrite);// Send I2C2 EEPROM data   
  while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));// Test on I2C2 EV8 and clear it  
  I2C_GenerateSTOP(I2C2, ENABLE);// Send I2C2 STOP Condition 
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));
	Delay_ms(5);
}

void ReadFromEEPROM(u8 readAddr)
{


/* While the bus is busy */
 while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

/* Send START condition */
 I2C_GenerateSTART(I2C2, ENABLE);

/* Test on EV5 and clear it */
 while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

/* Send MPU6050 address for write */
 I2C_Send7bitAddress(I2C2, EEPROM_ADDRESS, I2C_Direction_Transmitter);

 /* Test on EV6 and clear it */
 while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

/* Clear EV6 by setting again the PE bit */
 I2C_Cmd(I2C2, ENABLE);

/* Send the MPU6050's internal address to write to */
 I2C_SendData(I2C2, readAddr);

/* Test on EV8 and clear it */
 while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

/* Send STRAT condition a second time */
 I2C_GenerateSTART(I2C2, ENABLE);

/* Test on EV5 and clear it */
 while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

/* Send eeprom address for read */
 I2C_Send7bitAddress(I2C2, EEPROM_ADDRESS, I2C_Direction_Receiver);


/* Test on EV6 and clear it */
while(!I2C_CheckEvent(I2C2 ,I2C_EVENT_MASTER_BYTE_RECEIVED));

/* Read a byte from the 24C02 */
EepromData = I2C_ReceiveData(I2C2);

/* Disable Acknowledgement */
I2C_AcknowledgeConfig(I2C2, DISABLE); 
 
/* Send STOP Condition */
 I2C_GenerateSTOP(I2C2, ENABLE);
 


}

void saveData(void){
	int eeRegCount;	
		
	

	for(eeRegCount=0; eeRegCount<9;eeRegCount++){
							LEDon;
							WriteToEEPROM(eeRegCount, configData[eeRegCount]);
							Delay_ms(5);							
							}
							
							LEDoff;
	
}
void Timer1_Config(void)//ROLL Timer configuration
{	

	//Timer1 Config
	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Time Base configuration 
	TIM_TimeBaseInitStructure.TIM_Prescaler = 5; // Period*Prescaler=24'000'000Hz  //2400 1s
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);		

  //Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);	
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);	
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);	


  //Automatic Output enable, Break, dead time and lock configuration
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 80;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	
  //TIM1 counter enable
  TIM_Cmd(TIM1, ENABLE);

  //Main Output Enable
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

}	

void Timer8_Config(void)//Pitch Timer configuration
{	

	//Timer8 Config
	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Time Base configuration 
	TIM_TimeBaseInitStructure.TIM_Prescaler = 5; // Period*Prescaler=24'000'000Hz  //2400 1s
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);		

  //Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	
  TIM_OC1Init(TIM8, &TIM_OCInitStructure);	
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);	
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);	


  //Automatic Output enable, Break, dead time and lock configuration
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = 80;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
  TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
	
  //TIM8 counter enable
  TIM_Cmd(TIM8, ENABLE);

  //Main Output Enable
  TIM_CtrlPWMOutputs(TIM8, ENABLE);

}	
void Timer2_Config(void)//Main(loop) Timer configuration
{
	//Timer2 Config
	TIM2->PSC = 71;         // Set prescaler (PSC + 1)
	TIM2->ARR = 2000;           // Auto reload value 2000
	TIM2->DIER = TIM_DIER_UIE; // Enable update interrupt (timer level)
	TIM2->CR1 = TIM_CR1_CEN;   // Enable timer
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt from TIM2 (NVIC level)
}

void TIM2_IRQHandler(void)
{
if(TIM2->SR & TIM_SR_UIF) // if UIF flag is set
  {
  TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag  
	stop=1;
  }
}

void Timer5_Config(void)//Pitch Timer configuration
{	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Timer5 Config
		TIM_TimeBaseInitStructure.TIM_Prescaler = 5; // Period*Prescaler=24'000'000Hz  //2400 1s
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);	
	
	//PWM Config on Timer5 CH3
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	//PWM Config on Timer5 CH2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);	
	//PWM Config on Timer5 CH1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM5, &TIM_OCInitStructure);		
	

}	

void Timer4_Config(void)//Pitch Timer configuration
{	
	//PWM pin config
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	//Timer4 Config

	TIM_TimeBaseInitStructure.TIM_Prescaler = 5; // Period*Prescaler=24'000'000Hz  //2400 1s
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_Period = 1000; //20000(presc=24)=50hz(servo signal)
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);	


	//PWM Config on Timer4 CH3
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	
	//PWM Config on Timer4 CH2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);	
	//PWM Config on Timer4 CH1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;      
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);		
	

}	


void pitch_PID(void){
      //-------------------------------------PID Pitch-------------------------------
      
        pitch_Error_current = pitch_setpoint + pitch_angle*1000;        
       
        pitch_P = pitch_Error_current * (float)configData[0]/100;

        pitch_D = (float)configData[3]/100 * (pitch_Error_current - pitch_Error_last);
        pitch_Error_last = pitch_Error_current;
       
        
             pitch_output = (pitch_P  + pitch_D);
            
  
       
		TIM8->CCR1=(sin(pitch_output     )*5*configData[6])+500;
		TIM8->CCR2=(sin(pitch_output+2.09)*5*configData[6])+500;
		TIM8->CCR3=(sin(pitch_output+4.19)*5*configData[6])+500; 
      //----------------------------------------------------------------------------       
} 

void roll_PID(void){
      //-------------------------------------Roll Pitch-------------------------------
      
        roll_Error_current = roll_setpoint + roll_angle*1000;        
       
        roll_P = roll_Error_current * (float)configData[1]/100;

        roll_D = (float)configData[4]/100 * (roll_Error_current - roll_Error_last);
        roll_Error_last = roll_Error_current;
       
        
             roll_output = (roll_P  + roll_D);
            
		TIM1->CCR1=(sin(roll_output     )*5*configData[7])+500;
		TIM1->CCR2=(sin(roll_output+2.09)*5*configData[7])+500;
		TIM1->CCR3=(sin(roll_output+4.19)*5*configData[7])+500; 
      //----------------------------------------------------------------------------       
} 

void yaw_PID(void){
      //-------------------------------------Yaw Pitch-------------------------------
      
        yaw_Error_current = yaw_setpoint + yaw_angle*1000;        
       
        yaw_P = yaw_Error_current * (float)configData[2]/100;

        yaw_D = (float)configData[5]/100 * (yaw_Error_current - yaw_Error_last);
        yaw_Error_last = yaw_Error_current;
       
        
             yaw_output = (yaw_P  + yaw_D);
            
  
       
		YawPh1=(sin(yaw_output     )*5*configData[8])+500;
		YawPh2=(sin(yaw_output+2.09)*5*configData[8])+500;
		YawPh3=(sin(yaw_output+4.19)*5*configData[8])+500;
	
	TIM4->CCR1=YawPh1;
	TIM4->CCR2=YawPh2;
	TIM4->CCR3=YawPh3;
	
	YawPh1=YawPh1+30;
	YawPh2=YawPh2+30;
	YawPh3=YawPh3+30;
	
	if(YawPh1>=1000) YawPh1=1000;
	if(YawPh2>=1000) YawPh2=1000;
	if(YawPh3>=1000) YawPh3=1000;
	
	TIM5->CCR1=YawPh1;
	TIM5->CCR2=YawPh2;
	TIM5->CCR3=YawPh3;
      //----------------------------------------------------------------------------       
} 
