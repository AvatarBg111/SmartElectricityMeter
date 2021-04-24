#include "esp32_Rx_Tx.h"
#include "mcp39f511.h"
#include "electrical_consumers.h"
#include "delay.h"
#include "spi_flash.h"
#include "lfs.h"
#include "main.h"


/////////////////////////////// INCLUDES ///////////////////////////////////////

#define MAX_VAL 4000
#define MIN_VAL 50
#define KOEF_VAL (MAX_VAL-MIN_VAL)/100

////////////////////////////// DEFINITIONS /////////////////////////////////////

void Init_RCC();
void Init_RTC();
void Init_USART1();
void Init_USART2();
void Init_GPIO();
void ConfigureADC();
void ConfigureDAC();
void ConfigureTIM10();

event_measurement_struct_t event_measurement[100];	// Roll buffer for send to ESP32
MCP_measurement_struct_t shot_measurement[100];	// 100 measurements (N = 0) for started event
MCP_measurement_struct_t MCP_measurement;	// For measurements
event_control_struct_t event_control;

uint8_t first_empty_element_of_roll_buffer = 0;	// For event_measurement
uint8_t first_empty_element_of_roll_buffer_old = 0;
uint8_t first_element_for_transmit_of_roll_buffer = 0;	// To ESP32 (STM32 - USART2)
uint8_t first_empty_element_of_shot_measurement = 0;	// For shot_measurement

////////////////////////////////////////////////////////////////////////////////

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#define  FLASH_WRITE_ADDRESS      0x700000
#define  FLASH_READ_ADDRESS       FLASH_WRITE_ADDRESS
#define  FLASH_SECTOR_TO_ERASE    FLASH_WRITE_ADDRESS

#define  BufferSize (countof(Tx_Buffer)-1)

/* Private macro -------------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
uint8_t Tx_Buffer[] = "STM32F4xx SPI Firmware Library Example: communication with an M25P SPI FLASH";
uint8_t Rx_Buffer[BufferSize];
uint8_t UART2_buffer[256];
__IO uint8_t Index = 0x0;
volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = PASSED;
__IO uint32_t FlashID = 0;

/* Private functions ---------------------------------------------------------*/
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);

////////////////////////////////////////////////////////////////////////////////

int block_device_read(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, void *buffer, lfs_size_t size)
{
	sFLASH_ReadBuffer((uint8_t*)buffer, (block * c->block_size + off), size);

	return 0;
}

int block_device_prog(const struct lfs_config *c, lfs_block_t block,
	lfs_off_t off, const void *buffer, lfs_size_t size)
{
	sFLASH_WriteBuffer((uint8_t*)buffer, (block * c->block_size + off), size);
	
	return 0;
}

int block_device_erase(const struct lfs_config *c, lfs_block_t block)
{
	sFLASH_EraseSector(block * c->block_size);
	
	return 0;
}

int block_device_sync(const struct lfs_config *c)
{
	return 0;
}

lfs_t lfs;
lfs_file_t file;
struct lfs_config cfg;

uint8_t lfs_read_buf[256];
uint8_t lfs_prog_buf[256];
uint8_t lfs_lookahead_buf[16];	// 128/8=16
uint8_t lfs_file_buf[256] = "vld";

void LFS_Config(struct lfs_config *cfg)
{
	/* block device operations */
	cfg->read  = block_device_read;
	cfg->prog  = block_device_prog;
	cfg->erase = block_device_erase;
	cfg->sync  = block_device_sync;

	/* block device configuration */
	cfg->read_size = 256;
	cfg->prog_size = 256;
	cfg->block_size = 4096;
	cfg->block_count = 4096;
	cfg->block_cycles = 500;	// By me
	cfg->cache_size = 256;	// By me
	cfg->lookahead_size = 256;
	
	cfg->read_buffer = lfs_read_buf;
	cfg->prog_buffer = lfs_prog_buf;
	cfg->lookahead_buffer = lfs_lookahead_buf;
	//cfg->file_buffer = lfs_file_buf;

	cfg->name_max = 255;
}

////////////////////////////////////////////////////////////////////////////////

void Init_RCC(){
	//GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_ClocksTypeDef RCC_ClockFreq;

  	/* This function fills the RCC_ClockFreq structure with the current
     	frequencies of different on chip clocks (for debug purpose) **************/
  	RCC_GetClocksFreq(&RCC_ClockFreq);
  
  	/* Enable Clock Security System(CSS): this will generate an NMI exception
     	when HSE clock fails *****************************************************/
  	RCC_ClockSecuritySystemCmd(ENABLE);
 
  	/* Enable and configure RCC global IRQ channel, will be used to manage HSE ready 
     	and PLL ready interrupts. 
     	These interrupts are enabled in stm32f4xx_it.c file **********************/
  	NVIC_InitStructure.NVIC_IRQChannel = RCC_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

#if(0)
	/* Output HSE clock on MCO1 pin(PA8) ****************************************/ 
  	/* Enable the GPIOA peripheral */ 
  	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  
  	/* Configure MCO1 pin(PA8) in alternate function */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  	/* HSE clock selected to output on MCO1 pin(PA8)*/
  	RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
#endif

	//RCC_HSICmd(ENABLE);
    //RCC_PLLCmd(ENABLE);
    //RCC_PLLConfig(RCC_PLLSource_HSE, 0x04, 50, 0x02, 0x04);
    //RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
    //RCC_HCLKConfig(RCC_SYSCLK_Div1);
    //RCC_PCLK1Config(RCC_HCLK_Div4);
    //RCC_PCLK2Config(RCC_HCLK_Div2);
    
    RCC_ClocksTypeDef rcc_freq;
    RCC_GetClocksFreq(&rcc_freq);
    //printf("\nRCC INITIALIZATION: \nSystem clock freq - %d\nHCLK clock freq - %d\nPCLK1 clock freq - %d\nPCLK2 clock freq - %d\n", rcc_freq.SYSCLK_Frequency, rcc_freq.HCLK_Frequency, rcc_freq.PCLK1_Frequency, rcc_freq.PCLK2_Frequency);
}

void SysTickInit(uint16_t frequency){
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);
    (void) SysTick_Config(RCC_Clocks.HCLK_Frequency / frequency);
}

void Init_RTC(){
    ///INITIALIZE RTC_Init structure and error variable///
    RTC_InitTypeDef rtc_struct;
    ErrorStatus possible_error;    

    ///Enable Power Control Unit to enable shadow register reading///
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    ///Make RTC Reset, Smooth Calibration, Bypass of LSI through Smooth Calibration Unit, Enable LSI, make LSI check///
    RCC_RTCCLKCmd(DISABLE);
    RCC_RTCCLKCmd(ENABLE);
    RTC_SmoothCalibConfig(RTC_SmoothCalibPeriod_8sec, RTC_SmoothCalibPlusPulses_Set, 0x07F);
    RCC_LSEConfig(RCC_LSE_ON);
    while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

    ///Configure RTC to use LSE, fill RTC_Init structure///
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    rtc_struct.RTC_AsynchPrediv = (uint32_t)0x7F;
    rtc_struct.RTC_SynchPrediv = (uint32_t)0xFF;
    rtc_struct.RTC_HourFormat = RTC_HourFormat_24;

    ///ENABLE RTC Unit and wait for synchronisation///
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForSynchro();

    ///Initialize RTC Unit and check init status///
    possible_error = RTC_Init(&rtc_struct);
    printf("\n\nStatus in Initialization: %d", possible_error);   

    ///Set RTC Date and check error status///
    RTC_DateTypeDef rtc_date;
    rtc_date.RTC_WeekDay = RTC_Weekday_Monday;
    rtc_date.RTC_Month = RTC_Month_March;
    rtc_date.RTC_Date = 1;
    rtc_date.RTC_Year = 51;
    possible_error = RTC_SetDate(RTC_Format_BIN, &rtc_date);
    printf("\nStatus in setting date: %d", possible_error);

    ///Set RTC Time and check error status///
    RTC_TimeTypeDef rtc_time;
    rtc_time.RTC_Hours = 0;
    rtc_time.RTC_Minutes = 0;
    rtc_time.RTC_Seconds = 0;
    rtc_time.RTC_H12 = RTC_H12_AM;
    possible_error = RTC_SetTime(RTC_Format_BIN, &rtc_time);
    printf("\nStatus in setting time: %d\n\n", possible_error);
}

void Init_USART1(){
    GPIO_InitTypeDef gpio;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    gpio.GPIO_Pin = GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    
    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitTypeDef usart1_config;
    usart1_config.USART_BaudRate = 115200;
    usart1_config.USART_WordLength = USART_WordLength_8b;
    usart1_config.USART_StopBits = USART_StopBits_1;
    usart1_config.USART_Parity = USART_Parity_No;
    usart1_config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart1_config);
    
    //USART_OneBitMethodCmd(USART1, ENABLE);
    USART_OverSampling8Cmd(USART1, DISABLE);

    NVIC_InitTypeDef nvic_init;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    nvic_init.NVIC_IRQChannel = USART1_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}

void Init_USART2(){
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    USART_SetPrescaler(USART2, 0x04);
    USART_OverSampling8Cmd(USART2, ENABLE);

    USART_InitTypeDef usart2_config;
    usart2_config.USART_BaudRate = 115200;
    usart2_config.USART_WordLength = USART_WordLength_8b;
    usart2_config.USART_StopBits = USART_StopBits_1;
    usart2_config.USART_Parity = USART_Parity_No;
    usart2_config.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    usart2_config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usart2_config);

    NVIC_InitTypeDef nvic_init;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    nvic_init.NVIC_IRQChannel = USART2_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART2, ENABLE);
}

void Init_GPIO(){
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_0;					// For BUTTON
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_1;					// For LED
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_8;                 // For "PA8 Test"
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_5;                 // For "PC5 Test"
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
}

void MCU_Init(){
    Init_RCC();
	SysTickInit(1000);
    if(!RTC_FLAG_INITS) Init_RTC();
    Init_USART1();
    Init_USART2();
    Init_GPIO();
    Init_Mcp39f511();
    //ConfigureADC();
    //ConfigureDAC();
    //ConfigureTIM10();
}

#if(0)
void ConfigureADC(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);////LEROLEROLEROLEROLERO
    ///
    GPIO_InitTypeDef adc_gpio;
    adc_gpio.GPIO_Pin = GPIO_Pin_6;
    adc_gpio.GPIO_Mode = GPIO_Mode_AN;
    adc_gpio.GPIO_Speed = GPIO_Speed_100MHz;
    adc_gpio.GPIO_OType = GPIO_OType_PP;
    adc_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &adc_gpio);
    ///
    ADC_InitTypeDef adc_init;
    adc_init.ADC_Resolution = ADC_Resolution_12b;
    adc_init.ADC_ScanConvMode = DISABLE;
    adc_init.ADC_ContinuousConvMode = /*DISABLE*/ENABLE;
    adc_init.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//ADC_ExternalTrigConvEdge_None;//ADC_ExternalTrigConvEdge_Rising;
    adc_init.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    adc_init.ADC_DataAlign = ADC_DataAlign_Right;
    adc_init.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &adc_init);
    ///
    ADC_Cmd(ADC1, ENABLE);
    ///
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_112Cycles);
    ///
    ADC_ContinuousModeCmd(ADC1, ENABLE);
    /*ADC_DiscModeChannelCountConfig(ADC1, 5);
    ADC_DiscModeCmd(ADC1, ENABLE);*/  //Discontinuous mode
    ADC_SoftwareStartConv(ADC1);
}

void ConfigureDAC(){
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    
    GPIO_InitTypeDef dac_gpio;
    dac_gpio.GPIO_Pin = GPIO_Pin_4;
    dac_gpio.GPIO_Mode = GPIO_Mode_AN;
    dac_gpio.GPIO_Speed = GPIO_Speed_100MHz;
    dac_gpio.GPIO_OType = GPIO_OType_PP;
    dac_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &dac_gpio);

    DAC_InitTypeDef dac_init;
    dac_init.DAC_Trigger = DAC_Trigger_None;
    dac_init.DAC_WaveGeneration = DAC_WaveGeneration_None;
    dac_init.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
    dac_init.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    DAC_Init(DAC_Channel_1, &dac_init);
    
    DAC_SetChannel1Data(DAC_Align_12b_R, 400);
    DAC_Cmd(DAC_Channel_1, ENABLE);
}

void ConfigureTIM10(){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

    TIM_TimeBaseInitTypeDef tim_init;
    NVIC_InitTypeDef nvic_init;

    tim_init.TIM_Prescaler = 0xFFFF;
    tim_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_init.TIM_Period = 0x0000;
    tim_init.TIM_ClockDivision = TIM_CKD_DIV4;
    tim_init.TIM_RepetitionCounter = 0x0000;

    TIM_TimeBaseInit(TIM10, &tim_init);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    nvic_init.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = 0;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM10, ENABLE);
}

void TIM1_UP_TIM10_IRQHandler(){
  static int counter = 0;
  int value = ADC_GetConversionValue(ADC1);
  ++counter;
  
  DAC_SetChannel1Data(DAC_Align_12b_R, 750 + ((int)(value/((MAX_VAL-MIN_VAL)/100)))*8);
  if(counter == 0 || counter % (int)(value/250 + 1) == 0) GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  else  GPIO_SetBits(GPIOA, GPIO_Pin_1);

  TIM_ClearFlag(TIM10, 0x0);
}
#endif

uint16_t read_from_RtcBKR(uint8_t id_consumer){
	if(RTC_ReadBackupRegister(RTC_BKP_DR0) == 0xA5A5A5A5){
		if(id_consumer == CONSUMER_LAMP){
			return (uint16_t)RTC_ReadBackupRegister(RTC_BKP_DR1);
		}
		if(id_consumer == CONSUMER_IRON){
			return (uint16_t)RTC_ReadBackupRegister(RTC_BKP_DR2);
		}
	}
	return 0;
}

void backup_to_RtcBKR(uint8_t id_consumer, uint16_t id_event){
	RTC_WriteBackupRegister(RTC_BKP_DR0, 0xA5A5A5A5);
	if(id_consumer == CONSUMER_LAMP){
		RTC_WriteBackupRegister(RTC_BKP_DR1, (uint32_t)id_event);
	}
	if(id_consumer == CONSUMER_IRON){
		RTC_WriteBackupRegister(RTC_BKP_DR2, (uint32_t)id_event);
	}
}



void main(){
    MCU_Init();
	LFS_Config(&cfg);
	GPIO_SetBits(GPIOA, GPIO_Pin_1);	// LED off
	PWR_BackupAccessCmd(ENABLE);

#if(1)
////////////////////////////////////////////////////////////////////////////////

  /* Initialize the SPI FLASH driver */
	sFLASH_Init();

  /* Get SPI Flash ID */
    FlashID = sFLASH_ReadID();
  
  /* Check the SPI Flash ID */
  if (FlashID == sFLASH_W25Q16_ID)
  {
    /* OK: Turn on LED */
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);	// TEST

    /* Perform a write in the Flash followed by a read of the written data */
    /* Erase SPI FLASH Sector to write on */
    sFLASH_EraseSector(FLASH_SECTOR_TO_ERASE);

    /* Write Tx_Buffer data to SPI FLASH memory */
    sFLASH_WriteBuffer(Tx_Buffer, FLASH_WRITE_ADDRESS, BufferSize);

    /* Read data from SPI FLASH memory */
    sFLASH_ReadBuffer(Rx_Buffer, FLASH_READ_ADDRESS, BufferSize);

    /* Check the correctness of written dada */
    TransferStatus1 = Buffercmp(Tx_Buffer, Rx_Buffer, BufferSize);
    /* TransferStatus1 = PASSED, if the transmitted and received data by SPI1
       are the same */
    /* TransferStatus1 = FAILED, if the transmitted and received data by SPI1
       are different */
	printf("TransferStatus1: %0x\r\n", TransferStatus1);

    /* Perform an erase in the Flash followed by a read of the written data */
    /* Erase SPI FLASH Sector to write on */
    sFLASH_EraseSector(FLASH_SECTOR_TO_ERASE);

    /* Read data from SPI FLASH memory */
    sFLASH_ReadBuffer(Rx_Buffer, FLASH_READ_ADDRESS, BufferSize);

    /* Check the correctness of erasing operation dada */
    for (Index = 0; Index < BufferSize; Index++)
    {
      if (Rx_Buffer[Index] != 0xFF)
      {
        TransferStatus2 = FAILED;
      }
    }
    /* TransferStatus2 = PASSED, if the specified sector part is erased */
    /* TransferStatus2 = FAILED, if the specified sector part is not well erased */
	printf("TransferStatus2: %0x\r\n", TransferStatus2);
  }
  else
  {
    /* Error: Turn on LD2 */
    //STM_EVAL_LEDOn(LED2);
	GPIO_SetBits(GPIOA, GPIO_Pin_0);	// TEST
  }
  
////////////////////////////////////////////////////////////////////////////////
#endif

#if(1)
////////////////////////////////////////////////////////////////////////////////

	uint32_t err = lfs_mount(&lfs, &cfg);

	if(err){
		lfs_format(&lfs, &cfg);
		lfs_mount(&lfs, &cfg);
	}

	strcpy((char*)lfs_file_buf, "something");
	lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
	lfs_file_write(&lfs, &file, &lfs_file_buf, sizeof(lfs_file_buf));
    printf("Reset: %s\r\n", lfs_file_buf);
    lfs_file_close(&lfs, &file);
    strcpy((char*)lfs_file_buf, "");

	lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR);
	lfs_file_read(&lfs, &file, &lfs_file_buf, sizeof(lfs_file_buf));
    strcpy((char*)UART2_buffer, (char*)lfs_file_buf);
	printf("Read: %0x Sprintfed buffer:\n%s\r\n", lfs_file_buf, (char*)UART2_buffer);
    strcpy(event_measurement[0].timestamp_time_start, "10042021 021056");
    event_measurement[0].id_measurement = 0x0001;
    event_measurement[0].id_consumer = 0x0000;
    event_measurement[0].id_event = 0x0001;
    event_measurement[0].power = 0x0001;
    Fill_Tx_buffer(&event_measurement[0]);
    Transmit_to_esp32();

	//boot_count += 1;
	//lfs_file_rewind(&lfs, &file);
	//lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));


	while (!(USART1->SR & USART_SR_TC));
   	USART_SendData(USART1, 0x00);

	lfs_file_close(&lfs, &file);

	while (!(USART1->SR & USART_SR_TC));
   	USART_SendData(USART1, 0x00);

	lfs_unmount(&lfs);

////////////////////////////////////////////////////////////////////////////////
#endif

	Get_Single_Wire_Rx_Frame(0x01);
		
	uint8_t transmit_to_esp32_flag = 0;	// TEST
#if(0)
	backup_to_RtcBKR(CONSUMER_LAMP, 0x0005);
	backup_to_RtcBKR(CONSUMER_IRON, 0x0005);
#endif

    while(1){
		Get_Single_Wire_Rx_Frame(0x00);
		if(event_control.has_a_shot){
			set_id_consumer();
			//event_control.id_event = (read_from_RtcBKR(event_control.id_consumer) + 1);
			if(event_control.id_consumer == CONSUMER_LAMP) event_control.id_event = read_from_RtcBKR(CONSUMER_LAMP) +1;
			if(event_control.id_consumer == CONSUMER_IRON) event_control.id_event = read_from_RtcBKR(CONSUMER_IRON) +1;
			event_control.id_measurement = 0;
			//backup_to_RtcBKR(event_control.id_consumer, event_control.id_event);
			if(event_control.id_consumer == CONSUMER_LAMP) backup_to_RtcBKR(CONSUMER_LAMP, event_control.id_event);
			if(event_control.id_consumer == CONSUMER_IRON) backup_to_RtcBKR(CONSUMER_IRON, event_control.id_event);
			//printf("\n%d\n", event_control.id_event);	// TEST
			//printf("\n%d\n", event_control.id_consumer);
			event_control.has_a_shot = 0;
		}

/*		if(READ_RxSTATUS_FLAG){
 			printf("\nDATA: %s\n", rx_data);
			strcpy(rx_data, "");
			READ_RxSTATUS_FLAG = ~READ_RxSTATUS_FLAG;
        }*/
		
		if(first_empty_element_of_roll_buffer != first_element_for_transmit_of_roll_buffer) transmit_to_esp32_flag = 1;
        if(transmit_to_esp32_flag){
#if(1)			/* TEST */
			if(!event_measurement[first_element_for_transmit_of_roll_buffer].id_measurement){
				printf("\n%d %03d %d %.02f(W)\n", event_measurement[first_element_for_transmit_of_roll_buffer].id_consumer, \
					event_measurement[first_element_for_transmit_of_roll_buffer].id_event, \
						event_measurement[first_element_for_transmit_of_roll_buffer].id_measurement, \
							(float)event_measurement[first_element_for_transmit_of_roll_buffer].power/100);	// TEST
			} else {
				printf("%d %03d %d %d\n", event_measurement[first_element_for_transmit_of_roll_buffer].id_consumer, \
					event_measurement[first_element_for_transmit_of_roll_buffer].id_event, \
						event_measurement[first_element_for_transmit_of_roll_buffer].id_measurement, \
							event_measurement[first_element_for_transmit_of_roll_buffer].power);	// TEST
			}
			/* end of TEST */
#endif
			Fill_Tx_buffer(&event_measurement[first_element_for_transmit_of_roll_buffer]);
			Transmit_to_esp32();

			transmit_to_esp32_flag = 0;
			if(++first_element_for_transmit_of_roll_buffer == 100) first_element_for_transmit_of_roll_buffer = 0;
			delay_ms(100);
        }
    }
}

TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }

    pBuffer1++;
    pBuffer2++;
  }

  return PASSED;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  	/* User can add his own implementation to report the file name and line number,
     	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  	/* Infinite loop */
  	while (1)
  	{
  	}
}
#endif

