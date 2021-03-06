#include "delay.h"
#include "button.h"
#include <stdbool.h>
#include "mcp39f511.h"
#include "main.h"


////////////////////////////////////////////////////////////////////////////////

uint8_t tx_mode = 0;
uint8_t tx_flag = 0;
MCP_measurement_struct_t tx_measurement;
uint64_t data_of_read64 = 0;
uint8_t flag_of_read64 = 0;
uint8_t flag2_of_read64 = 0;
uint8_t UART1_RX_Data[35];

struct {
    uint8_t response;
    uint8_t valid_response;
    uint8_t number_of_bytes;
	uint8_t tx_byte_0;
    uint8_t tx_byte_1;
	uint8_t tx_byte_2;
	uint8_t tx_byte_3;
	uint8_t tx_byte_4;
	uint8_t tx_byte_5;
	uint8_t tx_byte_6;
	uint8_t tx_byte_7;
	uint32_t registerds64_data;
	uint8_t valid_data;
	uint8_t valid_MCP_measurement;
} rx1_data;


////////////////////////////////////////////////////////////////////////////////

void USART1_IRQHandler (void){
	static uint8_t i = 0;
	static uint8_t mode = 0;
	static uint8_t lock_MCP_measurement = 0;
	static uint8_t CHECKSUM = 0;

	GPIO_SetBits(GPIOA, GPIO_Pin_8);	// TEST
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);	// TEST

    if (USART1->SR & USART_CR1_RXNEIE){
	//USART1->SR &= ~USART_CR1_RXNEIE;

		uint8_t n = (uint8_t)USART_ReceiveData(USART1);
		
		if(i == 0){
			CHECKSUM = 0;
			if(n == HEADERBYTE1){
				CHECKSUM += n;
				i++;
				mode = 1;
			} else if(n == RESP_ACK){
				if(rx1_data.valid_response == 64){
					CHECKSUM += n;
					i++;
					mode = 2;
				} else {
					rx1_data.response = RESP_ACK;
					rx1_data.valid_response = 1;
					rx1_data.valid_data = 0;
					i = 0;
					mode = 0;
				}
			} else if(n == RESP_NAK){
				rx1_data.response = RESP_NAK;
				rx1_data.valid_response = 1;
				rx1_data.valid_data = 0;
				i = 0;
				mode = 0;
			} else if(n == RESP_CSFAIL){
				rx1_data.response = RESP_CSFAIL;
				rx1_data.valid_response = 1;
				rx1_data.valid_data = 0;
				i = 0;
				mode = 0;
			} else {
				i = 0;
				mode = 0;
			}
		} else if(i == 1){
			if(n == HEADERBYTE2 || mode == 1){
				CHECKSUM += n;
				i++;
			} else if(mode == 2){
				rx1_data.number_of_bytes = n;
				CHECKSUM += n;
				i++;
			} else {
				i = 0;
				mode = 0;
			}
		} else if(i == 2){
			if(n == HEADERBYTE3 && mode == 1){
				CHECKSUM += n;
				i++;
			} else if(mode == 2){
				CHECKSUM += n;
				rx1_data.tx_byte_0 = n;
				i++;
			} else {
				i = 0;
				mode = 0;
			}
		} else if(i == 3){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(rx1_data.valid_MCP_measurement == 1){
					lock_MCP_measurement = 1;
				} else {
					lock_MCP_measurement = 0;
					MCP_measurement.CURRENT_RMS = ((uint32_t)n & 0x000000FF);
				}
			} else if(mode == 2){
				rx1_data.tx_byte_1 = n;
			} else {
			}
		} else if(i == 4){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.CURRENT_RMS |= (((uint32_t)n << 8) & 0x0000FF00);
			} else if(mode == 2){
				rx1_data.tx_byte_2 = n;
			} else {
			}
		} else if(i == 5){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.CURRENT_RMS |= (((uint32_t)n << 16) & 0x00FF0000);
			} else if(mode == 2){
				rx1_data.tx_byte_3 = n;
			} else {
			}
		} else if(i == 6){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.CURRENT_RMS |= (((uint32_t)n << 24) & 0xFF000000);
			} else if(mode == 2){
				rx1_data.tx_byte_4 = n;
			} else {
			}
		} else if(i == 7){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.VOLTAGE_RMS = ((uint16_t)n & 0x00FF);
			} else if(mode == 2){
				rx1_data.tx_byte_5 = n;
			} else {
			}
		} else if(i == 8){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.VOLTAGE_RMS |= (((uint16_t)n << 8) & 0xFF00);
			} else if(mode == 2){
				rx1_data.tx_byte_6 = n;
			} else {
			}
		} else if(i == 9){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.ACTIVE_POWER = ((uint32_t)n & 0x000000FF);
			} else if(mode == 2){
				rx1_data.tx_byte_7 = n;
			} else {
			}
		} else if(i == 10){
			if(mode == 1){
				CHECKSUM += n;
				i++;
				if(!lock_MCP_measurement) MCP_measurement.ACTIVE_POWER |= (((uint32_t)n << 8) & 0x0000FF00);
			} else if(mode == 2){
				i = 0;
				rx1_data.valid_response = 1;
				if(n == CHECKSUM){
					rx1_data.registerds64_data = ((uint32_t)rx1_data.tx_byte_0 & 0x000000FF);
					rx1_data.registerds64_data |= (((uint32_t)rx1_data.tx_byte_1 << 8) & 0x0000FF00);
					rx1_data.registerds64_data |= (((uint32_t)rx1_data.tx_byte_2 << 16) & 0x00FF0000);
					rx1_data.registerds64_data |= (((uint32_t)rx1_data.tx_byte_3 << 24) & 0xFF000000);
					//rx1_data.registerds64_data |= (((uint64_t)n << 32) & 0x000000FF00000000);
					//rx1_data.registerds64_data |= (((uint64_t)n << 40) & 0x0000FF0000000000);
					//rx1_data.registerds64_data |= (((uint64_t)n << 48) & 0x00FF000000000000);
					//rx1_data.registerds64_data |= (((uint64_t)n << 56) & 0xFF00000000000000);
					rx1_data.valid_data = 1;
				} else {
					rx1_data.valid_data = 0xFF;
					rx1_data.registerds64_data = 0;
				}
			} else {
			}
		} else if(i == 11){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.ACTIVE_POWER |= (((uint32_t)n << 16) & 0x00FF0000);
			} else {
			}
		} else if(i == 12){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.ACTIVE_POWER |= (((uint32_t)n << 24) & 0xFF000000);
			} else {
			}
		} else if(i == 13){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.REACTIVE_POWER = ((uint32_t)n & 0x000000FF);
			} else {
			}
		} else if(i == 14){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.REACTIVE_POWER |= (((uint32_t)n << 8) & 0x0000FF00);
			} else {
			}
		} else if(i == 15){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.REACTIVE_POWER |= (((uint32_t)n << 16) & 0x00FF0000);
			} else {
			}
		} else if(i == 16){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.REACTIVE_POWER |= (((uint32_t)n << 24) & 0xFF000000);
			} else {
			}
		} else if(i == 17){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.LINE_FREQUENCY = ((uint16_t)n & 0x00FF);
			} else {
			}
		} else if(i == 18){
			CHECKSUM += n;
			i++;
			if(mode == 1){
				if(!lock_MCP_measurement) MCP_measurement.LINE_FREQUENCY |= (((uint16_t)n << 8) & 0xFF00);
			} else {
			}
		} else if(i == 19){
			if(mode == 1){
				i = 0;
				//if(rx1_data.valid_response != 64) rx1_data.valid_response = 0;
				if(n == CHECKSUM){	// OK
					rx1_data.valid_MCP_measurement = 1;
					lock_MCP_measurement = 0;
				} else {
					// ERR Single-wire transmission frame
					printf("Rx1_ERR_CHECKSUM: %0x\n", CHECKSUM);	// TEST
				}
			} else {
			}
		} else {
		}
    }

    GPIO_SetBits(GPIOA, GPIO_Pin_8);	// TEST
    GPIO_ResetBits(GPIOA, GPIO_Pin_8);	// TEST
}


void read_registerds(uint16_t address, uint8_t Number_of_Bytes_to_Read){
  	uint8_t CHECKSUM = FRAME_HEADER + 0x08 + MCP_CMD_SET_ADDRESS_POINTER + \
		address + MCP_CMD_REGISTER_READ + Number_of_Bytes_to_Read;

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, FRAME_HEADER);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x08);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_SET_ADDRESS_POINTER);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x00);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, (uint8_t)address);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_REGISTER_READ);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, Number_of_Bytes_to_Read);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, CHECKSUM);
    while (!(USART1->SR & USART_SR_TC));
}

/* cannot be called from an interrupt */
void read_registerds64(uint16_t address){
  	uint8_t CHECKSUM = FRAME_HEADER + 0x08 + MCP_CMD_SET_ADDRESS_POINTER + \
		address + MCP_CMD_REGISTER_READ + 0x08;

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, FRAME_HEADER);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x08);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_SET_ADDRESS_POINTER);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x00);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, (uint8_t)address);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_REGISTER_READ);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x08);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, CHECKSUM);
    while (!(USART1->SR & USART_SR_TC));
}

void write_registerd16(uint16_t address, int16_t data){
  	uint8_t CHECKSUM;

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, FRAME_HEADER);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x0A);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_SET_ADDRESS_POINTER);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x00);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, (uint8_t)address);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_REGISTER_WRITE);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x02);
    while (!(USART1->SR & USART_SR_TC));
	uint8_t d = (uint8_t)(data & 0x00FF);
	CHECKSUM = FRAME_HEADER + 0x0A + MCP_CMD_SET_ADDRESS_POINTER + (uint8_t)address + MCP_CMD_REGISTER_WRITE + 0x02 + d;
    USART_SendData(USART1, d);
    while (!(USART1->SR & USART_SR_TC));
	d = (uint8_t)((data >> 8) & 0x00FF);
	CHECKSUM = CHECKSUM + d;
    USART_SendData(USART1, d);
    while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, CHECKSUM);
}

void write_registerd32(uint16_t address, int32_t data){
  	uint8_t CHECKSUM;
  	uint32_t registerd;

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, FRAME_HEADER);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x0C);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_SET_ADDRESS_POINTER);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x00);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, (uint8_t)address);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_REGISTER_WRITE);
 
	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x04);

	while (!(USART1->SR & USART_SR_TC));
	registerd = data;
	uint8_t d = (uint8_t)(registerd & 0x000000FF);
	CHECKSUM = FRAME_HEADER + 0x0C + MCP_CMD_SET_ADDRESS_POINTER + (uint8_t)address + MCP_CMD_REGISTER_WRITE + 0x04 + d;
    USART_SendData(USART1, d);

	while (!(USART1->SR & USART_SR_TC));
	registerd = data;
	d = (uint8_t)((registerd >> 8) & 0x000000FF);
	CHECKSUM = CHECKSUM + d;
    USART_SendData(USART1, d);

	while (!(USART1->SR & USART_SR_TC));
	registerd = data;
	d = (uint8_t)((registerd >> 16) & 0x000000FF);
	CHECKSUM = CHECKSUM + d;
    USART_SendData(USART1, d);

	while (!(USART1->SR & USART_SR_TC));
	registerd = data;
	d = (uint8_t)((registerd >> 24) & 0x000000FF);
	CHECKSUM = CHECKSUM + d;
    USART_SendData(USART1, d);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, CHECKSUM);
}

void Save_Registers_To_Flash(){

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, FRAME_HEADER);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x04);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_SAVE_REGISTERS_TO_FLASH);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0xFC);
}

void Auto_Calibrate_Gain(){

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, FRAME_HEADER);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x04);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, MCP_CMD_AUTO_CALIBRATE_GAIN);

	while (!(USART1->SR & USART_SR_TC));
    USART_SendData(USART1, 0x03);
}

void Get_Single_Wire_Rx_Frame(uint8_t init){
	static RTC_TimeTypeDef start_time;
	static RTC_DateTypeDef start_date;
	static uint8_t the_same_CURRENT_RMS;
	static uint8_t wait_for_response = 0;

	/********************************************************************************/
	/* The code below is responsible for event recognition for electrical consumers */
	/********************************************************************************/
	if(event_control.guard_skip_measurements == 0xA5){
	} else if(event_control.guard_skip_measurements){
		if(rx1_data.valid_MCP_measurement){
			rx1_data.valid_MCP_measurement = 0;
			event_control.guard_skip_measurements--;
		}
	} else {
	}

	if(init){
		wait_for_response = 0;

		rx1_data.valid_data = 0;
		rx1_data.valid_response = 0;

		event_control.event_present = 0;
		event_control.has_a_shot = 0;
		event_control.id_consumer = 0;
		event_control.event_created = 0;
		event_control.guard_skip_measurements = 0xA5;
		event_control.read_registerds64_flag = 0;
	} else if(wait_for_response){
		if(rx1_data.valid_response ==1){
			rx1_data.valid_response = 0;
			wait_for_response--;
    		GPIO_SetBits(GPIOA, GPIO_Pin_8);	// TEST
			if(rx1_data.response != RESP_ACK) printf("Tx1_ERR_response: %0x\n", rx1_data.response);	// TEST
   			GPIO_ResetBits(GPIOA, GPIO_Pin_8);	// TEST
			
			if(wait_for_response == 1) write_registerd16(MCP_COMP_PERIPH_ENERGY_CONTROL, 0x0001);	// Energy Control enable
		}
	} else if(event_control.read_registerds64_flag && !wait_for_response){
		if(!rx1_data.valid_response){
			rx1_data.valid_response = 64;
			read_registerds64(MCP_RECORD_IMP_ACTIVE_ENERGY_COUNTER);
		} else if(rx1_data.valid_response == 1){
			event_control.read_registerds64_flag = 0;
			if(rx1_data.valid_data != 1){
				rx1_data.valid_response = 0;	// For ERR rx1_data.valid_response == 0x40 (!?)
				rx1_data.valid_data = 0;
			}
		}
	} else if(!wait_for_response){
		if(rx1_data.valid_MCP_measurement){
			if(MCP_measurement.CURRENT_RMS > CURRENT_RMS_OFFSET_VALUE){
				/* start an event */
				if(!event_control.event_present){
					event_control.event_present = 1;	// event_measurement started
					first_empty_element_of_shot_measurement = 0;	// Initialize shot_measurement index
					rx1_data.valid_response = 0;
					write_registerd16(MCP_COMP_PERIPH_ENERGY_CONTROL, 0x0001);	// Energy Control enable
					wait_for_response = 1;

					RTC_GetTime(RTC_Format_BIN, &start_time);	// Set event_start time
					event_control.start_time = start_time;
					RTC_GetDate(RTC_Format_BIN, &start_date);	// Set event_start date
					event_control.start_date = start_date;

					shot_measurement[first_empty_element_of_shot_measurement].CURRENT_RMS = MCP_measurement.CURRENT_RMS;
					shot_measurement[first_empty_element_of_shot_measurement].VOLTAGE_RMS = MCP_measurement.VOLTAGE_RMS;
					shot_measurement[first_empty_element_of_shot_measurement].ACTIVE_POWER = MCP_measurement.ACTIVE_POWER;
					shot_measurement[first_empty_element_of_shot_measurement].REACTIVE_POWER = MCP_measurement.REACTIVE_POWER;
					shot_measurement[first_empty_element_of_shot_measurement].LINE_FREQUENCY = MCP_measurement.LINE_FREQUENCY;
					first_empty_element_of_shot_measurement++;
					
					rx1_data.valid_MCP_measurement = 0;
				} else {
					/* event is started; first take a snapshot of the inclusion (50 point in 1 sec, N = 0) */
					if(!event_control.has_a_shot && !event_control.id_consumer){
						shot_measurement[first_empty_element_of_shot_measurement].CURRENT_RMS = MCP_measurement.CURRENT_RMS;
						shot_measurement[first_empty_element_of_shot_measurement].VOLTAGE_RMS = MCP_measurement.VOLTAGE_RMS;
						shot_measurement[first_empty_element_of_shot_measurement].ACTIVE_POWER = MCP_measurement.ACTIVE_POWER;
						shot_measurement[first_empty_element_of_shot_measurement].REACTIVE_POWER = MCP_measurement.REACTIVE_POWER;
						shot_measurement[first_empty_element_of_shot_measurement].LINE_FREQUENCY = MCP_measurement.LINE_FREQUENCY;
						if(++first_empty_element_of_shot_measurement == SHOT_MAX_NUMBER){
							first_empty_element_of_shot_measurement = 0;
							event_control.has_a_shot = 1;
									
							event_control.CURRENT_RMS = MCP_measurement.CURRENT_RMS;
							event_control.ACTIVE_POWER = MCP_measurement.ACTIVE_POWER;
							event_control.REACTIVE_POWER = MCP_measurement.REACTIVE_POWER;
									
							event_measurement[first_empty_element_of_roll_buffer].power = MCP_measurement.ACTIVE_POWER;

							/* Set	event_measurement[first_empty_element_of_roll_buffer].timestamp_time_start */
							sprintf(event_measurement[first_empty_element_of_roll_buffer].timestamp_time_start, \
										"20%02d%02d%02d%02d%02d%02d", \
											start_date.RTC_Year, start_date.RTC_Month, start_date.RTC_Date, \
												start_time.RTC_Hours, start_time.RTC_Minutes, start_time.RTC_Seconds);
						}
						rx1_data.valid_MCP_measurement = 0;
					/* event is started, the shooting is done; measurement arrives...*/
					} else {
						/* Is the newly measured current within the limits of digitalization? */
						uint32_t a = MCP_measurement.CURRENT_RMS;
						uint32_t b = event_control.CURRENT_RMS;
						float diff = (float)a / b;
						diff *= 100;
						diff -= 100;
						if(diff < 0)  diff = -diff;
						uint32_t procent = DIGITALIZATION_FACTOR;
						if(diff < procent){
							the_same_CURRENT_RMS = 1;
						} else the_same_CURRENT_RMS = 0;

						/* The newly measured current is within the limits of digitalization */
						if(the_same_CURRENT_RMS){
							/* load the structure to send to the ESP32 with the first measurement; ****/
							/* in this structure the "power" field contains the power of the consumer */
							if(!event_control.event_created){
								if(event_control.id_consumer){
									event_control.event_created = 1;
											
									event_measurement[first_empty_element_of_roll_buffer].id_consumer = event_control.id_consumer;
									event_measurement[first_empty_element_of_roll_buffer].id_event = event_control.id_event;
									event_measurement[first_empty_element_of_roll_buffer].id_measurement = event_control.id_measurement;
											
									if(++first_empty_element_of_roll_buffer == 100) first_empty_element_of_roll_buffer = 0;
									if(first_empty_element_of_roll_buffer == first_element_for_transmit_of_roll_buffer){
										if(first_empty_element_of_roll_buffer){
											first_empty_element_of_roll_buffer--;
										} else first_empty_element_of_roll_buffer = 99;
										//ERR roll_buffer
									}
								}
							}
							event_control.guard_skip_measurements = 0xA5;
							rx1_data.valid_MCP_measurement = 0;
						/* The newly measured current isn't within the limits of digitalization. */
						/* Create a new measurement for the event. In this structure, the */
						/*  "power" field contains the calculated power for the previous period */
						} else {
							if(event_control.event_created){
								/* skipping two cycles to establish the current in steady state */
								if(event_control.guard_skip_measurements == 0xA5){
									event_control.guard_skip_measurements = 5;
								} else {
									event_control.CURRENT_RMS = MCP_measurement.CURRENT_RMS;
									event_control.ACTIVE_POWER = MCP_measurement.ACTIVE_POWER;
									event_control.REACTIVE_POWER = MCP_measurement.REACTIVE_POWER;

									RTC_GetTime(RTC_Format_BIN, &start_time);	// Set event_start time
									event_control.start_time = start_time;
									RTC_GetDate(RTC_Format_BIN, &start_date);	// Set event_start date
									event_control.start_date = start_date;
									event_control.id_measurement++;

									event_measurement[first_empty_element_of_roll_buffer].id_consumer = event_control.id_consumer;
									event_measurement[first_empty_element_of_roll_buffer].id_event = event_control.id_event;
									event_measurement[first_empty_element_of_roll_buffer].id_measurement = event_control.id_measurement;
									/* Set	event_measurement[first_empty_element_of_roll_buffer].timestamp_time_start */
									sprintf(event_measurement[first_empty_element_of_roll_buffer].timestamp_time_start, \
											"20%02d%02d%02d%02d%02d%02d", \
												start_date.RTC_Year, start_date.RTC_Month, start_date.RTC_Date, \
													start_time.RTC_Hours, start_time.RTC_Minutes, start_time.RTC_Seconds);

									rx1_data.valid_response = 0;
									write_registerd16(MCP_COMP_PERIPH_ENERGY_CONTROL, 0x0000);	// Energy Control disible
									wait_for_response = 2;	// To send "Energy Control enable"

									if(++first_empty_element_of_roll_buffer == 100) first_empty_element_of_roll_buffer = 0;
									if(first_empty_element_of_roll_buffer == first_element_for_transmit_of_roll_buffer){
										if(first_empty_element_of_roll_buffer){
											first_empty_element_of_roll_buffer--;
										} else first_empty_element_of_roll_buffer = 99;
										// ERR roll_buffer
									}
									rx1_data.valid_MCP_measurement = 0;
								}
							} else {
								// Code for pulsating CURRENT_RMS
							}
						}
					}
				}
			/* The CURRENT_RMS is below the CURRENT_RMS_OFFSET_VALUE (no CURRENT_RMS) */
			} else {
				/* create a final structure with this id_event; reset event_control */
				/* "power" field contains the calculated power for the last period	*/
				if(event_control.event_present){
					if(!(rx1_data.valid_data && rx1_data.valid_response == 1)){
						RTC_GetTime(RTC_Format_BIN, &start_time);	// Set event_start time
						event_control.start_time = start_time;
						RTC_GetDate(RTC_Format_BIN, &start_date);	// Set event_start date
						event_control.start_date = start_date;
						event_control.id_measurement++;

						/* When no shot (less 1 sec) */
						if(!event_control.id_consumer){
							event_control.id_event = 0x0000;
							event_control.id_measurement = 0x00;
						}

						event_measurement[first_empty_element_of_roll_buffer].id_consumer = event_control.id_consumer;
						event_measurement[first_empty_element_of_roll_buffer].id_event = event_control.id_event;
						event_measurement[first_empty_element_of_roll_buffer].id_measurement = event_control.id_measurement;
						/* Set	event_measurement[first_empty_element_of_roll_buffer].timestamp_time_start */
						sprintf(event_measurement[first_empty_element_of_roll_buffer].timestamp_time_start, \
									"20%02d%02d%02d%02d%02d%02d", \
										start_date.RTC_Year, start_date.RTC_Month, start_date.RTC_Date, \
											start_time.RTC_Hours, start_time.RTC_Minutes, start_time.RTC_Seconds);

						rx1_data.valid_response = 0;
						write_registerd16(MCP_COMP_PERIPH_ENERGY_CONTROL, 0x0000);	// Energy Control disible
						wait_for_response = 1;
						event_control.read_registerds64_flag = 1;
					} else {
						event_measurement[first_empty_element_of_roll_buffer].power = rx1_data.registerds64_data;

						if(++first_empty_element_of_roll_buffer == 100) first_empty_element_of_roll_buffer = 0;
						if(first_empty_element_of_roll_buffer == first_element_for_transmit_of_roll_buffer){
							if(first_empty_element_of_roll_buffer){
								first_empty_element_of_roll_buffer--;
							} else first_empty_element_of_roll_buffer = 99;
							// ERR roll_buffer
						}

						rx1_data.valid_data = 0;
						rx1_data.valid_response = 0;
						event_control.event_present = 0;
						event_control.has_a_shot = 0;
						event_control.id_consumer = 0;
						event_control.event_created = 0;
						event_control.guard_skip_measurements = 0xA5;
						event_control.read_registerds64_flag = 0;
						
						rx1_data.valid_MCP_measurement = 0;
					}
				} else {
					rx1_data.valid_MCP_measurement = 0;
				}
			}
		}
	}
}


void Init_Mcp39f511(){
	bool flag_calibration = 0;

	/* Read BUTTON after reset for the single-point CALIBRATION */
	uint8_t r = readBtnSetupMatrix();	// initialize readBtnSetupMatrix()
	while(!wait_ms_ch(channelButton, guard_time + 1));	// waiting for readiness
	if(readBtnSetupMatrix() == BUTTON1){
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);	// Led on if the button is pressed
		while(readBtnSetupMatrix() == BUTTON1);	// Wait while the button is released
		while(readBtnSetupMatrix() != BUTTON1){	// Wait while the button is pressed
			if(wait_ms_ch(channelLed, 1000)) GPIO_ToggleBits(GPIOA, GPIO_Pin_1);	// the LED flashes
		}
		flag_calibration = true;	// to perform calibration
		GPIO_SetBits(GPIOA, GPIO_Pin_1);	// Led off & exit
	} else {
		GPIO_SetBits(GPIOA, GPIO_Pin_1);	// else led off
	}

	write_registerd32(MCP_CONFIG_1_SYSTEM_CONFIG, 0x03000000);	// Single-wire transmission off
	(void) wait_ms_ch(channelTime2, 1);	// Specific for while() only
	while(!wait_ms_ch(channelTime2, 100));

	/* Only for FIRST (after factory) inicialization for MCP39F511 */
#if(0)
	write_registerd32(MCP_CONFIG_1_RANGE, 0x1B120B12);	// Set Range
	while(!wait_ms_ch(channelTime2, 10));

	write_registerd32(MCP_CONFIG_1_SYSTEM_CONFIG, 0x03000000);	// Single-wire transmission off
	while(!wait_ms_ch(channelTime2, 10));

	write_registerd16(MCP_CALIB_GAIN_VOLTAGE_RMS, 0xB554);	// Set GANE V
	while(!wait_ms_ch(channelTime2, 10));

	write_registerd16(MCP_CALIB_GAIN_ACTIVE_POWER, 0x88A9);	// Set GANE Pa
	while(!wait_ms_ch(channelTime2, 10));

	//write_registerd32(MCP_CALIB_OFFSET_CURRENT_RMS, 0x00000006);	// Set OFFSET I
	//while(!wait_ms_ch(channelTime2, 10));

	//write_registerd32(MCP_CALIB_OFFSET_ACTIVE_POWER, 0x00000000);	// Set OFFSET Pa
	//while(!wait_ms_ch(channelTime2, 10));

	//write_registerd32(MCP_CALIB_OFFSET_REACTIVE_POWER, 0x00000000);	// Set OFFSET Pr
	//while(!wait_ms_ch(channelTime2, 10));

	Save_Registers_To_Flash();	// Save all registers
	while(!wait_ms_ch(channelTime2, 100));
#endif

	/* Calibration subroutine */
	if(flag_calibration){
		write_registerd32(MCP_CONFIG_1_CALIB_CURRENT, CALIB_CURRENT_VALUE);
		while(!wait_ms_ch(channelTime2, 10));

		write_registerd16(MCP_CONFIG_1_CALIB_VOLTAGE, CALIB_VOLTAGE_VALUE);
		while(!wait_ms_ch(channelTime2, 10));

		write_registerd32(MCP_CONFIG_1_CALIB_POWER_ACTIVE, CALIB_POWER_ACTIVE_VALUE);
		while(!wait_ms_ch(channelTime2, 10));

		Auto_Calibrate_Gain();
		while(!wait_ms_ch(channelTime2, 100));
	}

	/* Only if we want to read some registers of MCP39F511 */
#if(0)
	read_registerds(MCP_CONFIG_1_RANGE, 4);
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_OUTPUT_REG_CURRENT_RMS, 4);		// CURRENT_RMS value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_OUTPUT_REG_VOLTAGE_RMS, 2);		// VOLTAGE_RMS value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_OUTPUT_REG_ACTIVE_POWER, 4);	// ACTIVE_POWER value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_CALIB_GAIN_CURRENT_RMS, 2);		// GANE I value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_CALIB_GAIN_VOLTAGE_RMS, 2);		// GANE V value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_CALIB_GAIN_ACTIVE_POWER, 2);	// GANE Pa value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_CALIB_OFFSET_CURRENT_RMS, 4);	// OFFSET CURRENT_RMS value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_CALIB_OFFSET_ACTIVE_POWER, 4);	// OFFSET ACTIVE_POWER value
	while(!wait_ms_ch(channelTime2, 10));
	read_registerds(MCP_CALIB_OFFSET_REACTIVE_POWER, 4);// OFFSET REACTIVE_POWER value
	while(!wait_ms_ch(channelTime2, 100));
#endif

	write_registerd16(MCP_CONFIG_2_ACCUMULATION_INTERVAL, 0x0000);	// N number = 0.	The accumulation interval is defined as
	while(!wait_ms_ch(channelTime2, 10));							//							   an 2^N number of line cycles

	write_registerd32(MCP_CONFIG_1_SYSTEM_CONFIG, 0x03000100);	// Single-wire transmission on
	while(!wait_ms_ch(channelTime2, 10));
}