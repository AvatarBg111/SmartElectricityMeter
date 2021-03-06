#include "esp32_Rx_Tx.h"
#include "stm32f4xx.h"

////////////////////////////////////////////////////////////////////////////////
uint8_t READ_RxSTATUS_FLAG = 0x00;
char tx_data[MAX_BUF_LEN];
char rx_data[MAX_BUF_LEN];

////////////////////////////////////////////////////////////////////////////////
void USART2_IRQHandler(){
    static int index = -1;
    if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET){
        uint16_t buff = USART_ReceiveData(USART2);
        if((char)buff != '~' && !READ_RxSTATUS_FLAG){
            ++index;
            rx_data[index] = (char)buff;
        }else{
          READ_RxSTATUS_FLAG = ~READ_RxSTATUS_FLAG;
          index = -1;
        }
    }else{
        index = -1;
        strcpy(rx_data, "");
//        printf("\nNO DATA");
    }
//    GPIO_SetBits(GPIOC, GPIO_Pin_5);
//    GPIO_ResetBits(GPIOC, GPIO_Pin_5);    
//    USART_ClearFlag(USART2, USART_FLAG_RXNE);
//    USART2->SR &= ~USART_CR1_RXNEIE;
}

void Fill_Tx_buffer(event_measurement_struct_t *tx_struct){
    char num[20]="";
    int offset = 0;
    
    sprintf(tx_data, "~;0;%d;%d;%d;", (int)tx_struct->id_consumer, (int)tx_struct->id_event, (int)tx_struct->id_measurement);    
    for(int i = 0; i < strlen(tx_struct->timestamp_time_start); i++){
      if(i == 4 || i == 6){
        num[i+offset] = '-';
        offset++;
      }else if(i == 8){
        num[i+offset] = ' ';
        offset++;
      }else if(i == 10 || i == 12){
        num[i+offset] = ':';          
        offset++;
      }
      num[i+offset] = tx_struct->timestamp_time_start[i];
    }
    strcat(tx_data, num);
    strcpy(num, "");
    sprintf(num, ";%d;%d;%d", tx_struct->power, 0x66, 0x90);
    strcat(tx_data, num);
    strcpy(num, "");
}

void Transmit_to_esp32(){
    int index = 0;
    while(tx_data[index]){
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
        while(!(USART2->SR & USART_SR_TC));
        USART_SendData(USART2, tx_data[index++]);
    }
    strcpy(tx_data, "");
}

void Receive_from_esp32(){
  READ_RxSTATUS_FLAG = ~READ_RxSTATUS_FLAG;
}