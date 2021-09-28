/*
    Como se programa un seria en Arduino DUE
    Parece que hay un serial que no se utiliza en la plataforma Arduino
*/
// Use the Arduino core to set-up the unused USART2 on Serial4
RingBuffer rx_buffer5;
RingBuffer tx_buffer5;
USARTClass Serial4(USART2, USART2_IRQn, ID_USART2, &rx_buffer5, &tx_buffer5);
//void serialEvent4() __attribute__((weak));
//void serialEvent4() { }

void USART2_Handler(void)   // Interrupt handler for UART2
{
  Serial4.IrqHandler();     // In turn calls on the Serial2 interrupt handler
}
/*
    El serial3 no parece funcionar. Es posible que comparta 
    UART con el I2C0 (SCL0, SDA0) 
*/
/*
    El serial RX0 TX0, pines 0,1 es el serial del programing port
*/