/*
 * A7.h
 *
 *  Created on: Nov 7, 2024
 *      Author: camil
 */

#ifndef SRC_A7_H_
#define SRC_A7_H_


#define BRR_num 694
#define CALIBRATION_VALUE 0.807
#define CALIBRATION_OFFSET 0.794
#define UPPER_MASK 0x3000
#define CALIBRATION 0.825
#define PC4 7
#define NVIC_REG_MASK 0x1F
#define SAMPLE_SIZE 5000
#define TWOHUNDREDUS 200
#define ONEMICROSEC 1000000


void print_freq(void);
void TIM2_Init(void);
void COMP_init(void);
uint16_t get_average(uint16_t []);
void INT_DAC_init(void);

void SystemClock_Config(void);
void DAC_init(void);
uint16_t DAC_volt_conv(int32_t);
void DAC_write(uint16_t);

void UART_init(void);
void UART_print(char* );
void USART_ESC_Code(char*);

void ADC1_2_IRQHandler(void);
void ADC_init(void);
void ADC_print(int32_t);
int32_t ADC_calibration(int32_t);

#endif /* SRC_A7_H_ */
