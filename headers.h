#ifndef HEADERS_H_
#define HEADERS_H_

#include <io.h>
#include <stdint.h>
#include <mega32.h>
#include <alcd.h>
#include <delay.h>
#include <stdio.h>
#include <string.h>



/*******************************************************************
 * 							SPEAKER  							   *
 * *****************************************************************
*/ 
#define SPEAKER_PIN 1
#define SPEAKER_FREQUENCY 1000
volatile uint8_t speakerToggle = 0;

void PWM1_init();
void playTone();
void stopTone();
// void alertSpeaker();
/*******************************************************************
 * 							DISPLAY  							   *
 * *****************************************************************
*/      

unsigned char displayFlag = 0;

void GAS_display();
void TempDC_display();


/********************************************************************
 * 							GAS SENSORS								*
 * ******************************************************************
 */

#define GAS_SENSOR_CHANNEL_ID    0
int gasData();
void check_gas();
void warning(uint8_t flag);

/********************************************************************
 * 							UART HANDLER									*
 * ******************************************************************
 */

#define BAUD 9600
#define MYUBRR F_CPU / 16 / BAUD - 1
#define _FCPU 8000000UL
#define F_CPU 8000000UL
#define FOSC 8000000 // Clock Speed
#define BAUDRATE ((FOSC)/(BAUD*16UL)-1)
#define BUFFER_SIZE 128
#define pass (void)0

// UART Variables
volatile char uart_buffer[BUFFER_SIZE];
volatile uint8_t uart_index = 0;
volatile uint8_t string_received = 0;
unsigned char temporaryString[16];

// UART Functions
void uart_init();
void uart_transmit(uint8_t data);
void uart_transmit_string(char *str);
void process_received_string(const char *str);
//

/*********************************************************************
 *                         PASSWORD                                  *
 * *******************************************************************
 */

volatile uint8_t passFlag = 0;
char password[4] = "123";

void check_password();


/*********************************************************************
 *                         ADC                                 *
 * *******************************************************************
 */


// dcmotor
typedef enum
{
	STOP,CLOCKWISE,ANTI_CLOCKWISE
}DcMotor_State;

// ADC
#define ADC_MAXIMUM_VALUE    1023
// #define ADC_REF_VOLT_VALUE   (2.56)
#define ADC_REF_VOLT_VALUE   (5)
// lm35 sensor
#define SENSOR_CHANNEL_ID_LM35    2
#define SENSOR_MAX_VOLT_VALUE     1.5
#define SENSOR_MAX_TEMPERATURE    150

/*
 * Description :
 * Function responsible for initialize the ADC driver.
 */
void ADC_init();

/*
 * Description :
 * Function responsible for read analog data from a certain ADC channel
 * and convert it to digital using the ADC driver.
 */
uint16_t ADC_readChannel(uint8_t channel_num);



/*********************************************************************
 *                          Motor                                    *
 * *******************************************************************
 */


// dcmotor
/*
 * Description :
 * Function responsible for setup the direction for the two motor pins and stop at the DC-Motor at the beginning.
 */
void DcMotor_Init(void);
/*
 * Description :
 * Function responsible for rotate the DC Motor CW/ or A-CW or stop the motor based on the state input state value
 * and send the required duty cycle to the PWM driver based on the required speed value.
 */
void DcMotor_Rotate(DcMotor_State state,uint8_t speed);

// lm35 sensor
/*
 * Description :
 * Function responsible for calculate the temperature from the ADC digital value.
 */
uint8_t LM35_getTemperature(void);

// pwm
/*
 * Description :
 * The function responsible for trigger the Timer0 with the PWM Mode.
 */
void PWM_Timer0_Start(uint8_t duty_cycle);






// common macros ---------------------------------
/* Set a certain bit in any register */
#define SET_BIT(REG,BIT) (REG|=(1<<BIT))

/* Clear a certain bit in any register */
#define CLEAR_BIT(REG,BIT) (REG&=(~(1<<BIT)))

/* Toggle a certain bit in any register */
#define TOGGLE_BIT(REG,BIT) (REG^=(1<<BIT))

/* Check if a specific bit is set in any register and return true if yes */
#define BIT_IS_SET(REG,BIT) ( REG & (1<<BIT) )

/* Check if a specific bit is cleared in any register and return true if yes */
#define GET_BIT(REG,BIT) ( ( REG & (1<<BIT) ) >> BIT )
#define TOP 255



#endif /* HEADERS_H_ */
