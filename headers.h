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

/*******************************************************************************
 *                              STD types                                   *
 */

/* Boolean Values */
// #ifndef FALSE
// #define FALSE       (0u)
// #endif
// #ifndef TRUE
// #define TRUE        (1u)
// #endif

#define LOGIC_HIGH        (1u)
#define LOGIC_LOW         (0u)

// #define NULL_PTR    ((void*)0)

typedef unsigned char         uint8;          /*           0 .. 255              */
// typedef signed char           sint8;          /*        -128 .. +127             */
typedef unsigned short        uint16;         /*           0 .. 65535            */
// typedef signed short          sint16;         /*      -32768 .. +32767           */
typedef unsigned long         uint32;         /*           0 .. 4294967295       */
// typedef signed long           sint32;         /* -2147483648 .. +2147483647      */
// typedef float                 float32;


/*******************************************************************************
 *                         Types Declaration                                   *
 *******************************************************************************/
// khodam 
unsigned char temporaryString[16];

// dcmotor
typedef enum
{
	STOP,CLOCKWISE,ANTI_CLOCKWISE
}DcMotor_State;

// gpio
typedef enum
{
	PIN_INPUT,PIN_OUTPUT
}GPIO_PinDirectionType;

typedef enum
{
	PORT_INPUT,PORT_OUTPUT=0xFF
}GPIO_PortDirectionType;


/*******************************************************************************
 *                                Definitions                                  *
 *******************************************************************************/
// ADC
#define ADC_MAXIMUM_VALUE    1023
// #define ADC_REF_VOLT_VALUE   (2.56)
#define ADC_REF_VOLT_VALUE   (5)
// lm35 sensor
#define SENSOR_CHANNEL_ID_LM35    2
#define SENSOR_MAX_VOLT_VALUE     1.5
#define SENSOR_MAX_TEMPERATURE    150

// dcmotor
#define DcMotor_Pin0_PORT_ID           PORTB_ID
#define DcMotor_Pin0_PIN_ID            PIN0_ID

#define DcMotor_Pin1_PORT_ID           PORTB_ID
#define DcMotor_Pin1_PIN_ID            PIN1_ID

// common macros
/* Set a certain bit in any register */
#define SET_BIT(REG,BIT) (REG|=(1<<BIT))

/* Clear a certain bit in any register */
#define CLEAR_BIT(REG,BIT) (REG&=(~(1<<BIT)))

/* Toggle a certain bit in any register */
#define TOGGLE_BIT(REG,BIT) (REG^=(1<<BIT))

/* Rotate right the register value with specific number of rotates */
#define ROR(REG,num) ( REG= (REG>>num) | (REG<<(8-num)) )

/* Rotate left the register value with specific number of rotates */
#define ROL(REG,num) ( REG= (REG<<num) | (REG>>(8-num)) )

/* Check if a specific bit is set in any register and return true if yes */
#define BIT_IS_SET(REG,BIT) ( REG & (1<<BIT) )

/* Check if a specific bit is cleared in any register and return true if yes */


#define GET_BIT(REG,BIT) ( ( REG & (1<<BIT) ) >> BIT )

// gpio
#define NUM_OF_PORTS           4
#define NUM_OF_PINS_PER_PORT   8

#define PORTA_ID               0
#define PORTB_ID               1
#define PORTC_ID               2
#define PORTD_ID               3

#define PIN0_ID                0
#define PIN1_ID                1
#define PIN2_ID                2
#define PIN3_ID                3
#define PIN4_ID                4
#define PIN5_ID                5
#define PIN6_ID                6
#define PIN7_ID                7


// pwm
#define PWM_PORT_ID           PORTB_ID
#define PWM_PIN_ID            PIN3_ID

#define TOP 255

/*******************************************************************************
 *                      Functions Prototypes                                   *
 *******************************************************************************/

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
uint16 ADC_readChannel(uint8 channel_num);

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
void DcMotor_Rotate(DcMotor_State state,uint8 speed);

// gpio
/*
 * Description :
 * Setup the direction of the required pin input/output.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setupPinDirection(uint8 port_num, uint8 pin_num, GPIO_PinDirectionType direction);

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * If the input port number or pin number are not correct, The function will not handle the request.
 * If the pin is input, this function will enable/disable the internal pull-up resistor.
 */
void GPIO_writePin(uint8 port_num, uint8 pin_num, uint8 value);



// lm35 sensor
/*
 * Description :
 * Function responsible for calculate the temperature from the ADC digital value.
 */
uint8 LM35_getTemperature(void);

// pwm
/*
 * Description :
 * The function responsible for trigger the Timer0 with the PWM Mode.
 */
void PWM_Timer0_Start(uint8 duty_cycle);


#endif /* HEADERS_H_ */
