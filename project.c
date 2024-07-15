#include "headers.h"



void main(void)
{

	#asm("sei");
	ADC_init();
	lcd_init(16);
	uart_init();
	DcMotor_Init();
	PWM1_init();
    DDRA |= (1 << SPEAKER_PIN);
	DDRD |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7); 

	uart_transmit_string("Enter password : ");

	while (1)
	{

		// while (passFlag != 1);
		
		
		delay_ms(50);
		lcd_clear();
		
		check_gas();
		
		if (string_received)
		{
			string_received = 0;
			process_received_string(uart_buffer);
		}

		if(displayFlag == 0){
			TempDC_display();
		}else if(displayFlag == 1){
			GAS_display();
		}
		

	}
}

void check_gas(){
	uint16 gas_data = gasData();

	if(gas_data >= 600){warning(1);}
	else if(gas_data < 600){warning(0);}
}

void warning(uint8_t flag){
	if(flag == 1){
		playTone();
	}else if(flag == 0){
		stopTone();
	}

}

void GAS_display(){
	uint32 Gas_raw_data = gasData();

	lcd_gotoxy(0, 0);
	lcd_puts("-GAS Sensor-");
	lcd_gotoxy(0,1);
	sprintf(temporaryString , "Values: %d" , Gas_raw_data );
	lcd_puts(temporaryString);
}

void TempDC_display()
{
	uint8 temperature;

	temperature = LM35_getTemperature();
	switch (temperature / 10)
	{
	case 0: // 0-9
		DcMotor_Rotate(STOP, 0);
		sprintf(temporaryString, "    Fan is OFF");
		break;
	case 1: // 10-19
		DcMotor_Rotate(STOP, 0);
		sprintf(temporaryString, "    Fan is OFF");
		break;
	case 2: // 20-29
		DcMotor_Rotate(CLOCKWISE, 10);
		sprintf(temporaryString, "    Fan is ON");
		break;
	case 3: // 30-39
		DcMotor_Rotate(CLOCKWISE, 25);
		sprintf(temporaryString, "    Fan is ON");
		break;
	case 4: // 40-49
		DcMotor_Rotate(CLOCKWISE, 50);
		sprintf(temporaryString, "    Fan is ON");
		break;
	case 5: // 50-59
		DcMotor_Rotate(CLOCKWISE, 75);
		sprintf(temporaryString, "    Fan is ON");
		break;
	case 6: // 60-69
		DcMotor_Rotate(CLOCKWISE, 75);
		sprintf(temporaryString, "    Fan is ON");
		break;
	default: // 70 >=
		DcMotor_Rotate(CLOCKWISE, 100);
		sprintf(temporaryString, "    Fan is ON");
		break;
	}

	lcd_gotoxy(0, 0);
	lcd_puts(temporaryString);
	lcd_gotoxy(0, 1);
	sprintf(temporaryString, "   Temp = %d C", temperature);
	lcd_puts(temporaryString);
}

//                                     ADC
void ADC_init()
{
	ADMUX = 0b01001111;
	ADCSRA = 0b10000000;
}

uint16 ADC_readChannel(uint8 channel_num)
{
	ADMUX = (ADMUX & 0b11100000) | (channel_num);
	SET_BIT(ADCSRA, ADSC);
	while ((ADCSRA & (LOGIC_HIGH << ADIF)) == 0);
	SET_BIT(ADCSRA, ADIF);
	return ADCW;
}

//                                     LM35
uint8 LM35_getTemperature(void)
{
	uint8 temp_value = 0;
	uint16 adc_value = 0;
	/* Read ADC channel where the temperature sensor is connected */
	adc_value = ADC_readChannel(SENSOR_CHANNEL_ID_LM35);
	/* Calculate the temperature from the ADC value*/
	temp_value = (uint8)(((uint32)adc_value * SENSOR_MAX_TEMPERATURE * ADC_REF_VOLT_VALUE) / (ADC_MAXIMUM_VALUE * SENSOR_MAX_VOLT_VALUE));
	return temp_value;
}




// dcmotor
void DcMotor_Init(void)
{
	/* DC-Motor 2 output pins */
	GPIO_setupPinDirection(DcMotor_Pin0_PORT_ID, DcMotor_Pin0_PIN_ID,
						   PIN_OUTPUT);
	GPIO_setupPinDirection(DcMotor_Pin1_PORT_ID, DcMotor_Pin1_PIN_ID,
						   PIN_OUTPUT);

	/* Stop the DC-Motor at the beginning */
	GPIO_writePin(DcMotor_Pin0_PORT_ID, DcMotor_Pin0_PIN_ID, LOGIC_LOW);
	GPIO_writePin(DcMotor_Pin1_PORT_ID, DcMotor_Pin1_PIN_ID, LOGIC_LOW);
}

/*
 * Description :
 * Function responsible for rotate the DC Motor CW/ or A-CW or stop the motor based on the state input state value
 * and send the required duty cycle to the PWM driver based on the required speed value.
 */
void DcMotor_Rotate(DcMotor_State state, uint8 speed)
{
	uint8 duty_cycle;

	/*
	 * Write on the DC-Motor pins
	 * Set direction of motor based on state-
	 * */

	switch (state)
	{
	case CLOCKWISE:
		// Rotate motor clockwise
		GPIO_writePin(DcMotor_Pin0_PORT_ID, DcMotor_Pin0_PIN_ID, LOGIC_HIGH);
		GPIO_writePin(DcMotor_Pin1_PORT_ID, DcMotor_Pin1_PIN_ID, LOGIC_LOW);
		break;
	case ANTI_CLOCKWISE:
		// Rotate motor anti-clockwise
		GPIO_writePin(DcMotor_Pin0_PORT_ID, DcMotor_Pin0_PIN_ID, LOGIC_LOW);
		GPIO_writePin(DcMotor_Pin1_PORT_ID, DcMotor_Pin1_PIN_ID, LOGIC_HIGH);
		break;
	case STOP:
		// Stop the motor
		GPIO_writePin(DcMotor_Pin0_PORT_ID, DcMotor_Pin0_PIN_ID, LOGIC_LOW);
		GPIO_writePin(DcMotor_Pin1_PORT_ID, DcMotor_Pin1_PIN_ID, LOGIC_LOW);
		break;
	default:
		// Invalid motor state
		return;
	}
	//	/* Write on the DC-Motor pins */
	//	GPIO_writePin(PORTB_ID, PIN0_ID, (state & 0x01));
	//	GPIO_writePin(PORTB_ID, PIN1_ID, ((state >> 1) & 0x01));
	/*Calculate the Duty Cycle and send it to the PWM Driver*/
	duty_cycle = ((uint8)(((uint16)(speed * TOP)) / 100));
	PWM_Timer0_Start(duty_cycle);
}

// gpio
/*
 * Description :
 * Setup the direction of the required pin input/output.
 * If the input port number or pin number are not correct, The function will not handle the request.
 */
void GPIO_setupPinDirection(uint8 port_num, uint8 pin_num, GPIO_PinDirectionType direction)
{
	/*
	 * Check if the input port number is greater than NUM_OF_PINS_PER_PORT value.
	 * Or if the input pin number is greater than NUM_OF_PINS_PER_PORT value.
	 * In this case the input is not valid port/pin number
	 */
	if ((pin_num >= NUM_OF_PINS_PER_PORT) || (port_num >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}
	else
	{
		/* Setup the pin direction as required */
		switch (port_num)
		{
		case PORTA_ID:
			if (direction == PIN_OUTPUT)
			{
				SET_BIT(DDRA, pin_num);
			}
			else
			{
				CLEAR_BIT(DDRA, pin_num);
			}
			break;
		case PORTB_ID:
			if (direction == PIN_OUTPUT)
			{
				SET_BIT(DDRB, pin_num);
			}
			else
			{
				CLEAR_BIT(DDRB, pin_num);
			}
			break;
		case PORTC_ID:
			if (direction == PIN_OUTPUT)
			{
				SET_BIT(DDRC, pin_num);
			}
			else
			{
				CLEAR_BIT(DDRC, pin_num);
			}
			break;
		case PORTD_ID:
			if (direction == PIN_OUTPUT)
			{
				SET_BIT(DDRD, pin_num);
			}
			else
			{
				CLEAR_BIT(DDRD, pin_num);
			}
			break;
		}
	}
}

/*
 * Description :
 * Write the value Logic High or Logic Low on the required pin.
 * If the input port number or pin number are not correct, The function will not handle the request.
 * If the pin is input, this function will enable/disable the internal pull-up resistor.
 */
void GPIO_writePin(uint8 port_num, uint8 pin_num, uint8 value)
{
	/*
	 * Check if the input port number is greater than NUM_OF_PINS_PER_PORT value.
	 * Or if the input pin number is greater than NUM_OF_PINS_PER_PORT value.
	 * In this case the input is not valid port/pin number
	 */
	if ((pin_num >= NUM_OF_PINS_PER_PORT) || (port_num >= NUM_OF_PORTS))
	{
		/* Do Nothing */
	}
	else
	{
		/* Write the pin value as required */
		switch (port_num)
		{
		case PORTA_ID:
			if (value == LOGIC_HIGH)
			{
				SET_BIT(PORTA, pin_num);
			}
			else
			{
				CLEAR_BIT(PORTA, pin_num);
			}
			break;
		case PORTB_ID:
			if (value == LOGIC_HIGH)
			{
				SET_BIT(PORTB, pin_num);
			}
			else
			{
				CLEAR_BIT(PORTB, pin_num);
			}
			break;
		case PORTC_ID:
			if (value == LOGIC_HIGH)
			{
				SET_BIT(PORTC, pin_num);
			}
			else
			{
				CLEAR_BIT(PORTC, pin_num);
			}
			break;
		case PORTD_ID:
			if (value == LOGIC_HIGH)
			{
				SET_BIT(PORTD, pin_num);
			}
			else
			{
				CLEAR_BIT(PORTD, pin_num);
			}
			break;
		}
	}
}


// pwm
void PWM_Timer0_Start(uint8 duty_cycle)
{

	TCNT0 = 0;		   /* Set Timer Initial Value */
	OCR0 = duty_cycle; /* Set Compare Value */
	/* set PB3/OC0 as output pin --> pin where the PWM signal is generated from MC */
	GPIO_setupPinDirection(PWM_PORT_ID, PWM_PIN_ID, PIN_OUTPUT);
	/*
	 * Configure Timer0 Control Register:
	 * 1. Fast PWM mode FOC0=0
	 * 2. Fast PWM Mode WGM01=1 & WGM00=1
	 * 3. Clear OC0 when match occurs (non inverted mode) COM00=0 & COM01=1
	 * 4. clock = F_CPU/8 CS00=0 CS01=1 CS02=0
	 */
	TCCR0 = (1 << WGM00) | (1 << WGM01) | (1 << COM01) | (1 << CS01);
}

// UART
void uart_init()
{
	// Set the communication Baud Rate
	UBRRH = (BAUDRATE >> 8);
	UBRRL = BAUDRATE;
	// Enable Receiver, Transmitter, and RX Complete Interrupt
	UCSRB |= (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	// Select 8-bit data format
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

interrupt[USART_RXC] void uart_receive()
{
	char received_char = UDR;
	char temp[6];

	if (!string_received)
	{
		if (received_char != 13 && uart_index < BUFFER_SIZE - 1)
		{
			uart_buffer[uart_index++] = received_char;
			sprintf(temp, "%c", received_char);
			uart_transmit_string(temp);
		}
		else
		{
			uart_buffer[uart_index] = '\0';
			uart_index = 0;
			string_received = 1;
			if (passFlag == 0)
			{
				check_password();
			}
		}
	}
}

void uart_transmit(uint8_t data)
{
	// wait until the data register is empty
	while (!(UCSRA & (1 << UDRE)))
		;
	// set the data to the data Register
	UDR = data;
}

void uart_transmit_string(char *str)
{
	uint8_t idx = 0;
	while (str[idx] != '\0')
	{
		uart_transmit(str[idx++]);
	}
}

void process_received_string(const char *str)
{
	uart_transmit(13);

	if (strcmp(str, "1") == 0)
	{
		PORTD |= 0b11111100;
	}
	else if (strcmp(str, "2") == 0)
	{
		PORTD &= (0 << 2) | (0 << 3) | (0 << 4) | (0 << 5) | (0 << 6) | (0 << 7);
	}
	else if (strcmp(str, "3") == 0)
	{
		if(displayFlag == 0){
			displayFlag = 1;
		}else if (displayFlag == 1){
			displayFlag = 0;
		}
	}
	else if (strcmp(str, " ") == 0)
	{
		pass;
	}
	else
	{
		uart_transmit_string("What?");
	}
}

// password
void check_password()
{
	if (strcmp(uart_buffer, password) == 0)
	{
		passFlag = 1;
		string_received = 0;
		uart_transmit(13);
		uart_transmit_string("Welcome");
		uart_transmit(13);
		uart_transmit_string("1- LED ON");
		uart_transmit(13);
		uart_transmit_string("2- LED OFF");
		uart_transmit(13);
		uart_transmit_string("3- Change Display Mode");
	}
	else
	{
		uart_transmit(13);
		uart_transmit_string("Not correct shifo");
		string_received = 0;
	}
	uart_transmit(13);
}

// GAS
int gasData()
{
	uint16 adc_value = 0;
	adc_value = ADC_readChannel(GAS_SENSOR_CHANNEL_ID);
	return adc_value;
}


// SPEAKER

void PWM1_init() {
    // Set SPEAKER_PIN as output

    // Configure Timer1 for CTC mode
    TCCR1B |= (1 << WGM12); // CTC mode
    TIMSK |= (1 << OCIE1A); // Enable Timer1 Compare Match A interrupt
    
}

void playTone() {
    // Calculate and set the OCR1A value for the desired frequency
    uint16_t ocrValue = (_FCPU / (2 * SPEAKER_FREQUENCY)) - 1;
    OCR1A = ocrValue;

    // Start the timer with no prescaling
    TCCR1B |= (1 << CS10);
}

void stopTone() {
    // Stop the timer by clearing the clock select bits
    TCCR1B &= ~(1 << CS10);
	PORTA &= ~(1 << SPEAKER_PIN);
}


interrupt[TIM1_COMPA] void TIMCOMPA_ISR(void)
{
	if (speakerToggle) {
        PORTA |= (1 << SPEAKER_PIN);
    } else {
        PORTA &= ~(1 << SPEAKER_PIN);
    }
    speakerToggle = !speakerToggle;
}


