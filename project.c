#include "headers.h"
// add header file


void main(void)
{

	#asm("sei");
	// enable interrupts with assmebly code

	ADC_init();
	lcd_init(16);
	uart_init();
	DcMotor_Init();
	PWM1_init();
	// initialize the modules


    DDRA |= (1 << SPEAKER_PIN);
	DDRD |= (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7); 

	uart_transmit_string("Enter password : ");

	delay_ms(100);
	while (1)
	{

		// while (passFlag != 1);
		// if password is correct atmega will start the system
		
		delay_ms(50);
		lcd_clear();
		
		check_gas();
		
		// process string from client if enter '\0'
		if (string_received)
		{
			string_received = 0;
			process_received_string(uart_buffer);
		}

		// toggle for display
		if(displayFlag == 0){
			TempDC_display();
		}else if(displayFlag == 1){
			GAS_display();
		}
		

	}
}

// Check gas sensor value and call warning function
void check_gas(){
	uint16_t gas_data = gasData();

	if(gas_data >= 600){warning(1);}
	else if(gas_data < 600){warning(0);}
}

// warning function
void warning(uint8_t flag){
	if(flag == 1){
		playTone();
	}else if(flag == 0){
		stopTone();
	}

}


// show gas display
void GAS_display(){
	uint32_t Gas_raw_data = gasData();

	lcd_gotoxy(0, 0);
	lcd_puts("-GAS Sensor-");
	lcd_gotoxy(0,1);
	sprintf(temporaryString , "Values: %d" , Gas_raw_data );
	lcd_puts(temporaryString);
}

// show temperature and dc motor display
void TempDC_display()
{
	uint8_t temperature;

	temperature = LM35_getTemperature();
	switch (temperature / 10)
	{
	case 0: // 0-9
		DcMotor_Rotate(STOP, 0);
		sprintf(temporaryString, "   Fan is OFF");
		break;
	case 1: // 10-19
		DcMotor_Rotate(STOP, 0);
		sprintf(temporaryString, "   Fan is OFF");
		break;
	case 2: // 20-29
		DcMotor_Rotate(CLOCKWISE, 10);
		sprintf(temporaryString, "   Fan is ON 10%%");
		break;
	case 3: // 30-39
		DcMotor_Rotate(CLOCKWISE, 25);
		sprintf(temporaryString, "   Fan is ON 25%%");
		break;
	case 4: // 40-49
		DcMotor_Rotate(CLOCKWISE, 50);
		sprintf(temporaryString, "   Fan is ON 50%%");
		break;
	case 5: // 50-59
		DcMotor_Rotate(CLOCKWISE, 75);
		sprintf(temporaryString, "   Fan is ON 75%%");
		break;
	case 6: // 60-69
		DcMotor_Rotate(CLOCKWISE, 85);
		sprintf(temporaryString, "   Fan is ON 85%%");
		break;
	default: // 70 >=
		DcMotor_Rotate(CLOCKWISE, 100);
		sprintf(temporaryString, "   Fan is ON 100%%");
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

// read adc channel with channel number
uint16_t ADC_readChannel(uint8_t channel_num)
{
	ADMUX = (ADMUX & 0b11100000) | (channel_num);
	SET_BIT(ADCSRA, ADSC);
	while ((ADCSRA & (1 << ADIF)) == 0);
	SET_BIT(ADCSRA, ADIF);
	return ADCW;
}

//                                     LM35
// convert adc value to temperature
uint8_t LM35_getTemperature(void)
{
	uint8_t temp_value = 0;
	uint16_t adc_value = 0;
	/* Read ADC channel where the temperature sensor is connected */
	adc_value = ADC_readChannel(SENSOR_CHANNEL_ID_LM35);
	/* Calculate the temperature from the ADC value*/
	temp_value = (uint8_t)(((uint32_t)adc_value * SENSOR_MAX_TEMPERATURE * ADC_REF_VOLT_VALUE) / (ADC_MAXIMUM_VALUE * SENSOR_MAX_VOLT_VALUE));
	return temp_value;
}




// dcmotor init
void DcMotor_Init(void)
{
	/* DC-Motor 2 output pins */
	SET_BIT(DDRB,0);
	SET_BIT(DDRB,1);
	
	/* Stop the DC-Motor at the beginning */
	CLEAR_BIT(PORTB,0);
	CLEAR_BIT(PORTB,1);

}


// change direction of motor
void DcMotor_Rotate(DcMotor_State state, uint8_t speed)
{
	uint8_t duty_cycle;



	switch (state)
	{
	case CLOCKWISE:
		// Rotate motor clockwise
		SET_BIT(PORTB,0);
		CLEAR_BIT(PORTB,1);
		break;
	case ANTI_CLOCKWISE:
		// Rotate motor anti-clockwise
		CLEAR_BIT(PORTB,0);
		SET_BIT(PORTB,1);
		break;
	case STOP:
		// Stop the motor
		CLEAR_BIT(PORTB,0);
		CLEAR_BIT(PORTB,1);
		break;
	default:
		// Invalid motor state
		return;
	}

	/*Calculate the Duty Cycle and send it to the PWM Driver*/
	duty_cycle = ((uint8_t)(((uint16_t)(speed * TOP)) / 100));
	PWM_Timer0_Start(duty_cycle);
}






// pwm
void PWM_Timer0_Start(uint8_t duty_cycle)
{

	TCNT0 = 0;		   
	OCR0 = duty_cycle; 
	
	SET_BIT(DDRB, 3);


	// configure timer0 control register | fast PWM mode | non inverted mode | clock = 8mhz/8
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
		uart_transmit(13);
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
	uint16_t adc_value = 0;
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


