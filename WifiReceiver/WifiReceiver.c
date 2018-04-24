/*
 * AirBoat.c
 *
 * Created: 2018-03-20 15:26:52
 *  Author: Equipe 24
 */ 

/* ============ Include all required libraries ============= */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "driver.h"
#include "uart.h"
#include "fifo.h"
#include "lcd.h"

/* =========== Define the reception states ============= */
#define WAIT_OPEN_BRACKET 1 // Wait for beginning of data transmission
#define READ_DATA 2 // Read data transmission
#define PROCESS_DATA 3 // Data processing


/* ==================== AIRBOAT VARIABLES ==================== */
bool firstLoop;
bool cutEngines;
bool emergencyBrake;
char aText;
char aHorizontal[4];
char aVertical[4];
char aFlyMode;
uint8_t aNbChar;
uint8_t aHorizontalValue;
uint8_t aVerticalValue;
uint8_t aReceptionState;
uint8_t mBatterie;
uint8_t loopCount;
uint8_t tempStoredLift;

/* =================== All methods used by the microcontroller ============== */
uint8_t getMaxBatteryValue(uint8_t);
uint8_t getBatteryUsagePercentage(uint8_t,uint8_t,uint8_t);
uint8_t getRealBatteryTension(uint8_t,uint8_t);
void initializeAirboat();
void waitForOpenBrackets();
void readData();
void processData();
void batteryProtection();

int main(void)
{
	/* ================ Initialize all the values and microcontroller ports on the ATMEGA32 ================ */
	initializeAirboat();
	
	/* ================ Microcontroller loop ================ */
	while(1){
		/* ================== ASSIGNATION DES MOTEURS ================== */
		if(!cutEngines){
			if(emergencyBrake){
				pwm_set_a(0);
			}else{
				if (aFlyMode == '1') {
					pwm_set_a(aVerticalValue);
					pwm_set_b(0);
					tempStoredLift = aVerticalValue;
				}
				else if (aFlyMode == '0') {
					pwm_set_b(aVerticalValue);
					if(aHorizontalValue < 64 || aHorizontalValue > 192){
						pwm_set_a(55);
					}else{
						pwm_set_a(tempStoredLift);
					}
					
					
				}
			}
			
			servo_set_a(aHorizontalValue);
		}
		
		/* ================== DATA RECEPTION ================== */
			// If the reception buffer is not empty (we have received data)
		if(!uart_is_rx_buffer_empty() && firstLoop != 1) {
			//Get the byte from UART
			aText = uart_get_byte();
			switch (aReceptionState) {
				/* ================== Wait for start of data stream ================== */
				case WAIT_OPEN_BRACKET:waitForOpenBrackets();break;
				
				/* ================== Read data ==================== */
				case READ_DATA: readData();break;
				
				/* ================== Process data ================== */
				case PROCESS_DATA: processData();break;
				
				/* ================== In case of an error, just break ================== */
				default:break;
			}
			
		
		}
		
		/* ========== Protect the battery from undervoltage (cut the engines at 6V) ============= */
		batteryProtection();
	
		firstLoop = 0;
	}
	
	
	
}
	
/* ========================= ALL METHODS =========================== */

/************************************************************************/
/* Initialize the airboat microcontroller                               */
/************************************************************************/
void initializeAirboat(){
	lcd_init();
	adc_init();
	pwm_init(1,1);
	uart_init();
	uart_clean_rx_buffer();
	servo_init();
	SREG = set_bit(SREG, 7);
	uart_set_baudrate(BAUDRATE_9600);
	OSCCAL = OSCCAL + 6; // Frequency calibration of the microcontroller
	DDRD = set_bit(DDRD, PD2);
	PORTD = clear_bit(PORTD, PD2);
	_delay_ms(1500);
	PORTD = set_bit(PORTD, PD2);
	_delay_ms(1000);
	uart_put_string("AT+CIPMODE=1\r\n\0"); // Passthrough mode
	_delay_ms(2500);
	uart_put_string("AT+CIPSTART=\"UDP\",\"0.0.0.0\",123,456\r\n\0"); // Set the UDP transmission
	lcd_clear_display();
	lcd_write_string("Connected. :)");
	_delay_ms(1000);
	lcd_clear_display();
	DDRB = set_bit(DDRB,PB0);
	DDRB = set_bit(DDRB,PB1);
	DDRB = set_bit(DDRB,PB2);
	DDRB = set_bit(DDRB,PB3);
	DDRB = set_bit(DDRB,PB4);
	aNbChar = 0;
	aFlyMode = '0';
	aHorizontalValue = 128;
	aVerticalValue = 0;
	firstLoop = 1;
	loopCount = 0;
	cutEngines = 0;
	aReceptionState = WAIT_OPEN_BRACKET;
	PORTB = set_bit(PORTB, 0);
	emergencyBrake = 0;
	tempStoredLift = 0;
}	

/************************************************************************/
/* Wait a data packet                                                   */
/************************************************************************/
void waitForOpenBrackets(){
	if (aText == '[') {
		PORTB = set_bit(PORTB, 1);
		aReceptionState = READ_DATA;
		aNbChar = 0;
	}
} 

/************************************************************************/
/* Read data received from the controller                               */
/************************************************************************/
void readData(){
	/* While the data stream does not close... */
	if (aText != ']'){
		/* The first 3 characters are the horizontal value to set */
		if (aNbChar < 3){
			aHorizontal[aNbChar] = aText;	
		/* Characters 3 to 5: the vertical value */
		}else if (aNbChar >= 3 && aNbChar < 6){
			aVertical[aNbChar - 3] = aText;
		/* The last character define the fly mode */
		}else if (aNbChar >= 6){
			/* Detect an emergency brake */
			if(aText == 'B'){
				emergencyBrake = 1;
			}else{
				emergencyBrake = 0;
				if (aText == 'L'){
					aFlyMode = '1'; // LIFT MODE
					}else{
					aFlyMode = '0'; // SPEED MODE
				}
			}
			
		}
		/* Increment the index in the data stream */
		aNbChar++;
	}
	else {
		/* Reinitialize the index when the data stream is closed*/
		aNbChar = 0;
		aReceptionState = PROCESS_DATA;
	}
}

/************************************************************************/
/* Process the data received from the controller                        */
/************************************************************************/
void processData(){
	aHorizontal[3] = 0;
	aVertical[3] = 0;
	uart_clean_rx_buffer();	
	aReceptionState = WAIT_OPEN_BRACKET;
	aHorizontalValue = string_to_uint(aHorizontal);
	aVerticalValue = string_to_uint(aVertical);
}

/**************************************************************************/
/* Returns the maximum value between 0 and 255 that the battery can output*/
/**************************************************************************/
uint8_t getMaxBatteryValue(uint8_t maxTension){
	return  maxTension * 0.232558f / 3.3f * 255;
}

/************************************************************************/
/* Returns the battery usage percentage                                 */
/************************************************************************/
uint8_t getBatteryUsagePercentage(uint8_t adcValue, uint8_t maxTension, uint8_t minTension){
	return (int)((((float)adcValue / (float)getMaxBatteryValue(maxTension) - ((float)minTension/(float)maxTension)) *100) /(100-(100*minTension/maxTension)) * 100) ;
}

/************************************************************************/
/* Returns the real battery tension                                     */
/************************************************************************/
uint8_t getRealBatteryTension(uint8_t adcValue, uint8_t maxTension){
	return  (int)((float)adcValue * 10 / (float)getMaxBatteryValue(maxTension) * maxTension ) ; // For more precision, output the value x10
}

/************************************************************************/
/* This method protect the battery from undervoltage by 
shutting down the engine at 6V                                          */
/************************************************************************/
void batteryProtection(){
	mBatterie = getRealBatteryTension(adc_read(PA3), 9);
	if(mBatterie < 63){
		if(mBatterie < 60){
			cutEngines = 1;
			if(loopCount < 100){
				PORTB = set_bit(PORTB, 4);
				}else{
				PORTB = clear_bit(PORTB, 4);
				if(loopCount == 600){
					loopCount = 0;
				}
			}
			loopCount++;
		}
		else{
			cutEngines = 0;
			PORTB = set_bit(PORTB,4);
		}
		
	}
}


