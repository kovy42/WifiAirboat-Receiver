/*
 * AirBoat.c
 *
 * Created: 2018-03-20 15:26:52
 *  Author: Equipe 24
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "driver.h"
#include "uart.h"
#include "fifo.h"
#include "lcd.h"

#define WAIT_DATA 0 // Attente d'une transmission
#define WAIT_OPEN_BRACKET 1 // Début de la transmission
#define READ_DATA 2 // Lecture de la transmission
#define DISPLAY_DATA 3 // Affichage

bool isOnBoard;
uint8_t getMaxBatteryValue(uint8_t);
uint8_t getBatteryUsagePercentage(uint8_t,uint8_t,uint8_t);
uint8_t getRealBatteryTension(uint8_t,uint8_t);
void DelBattery(uint8_t);
/* ==================== AIRBOAT VARIABLES ==================== */
char aText;
uint8_t aNbChar;
char aHorizontale[4];
char aVerticale[4];
char aFlyMode;
uint8_t aValeurHorizontale;
uint8_t aValeurVerticale;
uint8_t aEtatReception;
void initializeAirboat();
void waitForOpenBrackets();
void readData();
void displayData();
bool firstLoop;
uint8_t mBatterie;
uint8_t loopCount;
bool cutEngines;
bool emergencyBrake;
uint8_t tempStoredLift;

int main(void)
{
	/* ================== INITIALISATION GLOBALE ================== */
	lcd_init();
	adc_init();
	pwm_init(1,1);
	uart_init();
	uart_clean_rx_buffer();
	servo_init();
	SREG = set_bit(SREG, 7);
	uart_set_baudrate(BAUDRATE_9600);
	
	/* Lecture de la valeur déterminant le type de contrôlleur (remote/receiver) */
	
	
	
	/* Effectuer le code de l'aéroglisseur */

	initializeAirboat();
		
	while(1){
		/* ================== ASSIGNATION DES MOTEURS ================== */
		if(!cutEngines){
			if(emergencyBrake){
				pwm_set_a(0);
			}else{
				if (aFlyMode == '1') {
					pwm_set_a(aValeurVerticale);
					pwm_set_b(0);
					tempStoredLift = aValeurVerticale;
				}
				else if (aFlyMode == '0') {
					pwm_set_b(aValeurVerticale);
					if(aValeurHorizontale < 64 || aValeurHorizontale > 192){
						pwm_set_a(42);
					}else{
						pwm_set_a(tempStoredLift);
					}
					
					
				}
			}
			
			servo_set_a(aValeurHorizontale);
		}
		
		/* ================== PROGRAMME PRINCIPAL ================== */
			// Si le buffer de réception n'est plus vide
		if(!uart_is_rx_buffer_empty() && firstLoop != 1) {
			aText = uart_get_byte();
			switch (aEtatReception) {
				/* ================== Attente d'entrée de données ================== */
				case WAIT_OPEN_BRACKET:waitForOpenBrackets();break;
				
				/* ================== Lecture des données ==================== */
				case READ_DATA: readData();break;
				
				/* ================== Affichage des données sur le LCD ================== */
				case DISPLAY_DATA: displayData();break;
				
				/* ================== Affichage d'une erreur en cas d'absence d'état ================== */
				default:lcd_write_string("Error in machine state");break;
			}
			
		
		}
		firstLoop = 0;
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
	
	
	
}
	


/* ================== MÉTHODES POUR L'AÉROGLISSEUR ================== */

/************************************************************************/
/* Initialiser l'aéroglisseur                                           */
/************************************************************************/
void initializeAirboat(){
	uart_clean_rx_buffer();
	OSCCAL = OSCCAL + 6; // Calibration de la fréquence du uC de l'aéroglisseur
	DDRD = set_bit(DDRD, PD2); // pin 16 (PD2) = true (RST de l'ESP)
	PORTD = clear_bit(PORTD, PD2);
	_delay_ms(1500);
	PORTD = set_bit(PORTD, PD2);
	_delay_ms(1000);
	uart_put_string("AT+CIPMODE=1\r\n\0"); // Mode Passthrough
	_delay_ms(2500);
	uart_put_string("AT+CIPSTART=\"UDP\",\"0.0.0.0\",123,456\r\n\0"); // Établir la transmission UDP
	
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
	aValeurHorizontale = 128;
	aValeurVerticale = 0;
	firstLoop = 1;
	loopCount = 0;
	cutEngines = 0;
	aEtatReception = WAIT_OPEN_BRACKET;
	PORTB = set_bit(PORTB, 0);
	emergencyBrake = 0;
	tempStoredLift = 0;
}	

/************************************************************************/
/* Attente d'un segment de données                                      */
/************************************************************************/
void waitForOpenBrackets(){
	if (aText == '[') {
		PORTB = set_bit(PORTB, 1);
		aEtatReception = READ_DATA;
		aNbChar = 0;
	}
} 

/************************************************************************/
/* Lire les données envoyées par la télécommande                        */
/************************************************************************/
void readData(){
	/* Tant qu'on atteint pas la fin du segment de données... */
	if (aText != ']'){
		/* 3 premiers caractères: valeur de direction horizontale */
		if (aNbChar < 3){
			aHorizontale[aNbChar] = aText;	
		/* Caractères 3 à 5: valeur de direction verticale */
		}else if (aNbChar >= 3 && aNbChar < 6){
			aVerticale[aNbChar - 3] = aText;
		/* Dernier caractère: Mode de pilotage */
		}else if (aNbChar >= 6){
			/* Mode de lancement */
			if(aText == 'B'){
				emergencyBrake = 1;
			}else{
				emergencyBrake = 0;
				if (aText == 'L'){
					aFlyMode = '1'; // LIFT MODE
					/* Mode vitesse */
					}else{
					aFlyMode = '0'; // SPEED MODE
				}
			}
			
		}
		/* Itérer la position dans le segment de données */
		aNbChar++;
	}
	else {
		/* Réinitialiser la position dans le segment et envoyer les données au LCD */
		aNbChar = 0;
		aEtatReception = DISPLAY_DATA;
	}
}

/************************************************************************/
/* Afficher les valeurs reçues sur l'écran                              */
/************************************************************************/
void displayData(){
	// Initialisation du lcd
	aHorizontale[3] = 0;
	aVerticale[3] = 0;
	uart_clean_rx_buffer();	
	//_delay_ms(100);
	aEtatReception = WAIT_OPEN_BRACKET;
	aValeurHorizontale = string_to_uint(aHorizontale);
	aValeurVerticale = string_to_uint(aVerticale);
}

/**************************************************************************/
/* Returns the maximum value between 0 and 255 that the battery can output*/
/**************************************************************************/
uint8_t getMaxBatteryValue(uint8_t maxTension){
	return  maxTension * 0.232558f / 3.3f * 255;
}

/************************************************************************/
/* Returns the battery tension                                          */
/************************************************************************/
uint8_t getBatteryUsagePercentage(uint8_t adcValue, uint8_t maxTension, uint8_t minTension){
	return (int)((((float)adcValue / (float)getMaxBatteryValue(maxTension) - ((float)minTension/(float)maxTension)) *100) /(100-(100*minTension/maxTension)) * 100) ;
}

uint8_t getRealBatteryTension(uint8_t adcValue, uint8_t maxTension){
	return  (int)((float)adcValue * 10 / (float)getMaxBatteryValue(maxTension) * maxTension ) ; // Pour plus de précision, il affiche la valeur fois 100
}


void DelBattery(uint8_t battValue)
{
	if(battValue <= 100 && battValue > 80)
	PORTB = 0b00011111;	
	if(battValue <= 80 && battValue > 60)
	PORTB = 0b00011111;
	if(battValue <= 60 && battValue > 40)
	PORTB = 0b00011111;
	if(battValue <= 40 && battValue > 20)
	PORTB = 0b00011111;
	if(battValue <= 20 && battValue >= 0)
	PORTB = 0b00011111;	
	
}