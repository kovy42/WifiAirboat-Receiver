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
#define WAIT_OPEN_BRACKET 1 // D�but de la transmission
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

/* ==================== REMOTE VARIABLES ===================== */
uint8_t mHorizontale;
uint8_t mVerticale;
uint8_t mLiftMemory;
uint8_t mBatterie;
bool mButtonState;
bool mFlyMode;
char mOutputText[16];
bool mOldFlyState;
void initializeRemote();

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
	
	/* Lecture de la valeur d�terminant le type de contr�lleur (remote/receiver) */
	
	
	
	/* Effectuer le code de l'a�roglisseur */

	initializeAirboat();
		
	while(1){
		/* ================== ASSIGNATION DES MOTEURS ================== */
		pwm_set_a(255);
		pwm_set_b(255);
		if (aFlyMode == '1') {
			//pwm_set_a(aValeurVerticale);
			//pwm_set_b(0);
			
		}
		else if (aFlyMode == '0') {
			//pwm_set_b(aValeurVerticale);
			
		}
		//servo_set_a(aValeurHorizontale);
		servo_set_a(255);
		/* ================== PROGRAMME PRINCIPAL ================== */
			// Si le buffer de r�ception n'est plus vide
		if(!uart_is_rx_buffer_empty()) {
			aText = uart_get_byte();
			switch (aEtatReception) {
				/* ================== Attente d'entr�e de donn�es ================== */
				case WAIT_OPEN_BRACKET:waitForOpenBrackets();break;
				
				/* ================== Lecture des donn�es ==================== */
				case READ_DATA: readData();break;
				
				/* ================== Affichage des donn�es sur le LCD ================== */
				case DISPLAY_DATA: displayData();break;
				
				/* ================== Affichage d'une erreur en cas d'absence d'�tat ================== */
				default:lcd_write_string("Error in machine state");break;
			}
			
		
		}
	
	}
	
	
	
}
	


/* ================== M�THODES POUR L'A�ROGLISSEUR ================== */

/************************************************************************/
/* Initialiser l'a�roglisseur                                           */
/************************************************************************/
void initializeAirboat(){
	OSCCAL = OSCCAL + 6; // Calibration de la fr�quence du uC de l'a�roglisseur
	DDRD = set_bit(DDRD, PD2); // pin 16 (PD2) = true (RST de l'ESP)
	PORTD = clear_bit(PORTD, PD2);
	_delay_ms(500);
	PORTD = set_bit(PORTD, PD2);
	_delay_ms(5000);
	uart_put_string("AT+CIPMODE=1\r\n\0"); // Mode Passthrough
	_delay_ms(250);
	uart_put_string("AT+CIPSTART=\"UDP\",\"0.0.0.0\",123,456\r\n\0"); // �tablir la transmission UDP
	
	lcd_clear_display();
	lcd_write_string("Connected. :)");
	_delay_ms(500);
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
	aEtatReception = WAIT_OPEN_BRACKET;
	PORTB = 0b00011111;
}	

/************************************************************************/
/* Attente d'un segment de donn�es                                      */
/************************************************************************/
void waitForOpenBrackets(){
	if (aText == '[') {
		
		aEtatReception = READ_DATA;
		aNbChar = 0;
	}
} 

/************************************************************************/
/* Lire les donn�es envoy�es par la t�l�commande                        */
/************************************************************************/
void readData(){
	/* Tant qu'on atteint pas la fin du segment de donn�es... */
	if (aText != ']'){
		/* 3 premiers caract�res: valeur de direction horizontale */
		if (aNbChar < 3){
			aHorizontale[aNbChar] = aText;	
		/* Caract�res 3 � 5: valeur de direction verticale */
		}else if (aNbChar >= 3 && aNbChar < 6){
			aVerticale[aNbChar - 3] = aText;
		/* Dernier caract�re: Mode de pilotage */
		}else if (aNbChar >= 6){
			/* Mode de lancement */
			if (aText == 'A'){
				aFlyMode = '1'; // LIFT MODE
			/* Mode vitesse */
			}else{
				aFlyMode = '0'; // SPEED MODE
			}		
		}
		/* It�rer la position dans le segment de donn�es */
		aNbChar++;
	}
	else {
		/* R�initialiser la position dans le segment et envoyer les donn�es au LCD */
		aNbChar = 0;
		aEtatReception = DISPLAY_DATA;
	}
}

/************************************************************************/
/* Afficher les valeurs re�ues sur l'�cran                              */
/************************************************************************/
void displayData(){
	// Initialisation du lcd
	aHorizontale[3] = 0;
	aVerticale[3] = 0;
	uart_clean_rx_buffer();
	lcd_clear_display();
	
	// Valeur horizontale
	lcd_set_cursor_position(0, 0);
	lcd_write_string("HOR : ");
	lcd_write_string(aHorizontale);
	
	// Affichage du mode
	lcd_set_cursor_position(11, 0);
	if(aFlyMode == '1')
	lcd_write_string("LIFT");
	if(aFlyMode == '0')
	lcd_write_string("SPEED");
	
	// Valeur verticale
	lcd_set_cursor_position(0, 1);
	lcd_write_string("VER : ");
	lcd_write_string(aVerticale);
	_delay_ms(100);
	
	aEtatReception = WAIT_OPEN_BRACKET;
	aValeurHorizontale = string_to_uint(aHorizontale);
	aValeurVerticale = string_to_uint(aVerticale);
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