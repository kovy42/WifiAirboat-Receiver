/**
	\file  driver.c
	\brief Code source de fonctions qui pilotent directement du matériel
	
	\author Emmanuel Proulx
	\author Nicolas Piette
	\author Nicolas Hamon
	\author David Nguyen
	\author Guillaume Moffette
	\author Jonathan Tessier
	\date 2018-02-05
*/

/******************************************************************************
Includes
******************************************************************************/

#include <avr/io.h>
#include "driver.h"
#include "lcd.h"
#include "utils.h"


/******************************************************************************
Définitions de fonctions
******************************************************************************/

void adc_init(void){
	
	// Configuration des broches utilisées du port A en entrée (Entre PA0 et PA7)	
	DDRA = clear_bit(DDRA,PA0);
	DDRA = clear_bit(DDRA,PA1);

	// Sélectionner la référence de tension: la tension d'alimentation (AVCC)
	ADMUX = clear_bit(ADMUX, REFS1);
	ADMUX = set_bit(ADMUX, REFS0);
    

	// Choisir le format du résultat de conversion: shift à gauche pour que
	// les 8 MSB se retrouvent dans le registre ADCH
	ADMUX = set_bit(ADMUX,ADLAR);

	// Choisir un facteur de division d'horloge (64) afin que l'horloge ait
	// une fréquence entre 50kHz et 200kHz. Donc 8MHz/64 = 125kHz.
	ADCSRA = set_bit(ADCSRA,ADPS2);
	ADCSRA = set_bit(ADCSRA,ADPS1);
	ADCSRA = clear_bit(ADCSRA,ADPS0);

	// Activer le CAN
	ADCSRA = set_bit(ADCSRA,ADEN);
}

uint8_t adc_read(uint8_t channel){
	
	// Choisir l'entrée analogique (broche) à convertir 
	ADMUX = write_bits(ADMUX, 0b00011111, channel);

	// Démarrage d'une conversion 
	ADCSRA = set_bit(ADCSRA,ADSC);
	
	// Attente de la fin de la conversion
	
	while(read_bit(ADCSRA,ADSC) == 1){
		
	}
	
	// Lecture et renvoie du résultat
	return ADCH;
	
}

void servo_init(void){
	
	
	// Configuration des broches de sortie
	DDRD = set_bit(DDRD,PD5);
	//DDRD = set_bit(DDRD,PD4);
	
	// Configuration du compteur et du comparateur
	TCCR1A = set_bit(TCCR1A,COM1A1);
	TCCR1A = set_bit(TCCR1A,COM1B1);
	
	TCCR1A = clear_bit(TCCR1A,COM1B0);
	TCCR1A = clear_bit(TCCR1A,COM1A0);
	
	TCCR1A = set_bit(TCCR1A,WGM11);
	
	TCCR1B = set_bit(TCCR1B,WGM13);
	TCCR1B = set_bit(TCCR1B,WGM12);

	// Configuration de la valeur maximale du compteur (top) à 20000
	ICR1 = write_bits(ICR1,0b1111111111111111,0b0100111000100000);
	
	// Initialiser la valeur du compteur à 0
	TCNT1 = write_bits(TCNT1,0b1111111111111111,0b0000000000000000);

	// Démarrer le compteur et fixer un facteur de division de fréquence à 8
	TCCR1B = clear_bit(TCCR1B,CS12);
	TCCR1B = set_bit(TCCR1B,CS11);
	TCCR1B = clear_bit(TCCR1B,CS10);
}

void servo_set_a(uint8_t angle){
	
	// Mise à l'échelle de la valeur du joystick (entre 0 à 255)
	// pour obtenir une valeur de comparaison entre 600 et 2400
	uint16_t scale = ((angle*7.06) + 600);
	
	// Modification du rapport cyclique du PWM du servomoteur (Timer 1, PD5 - OC1A)
	OCR1A = scale;
}

void servo_set_b(uint8_t angle){
	
	// Mise à l'échelle de la valeur du joystick (entre 0 à 255) 
	// pour obtenir une valeur de comparaison entre 600 et 2400
	
	
	// Modification du rapport cyclique du PWM du servomoteur (Timer 1, PD4 - OC1B)

}

void pwm_init(bool init_a, bool init_b){
	if(init_a){
		// Configuration des broches de sortie
		
		DDRB = set_bit(DDRB,PB3);
		// Configuration du compteur et du comparateur
		TCCR0 = set_bit(TCCR0,WGM00);
		TCCR0 = set_bit(TCCR0,WGM01);
		
		

		// Démarrer le compteur et fixer un facteur de division de fréquence à 1024
		TCCR0 = set_bits(TCCR0,0b00000101);
	}
	if(init_b){
		// Configuration des broches de sortie
		
		DDRD = set_bit(DDRD,PD7);
		// Configuration du compteur et du comparateur
		TCCR2 = set_bit(TCCR2,WGM20);
		TCCR2 = set_bit(TCCR2,WGM21);
		
		

		// Démarrer le compteur et fixer un facteur de division de fréquence à 1024
		TCCR2 = set_bits(TCCR2,0b00000111);
	}
	
	
}

void pwm_set_a(uint8_t duty){
	
	// Timer 0, PB3 - OC0
	
	// Pour avoir un duty de 0, il faut éteindre le PWM et directement piloter la sortie à 0
	if(duty == 0){
		
		//Mettre 0 dans la broche PB3 (OC0) du port B
		PORTB = clear_bit(PORTB, PB3);
		
		//Désactive le comparateur
		TCCR0 = clear_bit(TCCR0, COM01);
	}
	
	else{
		// Modification du rapport cyclique du PWM du moteur (Timer 0, PB3 - OC0)
		
		OCR0 = write_bits(OCR0,0b11111111,duty);
		//Active le comparateur
		TCCR0 = set_bit(TCCR0, COM01);
	}
}


void pwm_set_b(uint8_t duty){

	// Timer 2, PD7 - OC2
	
	// Pour avoir un duty de 0, il faut éteindre le PWM et directement piloter la sortie à 0
	if(duty == 0){
		
		//Mettre 0 dans la broche PD7 (OC2) du port D
		PORTD = clear_bit(PORTD, PD7);
		
		//Désactive le comparateur
		TCCR2 = clear_bit(TCCR2, COM21);
	}
	
	else{
		// Modification du rapport cyclique du PWM du moteur (Timer 2, PD7 - OC2)
		OCR2 = write_bits(OCR2,0b11111111,duty);
		
		//Active le comparateur
		TCCR2 = set_bit(TCCR2, COM21);
	}
}

void joystick_button_init(void){
	
}


bool joystick_button_read(void){
	return 0;
}
