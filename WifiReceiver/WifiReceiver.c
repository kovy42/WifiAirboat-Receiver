#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "driver.h"
#include "fifo.h"
#include "uart.h"
#include "lcd.h"

#define WAIT_DATA 0 // Attente d'une transmission
#define WAIT_OPEN_BRACKET 1 // Début de la transmission
#define READ_DATA 2 // Lecture de la transmission
#define DISPLAY_DATA 3 // Affichage

int main(void)
{
	/* ================== INITIALISATION ================== */
	lcd_init();
	adc_init();
	pwm_init(1,1);
	uart_init();
	uart_clean_rx_buffer();
	servo_init();

	SREG = set_bit(SREG, 7);

	/* ================== CONFIGURATION DU WIFI ================== */
	lcd_write_string("Connecting 2");
	lcd_set_cursor_position(4, 1);
	lcd_write_string("WIFI...");
		
	OSCCAL = OSCCAL + 6; // Calibration de la fréquence du uC de l'aéroglisseur
	DDRD = set_bit(DDRD, PD2); // pin 16 (PD2) = true (RST de l'ESP)
	PORTD = clear_bit(PORTD, PD2);
	_delay_ms(500);
	PORTD = set_bit(PORTD, PD2);
	_delay_ms(5000);
	uart_put_string("AT+CIPMODE=1\r\n\0"); // Mode Passthrough
	_delay_ms(250);
	uart_put_string("AT+CIPSTART=\"UDP\",\"0.0.0.0\",123,456\r\n\0"); // Établir la transmission UDP
	
	/*if (read_bit(DDRD, PD6) == 0) {
		// CODE DE LA MANETTE
		_delay_ms(250);
		uart_put_string("AT+CISEND\r\n\0");
		_delay_ms(250);
		uart_put_string("AT+CIPSTART=\"UDP\",\"XXX.XXX.XXX.XXX\",456,123\r\n\0"); // Établir la transmission UDP
	}
	else {
		// CODE DE L'AÉROGLISSEUR
		_delay_ms(250);
		uart_put_string("AT+CIPSTART=\"UDP\",\"0.0.0.0\",123,456\r\n\0"); // Établir la transmission UDP
	}*/
	
	/*
	*	Il faut connecter PD6 du PCB de l'aéroglisseur au VCC (3V3) pour que la pin PD6 soit à 1
	*/
	
	lcd_clear_display();
	lcd_write_string("Connected. :)");
	_delay_ms(500);
	lcd_clear_display();
	
	char text;
	uint8_t i = 0;
	char horizontale[4];
	char verticale[4];
	char mode = 'A';
	uint8_t valeurHori = 128;
	uint8_t valeurVerti = 0;
	uint8_t reception_state = WAIT_OPEN_BRACKET;
	
    while(1)
    {		
        /* ================== ASSIGNATION DES MOTEURS ================== */
		if (mode == '1') {
			pwm_set_a(valeurVerti);
			pwm_set_b(0);
		}
		else if (mode == '0') {
			pwm_set_b(valeurVerti);
		}
		servo_set_a(valeurHori);
		
		/* ================== PROGRAMME PRINCIPAL ================== */

		// Si le buffer de réception n'est plus vide
		if(!uart_is_rx_buffer_empty()) {
			text = uart_get_byte();

			switch (reception_state) {
				case WAIT_OPEN_BRACKET:
					if (text == '[') {
						reception_state = READ_DATA;
						i = 0;
					}
				break;
				
				case READ_DATA:
					if (text != ']'){
						if (i < 3)
							horizontale[i] = text;
							
						else if (i >= 3 && i < 6)
							verticale[i - 3] = text;
							
						else if (i >= 6) {
							if (text == 'A')
								mode = '1'; // LIFT MODE
							else
								mode = '0'; // SPEED MODE
						}
						i++;
					}
					else {
						i = 0;
						reception_state = DISPLAY_DATA;
					}
				break;
				
				/* ================== AFFICHAGE DU DATA SUR LE LCD ================== */
				case DISPLAY_DATA:
					// Initialisation du lcd
					horizontale[3] = 0;
					verticale[3] = 0;
					uart_clean_rx_buffer();
					lcd_clear_display();
					
					// Valeur horizontale
					lcd_set_cursor_position(0, 0);
					lcd_write_string("HOR : ");
					lcd_write_string(horizontale);
					
					// Affichage du mode
					lcd_set_cursor_position(11, 0);
					if(mode == '1')
						lcd_write_string("LIFT");
					if(mode == '0')
						lcd_write_string("SPEED");
					
					// Valeur verticale
					lcd_set_cursor_position(0, 1);
					lcd_write_string("VER : ");
					lcd_write_string(verticale);
					_delay_ms(100);
					
					reception_state = WAIT_OPEN_BRACKET;
					
					valeurHori = string_to_uint(horizontale);
					valeurVerti = string_to_uint(verticale);
					break;
					
				default:
					lcd_write_string("Error in machine state");
					break;
			}
		
		}
    }
}
