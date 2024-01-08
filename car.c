/*
 * main.c
 *
 *  Created on: 10 sty 2016
 *      Author: Mariusz
 */


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "MK_USART/mkuart.h"


volatile uint8_t serwo1; // nalezy ustawic od 24 do 36
volatile uint8_t serwo2; // dla masztu
volatile uint8_t serwo3; // dla dzia³ka
volatile uint8_t serwo4; // dla dzia³ka
volatile uint8_t pwm1;   // obroty silnika silnika

void analizuj_dane_rs232(char *buf);
void key_press( uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) ; // reakcja na wciœniêcie klawiszu
void key_push_up( uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) ;// na zwolnienie klawiszu

uint8_t key1_lock, key2_lock, key3_lock, key4_lock; /// do obs³ugi klawisza kolejno 1,2 itd

#define KEY1 (1<<PA2)		  /// klawisz do wysy³ania przód/ty³/stop/krancówki
#define KEY2 (1<<PA3)		  /// Klawisz do wysy³ania przód/ty³/stop/krancówki

char uart_buf[100];

/* przydatne definicje pinów steruj¹cych silnikiem(napêd) */
#define WE_A PD7
#define WE_B PC0
// Definicje steruj¹ce wysówem masztu
#define WE_1 PA0
#define WE_2 PA1
// Definicje steruj¹ce autopomp¹
#define WE_3 PD4
// maszt
//#define WE_4 PD5


/* definicje poleceñ steruj¹cych prac¹ silnika */
#define DC_TYL PORTD &= ~(1<<WE_A); PORTC |= (1<<WE_B)
#define DC_PRZOD PORTD |= (1<<WE_A); PORTC &= ~(1<<WE_B)
#define DC_STOP PORTD &= ~(1<<WE_A); PORTC &= ~(1<<WE_B)

/* definicje poleceñ steruj¹cych wysówem masztu */
#define DC_GORA PORTA &= ~(1<<WE_1); PORTA |= (1<<WE_2)
#define DC_DOL PORTA |= (1<<WE_1); PORTA &= ~(1<<WE_2)
#define DC_ZATRZYMAJ PORTA |=(1<<WE_1); PORTA |= (1<<WE_2)
#define DC_ZATRZYMAJDOWN PORTA &= ~(1<<WE_1); PORTA &= ~(1<<WE_2) // dla opuszczania

/* definicje poleceñ steruj¹cych autopompa */
#define DC_ON PORTD |= (1<<WE_3)
#define DC_OFF PORTD &= ~(1<<WE_3)

/* definicje poleceñ steruj¹cych do wszystkich Ledów*/
#define AWA_ON  PORTB |= (1<<PB2);  PORTB |= (1<<PB0)  // makrodefinicja – Awaryjne zapalone
#define AWA_OFF PORTB &= ~(1<<PB2); PORTB &= ~(1<<PB0) // makrodefinicja – Awaryjne zgaszone
#define AWA_TOG PORTB ^= (1<<PB2);	PORTB ^= (1<<PB0)  // makrodefinicja – Awaryjne zmiana stanu

#define SWIATLA_ON  PORTB |= (1<<PB1)	// makrodefinicja – Œwiat³a w³¹czone
#define SWIATLA_OFF PORTB &= ~(1<<PB1)	// makrodefinicja – Œwiat³a wy³¹czone

#define VCC_DZIALKA_ON  PORTA |= (1<<PA6)	// makrodefinicja – Œwiat³a w³¹czone
#define VCC_DZIALKA_OFF PORTA &= ~(1<<PA6)	// makrodefinicja – Œwiat³a wy³¹czone

#define COFANIE_ON  PORTC |= (1<<PC3)	// makrodefinicja – Œwiat³a cofania w³¹czone
#define COFANIE_OFF PORTC &= ~(1<<PC3)	// makrodefinicja – Œwiat³a cofania wy³¹czone

#define LEWY_ON  PORTB |= (1<<PB2)	 // makrodefinicja – Lewy kierunek zapalony
#define LEWY_OFF PORTB &= ~(1<<PB2)	 // makrodefinicja – Lewy kiedunek zgaszony
#define LEWY_TOG PORTB ^= (1<<PB2)	 // makrodefinicja – Lewy kierunek zmiana stanu

#define VCC_LEDY_ON  PORTB |= (1<<PB3)	 // makrodefinicja – Napiêcie do ledów i skretu ON
#define VCC_LEDY_OFF PORTB &= ~(1<<PB3)  // makrodefinicja – Napiêcie do ledów i skrêtu OFF


#define PRAWY_ON  PORTB |= (1<<PB0)	 // makrodefinicja – Prawy kierunek zapalony
#define PRAWY_OFF PORTB &= ~(1<<PB0) // makrodefinicja – Prawy kiedunek zgaszony
#define PRAWY_TOG PORTB ^= (1<<PB0)	 // makrodefinicja – Prawy kierunek zmiana stanu

#define LEWYLED_ON  PORTC |= (1<<PC6)	 // makrodefinicja – Lewy górny niebieski led zapalony
#define LEWYLED_OFF PORTC &= ~(1<<PC6)	 // makrodefinicja – Lewy górny niebieski led zgaszony
#define LEWYLED_TOG PORTC ^= (1<<PC6)	 // makrodefinicja – Lewy górny niebieski led zmiana stanu

#define PRAWYLED_ON  PORTC |= (1<<PC5)	 // makrodefinicja – Prawy górny niebieski led zapalony
#define PRAWYLED_OFF PORTC &= ~(1<<PC5)  // makrodefinicja – Prawy górny niebieski led zgaszony
#define PRAWYLED_TOG PORTC ^= (1<<PC5)	 // makrodefinicja – Prawy górny niebieski led zmiana stanu

#define LED_ON  PORTC |= (1<<PC2)	 // makrodefinicja – Niebieskie led zapalone
#define LED_OFF PORTC &= ~(1<<PC2)   // makrodefinicja – Niebieskie led zgaszone
#define LED_TOG PORTC ^= (1<<PC2)	 // makrodefinicja – Niebieskie led zmiana stanu

#define MASZT_ON  PORTA |= (1<<PA7)		// makrodefinicja – w³¹czanie masztu
#define MASZT_OFF PORTA &= ~(1<<PA7)	// makrodefinicja – wy³aczanie masztu

#define SYRENA_OFF  PORTD |= (1<<PD3);  PORTD |= (1<<PD2)  	// makrodefinicja – syrena OFF
#define SYRENA_ON 	PORTD &= ~(1<<PD3)					    // makrodefinicja – syrena ON
#define ZSYRENA_ON 	PORTD &= ~(1<<PD2)					    // makrodefinicja – 2 syrena ON
#define ZSYRENA_OFF PORTD |= (1<<PD2)					    // makrodefinicja – 2 syrena OFF


uint16_t obroty;	// zmienna do ADC przy PWM
uint16_t kierunek;	// zmienna do ADC przy prawo/lewo
uint16_t masztlp;	// zmienna do ADC przy prawo/lewo
uint16_t dzialko;	// zmienna do ADC dla dzia³ka góra dó³
uint16_t lpdzialko;	// zmienna do ADC dla dzia³ka lewo prawo
uint8_t jazda; 		// zmienna do odbierania w RS czy auto ma jechac do przodu czy do ty³u
uint8_t maszt; 		// zmienna do odbierania w RS czy maszt ma sie wysówac czy chowac
uint8_t krancowka; 	// zmienna do blokady masztu przed za duzym wysuniêciem
uint8_t swiatla; 	// zmienna do obioru RS dla œwiate³ mijania
uint8_t awaryjne; 	// zmienna do obioru RS dla œwiate³ awaryjnych
uint8_t migacz; 	// zmienna do obioru RS dla migaczy
uint8_t sygnal; 	// zmienna do obioru RS dla syreny
uint8_t led; 		// zmienna do obioru RS dla LEDÓW niebieskich
uint8_t pompa; 		// zmienna do obioru RS dla pompki
uint8_t zmaszt; 	// zmienna do obioru RS dla on/off maszt
uint8_t syrena ; 	// zmienna do obioru RS dla syreny

void max(void) {			// maszt maksymalnie do gory
	krancowka = 1;

}
void min(void) {			// maszt maksymalnie na dole
	krancowka = 2;

}
void neutral(void) {		// maszt miedyz max a minimum
	krancowka = 0;

}

//*****************************POCZATEK PROGRAMU**************************************
int main(void){


	/// Ustawienie portów jako wyjœcia

	DDRC |=(1<<PC0);
	DDRC |=(1<<PC2);
	DDRC |=(1<<PC3);
	DDRC |=(1<<PC5);
	DDRC |=(1<<PC6);
	DDRC |=(1<<PC7);

	DDRA |=(1<<PA4);
	DDRA |=(1<<PA5);
	DDRA |=(1<<PA6);
	DDRA |=(1<<PA7);

	DDRB |=(1<<PB0);
	DDRB |=(1<<PB1);
	DDRB |=(1<<PB2);
	DDRB |=(1<<PB3);


	DDRD |=(1<<PD3);
	DDRD |=(1<<PD2);


	/* ZMIENNE DO OBS£UGI KLAWISZY */
	PORTA |= KEY1|KEY2;  // podci¹gamy linie klawiszy do VCC

	/* ustawiamy piny steruj¹ce serwami jako wyjœcia */
	DDRC |=(1<<PC1);
	DDRA |=(1<<PA4);

	/* ustawiamy piny steruj¹ce MOSTKA H jako wyjœcia */
	DDRC |= (1<<WE_B);
	DDRD |= (1<<WE_A);
	DDRD |=(1<<PD6); /// PWM do MOSTKA H

	DDRA |= (1<<WE_1);
	DDRA |= (1<<WE_2);
	DDRD |= (1<<WE_3);
	DDRC |= (1<<PC4);

	serwo1=20; // tu ustawic pozycjwe srodkowa nw ile
	DC_STOP;   // takie na wszelki wypadek przed startem auta
	obroty=0;



	// Timer 2 do diód przerwanie co 0,01 sekundy
		TCCR2 |= (1<<WGM21);	// tryb  CTC
		TCCR2 |= (1<<CS20)| (1<<CS21)| (1<<CS22);		// preskaler = 1024
		OCR2 = 107;				// dodatkowy podzia³ czêsttotliwoœci przez 200
		TIMSK |= (1<<OCIE2);	// zezwolenie na przerwanie CompareMatch

	/// Timer 0 do obs³ugi serwomechanizmów
		TCCR0 |= (1<<WGM01); //tryb ctc
		TCCR0 |= (1<<CS01); //preskaler 8
		OCR0 = 69; // podzia³ co 70 impulsów czyli co 0,00005 sekundy
		TIMSK |= (1<<OCIE0);




	USART_Init( __UBRR );

	register_uart_str_rx_event_callback(analizuj_dane_rs232);

	_delay_ms(10);

	sei();


	masztlp=127;
	_delay_ms(50);
	kierunek=127;
	VCC_LEDY_ON;  // W³¹czenie zasilania ledów  i  dzia³ka od skrêtu
	// kolejne delay gdzieœ za chwilê

	_delay_ms(50);
	dzialko=127;
	lpdzialko=127;
	VCC_DZIALKA_ON; // w³aczenie zasilania


//************************************ PÊTLA G£ÓWNA ******************************************

	while(1){


		UART_RX_STR_EVENT( uart_buf );

		//*********** LEWO/PRAWO *******************

		serwo1 = 9 + (kierunek/27);  // korygowanie serwa//
		serwo2 = 12 + (masztlp/8);   // serwo do lewo prawo masztu
		serwo3 = 12 + (dzialko/15);   // serwo do góra dó³ dzia³ka ju¿ ustaione
		serwo4 = 12 + (lpdzialko/8);  // korygowanie serwa dzialko lewo prawo

		//****** Sterowanie prêdkosci¹ napedu Start

		if( jazda == 0){
			DC_STOP;
			COFANIE_OFF;
		}

		if( jazda == 2){
			COFANIE_ON;
			DC_TYL;
		}
		if( jazda == 1){
			DC_PRZOD;
		}


		if(obroty < 15){		  // Ustawianie PWM od którego bd rusza³ silnik
			 pwm1 = 0;
			 DC_STOP;
		}
			else pwm1 = obroty;
		//****** Sterowanie prêdkosci¹ napedu Koniec


		/* OBS£UGA MASZTU */// krancowki

		key_press( &key1_lock, &PINA, KEY1, max  );	// maszt maksymalnie podniesiony
		key_push_up( &key2_lock, &PINA, KEY1, neutral );	// maszt miedzy max a minimum
		key_press( &key3_lock, &PINA, KEY2, min );	// maszt maksymalnie opóŸniony
		key_push_up( &key4_lock, &PINA, KEY2, neutral );	// maszt miedzy max a minimum

		//****** wysów masztu i chowanie



		if( maszt == 0 ){
			DC_ZATRZYMAJ;

		}

		if( maszt == 2)
		{
			if( krancowka ==2) {
			DC_ZATRZYMAJDOWN;}
			else DC_DOL ;
		}

		if( maszt == 1)
		{
			if( krancowka ==1) {
			DC_ZATRZYMAJ;}
			else DC_GORA ;
		}

// Œwiat³a LEDY

		if( swiatla == 0 ){
			SWIATLA_OFF;
		}


		if( swiatla == 1 ){
			SWIATLA_ON;
		}


		if( awaryjne == 0 && migacz == 0 ){
			AWA_OFF;
		}


		if( migacz == 0 && awaryjne == 0 ){
			LEWY_OFF;
			PRAWY_OFF;
		}

		if( led ==0 ){
			LEWYLED_OFF;
			PRAWYLED_OFF;
			LED_OFF;
		}

		if( pompa == 1 ){
			DC_ON;
		}

		if( pompa == 0 ){
			DC_OFF;
		}

		if( zmaszt == 1 ){
			MASZT_ON;
		}

		if( zmaszt == 0 ){
			MASZT_OFF;
		}

		if( syrena == 0 ){
			SYRENA_OFF;

		}







	}
}





//************************************ KONIEC PÊTLI G£ÓWNEJ **********************************


//************************************ FUNKCJA DO SERW I PWM *********************************

ISR( TIMER0_COMP_vect )
{

// PWM DO SERWOMECHANIZMÓW
		static uint16_t cnt; // ZMIENNA DO SERW

		if(cnt <= serwo1)PORTC |= (1<<PC1);	// PRAWO LEWO
			else PORTC &=  ~(1<<PC1);

		if(cnt <= serwo2)PORTA |= (1<<PA4);	// maszt
			else PORTA &=  ~(1<<PA4);

		if(cnt <= serwo3)PORTA |= (1<<PA5);	// dzia³ko góra dó³
			else PORTA &=  ~(1<<PA5);

		if(cnt <= serwo4)PORTC |= (1<<PC7);	// dzia³ko lewo/prawo
			else PORTC &=  ~(1<<PC7);

		cnt++;
		if(cnt>399) cnt = 0;

// PWM DO SILNIKOW

			static uint16_t pwm;

			if(pwm <= pwm1)PORTD |= (1<<PD6);
				else PORTD &=  ~(1<<PD6);

			pwm++;
			if(pwm>255) pwm = 0;


}

ISR( TIMER2_COMP_vect ) // timer do odliczania czasów dla migaczy syreny ledów itd
	{

	static uint8_t x; // ZMIENNA DO AWARYJNYCH

		if( awaryjne == 1){ x++;
		migacz = 0; }
		if( x == 40){
			AWA_ON;
		}
		if( x == 80){
			AWA_OFF;
			x = 0;
			}

	static uint8_t y; // ZMIENNA DO PRAWEGO MIGACZA

		if( migacz == 1 && awaryjne ==0) y++;
		if( y == 40){
			PRAWY_TOG;
			y = 0;
		}

	static uint8_t z; // ZMIENNA DO LEWEGO MIGACZA

		if( migacz == 2 && awaryjne ==0) z++;
		if( z == 40){
			LEWY_TOG;
			z = 0;
		}

	static uint8_t c; // ZMIENNA DO kogótów

		if( led == 1) c++;
		if( c == 1){
			LEWYLED_ON; }
		if( c == 8){
			LEWYLED_OFF; }
		if( c == 15){
			LEWYLED_ON; }
		if( c == 22){
			LEWYLED_OFF; }
		if( c == 29){
			LEWYLED_ON; }
		if( c == 36){
			LEWYLED_OFF; }	//
		if( c == 43){
			PRAWYLED_ON; }
		if( c == 50){
			PRAWYLED_OFF; }
		if( c == 57){
			PRAWYLED_ON; }
		if( c == 64){
			PRAWYLED_OFF; }
		if( c == 71){
			PRAWYLED_ON; }
		if( c == 78){
			PRAWYLED_OFF; }
		if( c == 84){
			c=0;  }

	static uint8_t v; // ZMIENNA DO kogótów

		if( led == 1) v++;
		if( v == 10){
			LED_ON; }
		if( v == 20){
			LED_OFF; }
		if( v == 30){
			LED_ON; }
		if( v == 40){
			LED_OFF; }
		if( v == 50){
			LED_ON; }
		if( v == 60){
			LED_OFF; }
		if( v == 90){ // jak by za d³uga przerwa by³a
			v = 0; }


		static uint16_t b; // ZMIENNA DO SYRENY

		if( syrena == 0 ){
			b = 0;}
		if( syrena == 1 )
			{ b++;
		if( b == 1){
			SYRENA_ON; }
		if( b == 600){
			ZSYRENA_ON;
			SYRENA_ON;}
		if( b == 1700){
			ZSYRENA_OFF;
			b = 0;}
			}
	}

//************************************ FUNKCJA DO OBS£UGI ODEBRANYCH DANYCH ******************

void analizuj_dane_rs232(char *buf){

// PORÓWNYWANIE POJEDYNCZYCH S£ÓW I USTAWIANIE KIERUNKU JAZDY SWIATE£ ITP

 if(!strcmp(buf, "przod"))  {		// JAZDA DO PRZODU
	 jazda=1;
 }

 if(!strcmp(buf, "stop"))  {		// STOP/RECZNY
	 jazda=0;
 }
 if(!strcmp(buf, "tyl"))  {			// JAZDA DO TY£U
	 jazda=2;
 }


 if(!strcmp(buf, "masztup"))  {		// PODNSZENIE MASZTU
	 maszt=1;
 }

 if(!strcmp(buf, "masztwait"))  {	// ZATRZYMANIE MASZTU
	 maszt=0;
 }
 if(!strcmp(buf, "masztdown"))  {	// OPUSZCZANIE MASZTU
	 maszt=2;
 }

 if(!strcmp(buf, "swiatlaon"))  {		// ŒWIAT£A MIJANIA ON
	 swiatla=1;
 }

 if(!strcmp(buf, "swiatlaoff"))  {	    // ŒWIAT£A MIJANIA OFF
	 swiatla=0;
 }

 if(!strcmp(buf, "awaryjneon"))  {		// ŒWIAT£A AWARYJNE ON
	 awaryjne=1;
 }

 if(!strcmp(buf, "awaryjneoff"))  {	    // ŒWIAT£A AWARYJNE OFF
	 awaryjne=0;
 }

 if(!strcmp(buf, "prawymigacz"))  {		// PRAWY MIGACZ
	 migacz=1;
 }

 if(!strcmp(buf, "lewymigacz"))  {	    // LEWY MIGACZ
	 migacz=2;
 }

 if(!strcmp(buf, "migaczoff"))  {	    // MIGACZ OFF
	 migacz=0;
 }

 if(!strcmp(buf, "sygnalyon"))  {	    // SYRENA ON
	 sygnal=1;
 }

 if(!strcmp(buf, "sygnalyoff"))  {	    // SYRENA OFF
	 sygnal=0;
 }

 if(!strcmp(buf, "ledon"))  {	    // LED ON
	 led=1;
 }

 if(!strcmp(buf, "ledoff"))  {	    // LED OFF
	 led=0;
 }

 if(!strcmp(buf, "dzialkoon"))  {	    // POMPA ON
	 pompa=1;
 }

 if(!strcmp(buf, "dzialkooff"))  {	    // POMPA OFF
	 pompa=0;
 }

 if(!strcmp(buf, "maszton"))  {	       	// MASZT ON
	 zmaszt=1;
 }

 if(!strcmp(buf, "masztoff"))  {	    // MASZT OFF
	 zmaszt=0;
 }

 if(!strcmp(buf, "sygnalyon"))  {	    // SYGNA£Y ON
	 syrena=1;
 }

 if(!strcmp(buf, "sygnalyoff"))  {	    // SYGNA£Y OFF
	 syrena=0;
 }



 //*************** PORÓWNYWANIE WARTOŒCI PWM***********************

	 char *wsk;
	wsk = strtok(buf, ",");

	if(!strcmp(wsk, "silnik")){

		wsk = strtok(NULL, ",");

		obroty = atoi(wsk);

	}

	if(!strcmp(wsk, "lewo")){

		wsk = strtok(NULL, ",");

		kierunek = atoi(wsk);

	}
	if(!strcmp(wsk, "maszt")){

		wsk = strtok(NULL, ",");

		masztlp = atoi(wsk);


	}
	if(!strcmp(wsk, "dzialkogd")){

		wsk = strtok(NULL, ",");

		dzialko = atoi(wsk);


	}
	if(!strcmp(wsk, "dzialkolp")){

		wsk = strtok(NULL, ",");

		lpdzialko = atoi(wsk);


	}



}

////////////////// REAKCJA  NA WCISNIÊCIE I ZWOLNINIE KLAWISZA
void key_press(uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) {

 register uint8_t key_press = (*KPIN & key_mask);

 if( !*klock && !key_press ) {
  *klock=1;

  // reakcja na PRESS (wciniêcie przycisku)
  if(kfun) kfun();

 } else if( *klock && key_press ) (*klock)++;
}



void key_push_up(uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) {

 register uint8_t key_press = (*KPIN & key_mask);

 if( !*klock && !key_press ) *klock=1;
 else if( *klock && key_press ) {
  if( !++*klock ) {

   // reakcja na PUSH_UP (zwolnienie przycisku)
   if(kfun) kfun();
  }
 }
}

