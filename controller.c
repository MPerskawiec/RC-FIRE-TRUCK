/*
 * main.c
 *
 *  Created on: 9 sty 2016
 *      Author: Mariusz
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>


#include "MK_USART/mkuart.h"


void analizuj_dane_rs232(char *buf); 				// funkcja odbierajaca danew z RS232
uint16_t pomiar(uint8_t kanal);						// Deklaracja funkcji pomiaru
char *int_to_str(int val, char *str, int8_t fw);	// Deklaracja funkcji do konwersji liczby na �a�cuch tekstowy ASCII

char uart_buf[100]; 				// bufor na dane z rs232
char buf[10];						// bufor na potrzeby konwersji liczb do ASCII

void key_press( uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) ; // reakcja na wci�ni�cie klawiszu
void key_push_up( uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) ;// na zwolnienie klawiszu

uint8_t key1_lock, key2_lock, key3_lock, key4_lock;     /// do obs�ugi klawisza od prz�t/stop/ty�
uint8_t key5_lock, key6_lock, key7_lock, key8_lock;     /// do obs�ugi klawisza od maszt up/wait/down
uint8_t key9_lock, key10_lock, key11_lock, key12_lock;  /// do obs�ugi kierunkowskaz�w
uint8_t key13_lock, key14_lock, key15_lock, key16_lock; /// do obs�ugi dzia�ko on/off / i sygnaly on/off
uint8_t key17_lock, key18_lock, key19_lock, key20_lock; /// do obs�ugi kogut�w on/off i maszt on/off
uint8_t key21_lock, key22_lock, key23_lock, key24_lock; /// do obs�ugi swiate� on/off i awaryjne on/off

#define KEY1  (1<<PB0)		  /// klawisz do wysy�ania prz�d/ty�/stop
#define KEY2  (1<<PB1)		  /// Klawisz do wysy�ania prz�d/ty�/stop
#define KEY3  (1<<PC3)		  /// klawisz do wysy�ania maszt g�ra/stop
#define KEY4  (1<<PC2)		  /// Klawisz do wysy�ania maszt d�/stop
#define KEY5  (1<<PD7)		  /// klawisz do wysy�ania prawy kiedunkowskaz
#define KEY6  (1<<PC0)		  /// Klawisz do wysy�ania lewy kierunkowskaz
#define KEY7  (1<<PA5)		  /// Klawisz do wysy�ania lewy kierunkowskaz
#define KEY8  (1<<PC7)		  /// klawisz do syngan�y on/off
#define KEY9  (1<<PC6)		  /// Klawisz do led on/off
#define KEY10 (1<<PC5)		  /// klawisz do maszt on/off
#define KEY11 (1<<PC1)		  /// Klawisz do �wiat�a on/off
#define KEY12 (1<<PD6)		  /// Klawisz do awaryne on/off

void przod(void) {				// funkcja do jazdy do przodu
	uart_puts("przod");
	uart_puts("\r\n");

}
void tyl(void) {				// funkcja do jazdy do ty�u
	uart_puts("tyl");
	uart_puts("\r\n");

}
void stop(void) {				// funkcja do stopu
	uart_puts("stop");
	uart_puts("\r\n");

}
void masztup(void) {			// funkcja do podnoszenia masztu
	uart_puts("masztup");
	uart_puts("\r\n");

}
void masztdown(void) {			// funkcja do opuszczania masztu
	uart_puts("masztdown");
	uart_puts("\r\n");

}
void masztwait(void) {			// funkcja do zatrzymywania masztu
	uart_puts("masztwait");
	uart_puts("\r\n");

}
void prawymigacz(void) {		// funkcja do prawy kierunkowskaz
	uart_puts("prawymigacz");
	uart_puts("\r\n");

}
void lewymigacz(void) {			// funkcja do lewy kierunkowskaz
	uart_puts("lewymigacz");
	uart_puts("\r\n");

}
void migaczoff(void) {			// funkcja do wy��czania kierunkowskazu
	uart_puts("migaczoff");
	uart_puts("\r\n");

}
void dzialkoon(void) {			// funkcja do dzia�ko on
	uart_puts("dzialkoon");
	uart_puts("\r\n");

}
void dzialkooff(void) {			// funkcja do dzialko off
	uart_puts("dzialkooff");
	uart_puts("\r\n");

}
void sygnalyon(void) {			// funkcja do sygna�y on
	uart_puts("sygnalyon");
	uart_puts("\r\n");

}
void sygnalyoff(void) {			// funkcja do sygna�yoff
	uart_puts("sygnalyoff");
	uart_puts("\r\n");

}
void ledon(void) {			    // funkcja do kogut�w on
	uart_puts("ledon");
	uart_puts("\r\n");

}
void ledoff(void) {				// funkcja do kogut�w off
	uart_puts("ledoff");
	uart_puts("\r\n");

}
void maszton(void) {			// funkcja do maszt on
	uart_puts("maszton");
	uart_puts("\r\n");

}
void masztoff(void) {			// funkcja do maszt off
	uart_puts("masztoff");
	uart_puts("\r\n");

}
void swiatlaon(void) {			// funkcja do w��czania �wiate�
	uart_puts("swiatlaon");
	uart_puts("\r\n");

}
void swiatlaoff(void) {			// funkcja do wy��czania �wiate�
	uart_puts("swiatlaoff");
	uart_puts("\r\n");

}
void awaryjneon(void) {			// funkcja do w�aczania awaryjnych
	uart_puts("awaryjneon");
	uart_puts("\r\n");

}
void awaryjneoff(void) {		// funkcja do wy��czania awaryjnych
	uart_puts("awaryjneoff");
	uart_puts("\r\n");

}

//*****************************POCZATEK PROGRAMU**************************************

int main(void){

	USART_Init( __UBRR );
	register_uart_str_rx_event_callback(analizuj_dane_rs232);

	_delay_ms(10);


	//// inicjalizacja ADC ////
		ADCSRA |= (1<<ADEN);							 // w��czenie modu�u ADC
		ADCSRA |= (1<<ADPS1) | (1<<ADPS2); 				 // ustawienie prescalera na 64// 125 khz  powinno byc od 50 do 200
		ADMUX |= (1<<REFS0); 							 // Napi�cie odniesienia w postaci VCC				 // Napi�cie odniesienia 2.56 V
		ADMUX |= (1<<ADLAR); 							 // odczyt pomiaru na 8 bitach od 0-255


	// ZMIENNE DO OBS�UGI KLAWISZY
		PORTB |= KEY1|KEY2; 	  // podci�gamy linie klawiszy do VCC
		PORTC |= KEY3|KEY4|KEY6|KEY8|KEY9|KEY10|KEY11;  // podci�gamy linie klawiszy do VCC
		PORTD |= KEY5|KEY12;			  // podci�gamy linie klawiszy do VCC
		PORTA |= KEY7; 	  // podci�gamy linie klawiszy do VCC


	//*** zmienna dla pomiar�w ADC
		uint16_t moc;		//zmiiennna do wysy�ania mocy silnika tylko je�eli sie zmieni�a
		uint16_t obroty;	// zmienna do PWM SILNIKA
		uint16_t kierunek;	// Zmienna do PRAWO LEWO AUTA
		uint16_t lewo;		// zmienna do wysy�ania lewo- prawo tylko jesli sie zmieni�a wartosc pot...
		uint8_t petla;		// zwalnia wysy�anie o PWM potrzebna po to �eby nie blokowa odbiornika zbyt czestym wysy�aniem
		uint16_t maszt;		// zmienna do lewo/prawo maszt
		uint16_t lpmaszt;   // zmienna do wysy�ania lewo/prawo maszt tylko je�eli si� zmieni pomiar z ADC
		uint16_t dzialko;	// zmienna do lewo/prawo dzialko
		uint16_t lpdzialko; // zmienna do wysy�ania lewo/prawo dzialko tylko je�eli si� zmieni pomiar z ADC
		uint16_t zdzialko;	// zmienna do lewo/prawo maszt
		uint16_t zgddzialko;// zmienna do wysy�ania lewo/prawo maszt tylko je�eli si� zmieni pomiar z ADC
		obroty = 0;
		kierunek = 0;
		moc = 0;
		lewo = 0 ;
		petla = 0;
		maszt = 0;
		lpmaszt = 0;
		dzialko = 0;
		lpdzialko = 0;
		zdzialko = 0;
		zgddzialko = 0;


	sei();

//************************************ P�TLA G��WNA ******************************************

	while(1){

		UART_RX_STR_EVENT( uart_buf );   // do Rs232 POTRZEBNE


//**************************** p�tla do wysy�ania odczytanych warto�ci Z ADC *******************

		if( petla==1 ){

//*************************** Obroty***********************

		obroty = pomiar(0);		// WYSY� PWM DO OBRTOT�W SILNIKA (PORTA 0)

		if(obroty != moc){
			uart_puts("silnik,");
			uart_puts(int_to_str(obroty, buf, 3));
			uart_puts("\r\n");

		}
		moc = obroty;

//*************************** LEWO/PRAWO*********************

		kierunek = pomiar(1);		// WYSY� PWM DO OBRTOT�W SILNIKA (PORTA 1)

		if(kierunek != lewo){
			uart_puts("lewo,");
			uart_puts(int_to_str(kierunek, buf, 3));
			uart_puts("\r\n");

		}
		lewo = kierunek;

//*************************** maszt lewo/prawo*********************

		maszt = pomiar(2);		// WYSY� PWM DO MASZTU LEWO/PRAWO (PORTA 2)

		if(maszt != lpmaszt){
			uart_puts("maszt,");
			uart_puts(int_to_str(maszt, buf, 3));
			uart_puts("\r\n");

		}
		lpmaszt = maszt;

//***************************DZIA�KO LEWO/PRAWO*********************

		dzialko = pomiar(4);		// WYSY� PWM DO DZIA�KO LEWO/PRAWO (PORTA 4)

		if(dzialko != lpdzialko){
			uart_puts("dzialkolp,");
			uart_puts(int_to_str(dzialko, buf, 3));
			uart_puts("\r\n");

		}
		lpdzialko = dzialko;

//***************************DZIA�KO G�RA/Dӣ*********************

		zdzialko = pomiar(3);		// WYSY� PWM DO DZIA�KO G�RA/Dӣ (PORTA 3)

		if(zdzialko != zgddzialko){
			uart_puts("dzialkogd,");
			uart_puts(int_to_str(zdzialko, buf, 3));
			uart_puts("\r\n");

		}
		zgddzialko = zdzialko;

//************************************************************

		}	// koniec od if(petla)

		petla ++;
//		if (petla >100 ) petla = 0;  jak by za wolno wysy�a�o

//************************* koniec p�tli do wysy�ania ADC*********************

		// PRZ�D/STOP/TY�
		key_press( &key1_lock, &PINB, KEY1, tyl );		// cofanie
		key_push_up( &key2_lock, &PINB, KEY1, stop );	// Stop
		key_press( &key3_lock, &PINB, KEY2, przod );	// prz�d
		key_push_up( &key4_lock, &PINB, KEY2, stop );	// Stop
		// MASZT UP/WAIT/DOWN
		key_press( &key5_lock, &PINC, KEY3, masztup );		// g�ra
		key_push_up( &key6_lock, &PINC, KEY3, masztwait );	// Stop
		key_press( &key7_lock, &PINC, KEY4, masztdown );	// d�
		key_push_up( &key8_lock, &PINC, KEY4, masztwait );	// Stop
		key_press( &key19_lock, &PINC, KEY10, maszton );	// maszt on
		key_push_up( &key20_lock, &PINC, KEY10, masztoff );	// masztoff
		// Kierunkowskaz prawy/lewy
		key_press( &key9_lock, &PIND, KEY5, prawymigacz );		// prawy kierunkowskaz
		key_push_up( &key10_lock, &PIND, KEY5, migaczoff );	    // kierunkowskaz off
		key_press( &key11_lock, &PINC, KEY6, lewymigacz );		// lewy kierunkowskaz
		key_push_up( &key12_lock, &PINC, KEY6, migaczoff );		// kierunkowskaz off
		// Dzia�ko onn/off
		key_press( &key13_lock, &PINA, KEY7, dzialkoon );		// dzialko on
		key_push_up( &key14_lock, &PINA, KEY7, dzialkooff );    // dzialko off
		// Modulator
		key_press( &key15_lock, &PINC, KEY8, sygnalyon );		// sygnalyon
		key_push_up( &key16_lock, &PINC, KEY8, sygnalyoff );	// sygnalyoff
		key_press( &key17_lock, &PINC, KEY9, ledon );			// koguty on
		key_push_up( &key18_lock, &PINC, KEY9, ledoff );		// koguty off
		// Pozosta�e �wiat�a i awaryjne
		key_press( &key21_lock, &PINC, KEY11, swiatlaon );		// �wiat�a on
		key_push_up( &key22_lock, &PINC, KEY11, swiatlaoff );	// �wiat�a off
		key_press( &key23_lock, &PIND, KEY12, awaryjneon );		// awaryjne on
		key_push_up( &key24_lock, &PIND, KEY12, awaryjneoff );	// awaryjne off
}
}

//************************************ KONIEC P�TLI G��WNEJ **********************************


//************************************ FUNKCJA DO OBS�UGI ODEBRANYCH DANYCH ******************

void analizuj_dane_rs232(char *buf){



}

//************************************ FUNKCJA DO POMIARU ADC ********************************

uint16_t pomiar(uint8_t kanal){ 					 // funkcja w kt�rej dokonujemy pomiaru na wybranym wejciu(kanale)

	ADMUX = (ADMUX & 0xF8) | kanal;		// Ustawienie  wybranego kana�u ADC w rejestrze ADMUX
	ADCSRA |= (1<<ADSC);				// Start pomairu
	while( ADCSRA & (1<<ADSC) );		// oczekiwanie na koniec pomiaru
	return ADCH;						// wynik w postaci 8-bitowej

}


//************************ funkcji do konwersji liczby na �a�cuch tekstowy ASCII *******************

char *int_to_str(int val, char *str, int8_t fw) {

	char *strp = str;

	do{

		div_t divmod = div(val, 10);

		if( (val==0) && (strp != str)) {
			break;
		}	else{
			*strp++ = divmod.rem + '0';
		}

		val = divmod.quot;

		if(fw) fw--;

	} while ( (fw>0));

	// *(++strp); // nw czy to jest potrzebne
	 *strp = 0;

	 strrev(str);

	 return str;

	}

////////////////// REAKCJA  NA WCISNI�CIE I ZWOLNINIE KLAWISZA
void key_press(uint8_t * klock, volatile uint8_t * KPIN, uint8_t key_mask, void (*kfun)(void) ) {

 register uint8_t key_press = (*KPIN & key_mask);

 if( !*klock && !key_press ) {
  *klock=1;

  // reakcja na PRESS (wcini�cie przycisku)
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
