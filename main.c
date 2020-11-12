/*
 * Autor: P.Dabrowa
 */

//dziala tryb sleep mode, wyslal sms
//jak nie wysylam smsa to sie nie resetuje, jak wysylam z urzadzenia to sie resetuje urzadzenie

//podlaczony czujnik cisnienia, czujnik wilgotnosci i czujniki temperatury


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>


#include "LCD/LCD_16x2_H_file.h"
#include "USART/USART_Interrupt.h"
#include "1Wire/ds18x20.h"
#include "I2C_TWI/i2c_twi.h"
#include "BMP180/BMP180.h"

/////////////GSM G510_V2
void GSM_Inicjalizacja(void);

void GSM_Odpowiedz();
void GSM_Response_Display();
void GSM_Msg_Read(int);
bool GSM_Wait_for_Msg();
void GSM_Msg_Display();
void GSM_Send_Msg(char* , char*);


char buffor[160];								// bufor do odpowiedzi i wiadomosci
char status_flag = 0;						// do sprawdzania nowych wiadomosci
volatile int buffer_pointer;
char Mobile_no[14];							// numer telefonu
char message_received[10];					// zapisana nowa wiadomosc
int position = 0;							// lokacja nowej wiadomosci

//DHT11
#define DHT11_PIN 0
volatile uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

void Request();
void Response();
uint8_t Receive_data();
void display_humidity();
void get_humidity(void);

////////////DS18B20
void display_temp(uint8_t x);

uint8_t czujniki_cnt;		/* iloœæ czujników na magistrali */
volatile uint8_t s1_flag;	/* flaga tykniêcia timera co 1 sekundê */
volatile uint8_t sekundy;	/* licznik sekund 0-59 */

uint8_t subzero, cel, cel_fract_bits;

//BMP180
void display_pressure(void);
long press, temp;

int main(void)
{
	buffer_pointer = 0;
	//int is_msg_arrived;
	memset(message_received, 0, 10);
    LCD_Init();
	USART_Init(9600);						    // inicjalizacja usarta

	//DHT11
	get_humidity();

	//BMP180
	i2cSetBitrate(100);		// inicjalizacja I2C
	BMP180_init();			// inicjalizacja BMP180 - pobranie danych calibracyjnych

	/* ustawienie TIMER0 dla F_CPU=11,0592MHz */
	TCCR0 |= (1<<WGM01);				/* tryb CTC */
	TCCR0 |= (1<<CS02)|(1<<CS00);		/* preskaler = 1024 */
	OCR0 = 107;							/* dodatkowy podzia³ przez 108 (rej. przepe³nienia) */
	TIMSK |= (1<<OCIE0);				/* zezwolenie na przerwanie CompareMatch */
	/* przerwanie wykonywane z czêstotliwoœci¹ ok 10ms (100 razy na sekundê) */
	/* do naszych celów nie musi byæ to bardzo dok³adne 10ms */

	czujniki_cnt = search_sensors();		 // sprawdzamy ile czujników DS18xxx widocznych jest na magistrali

	/* czekamy 750ms na dokonanie konwersji przez pod³¹czone czujniki */
	_delay_ms(750);


	sei(); //globalne odblokowanie przerwañ
	LCD_String_xy(1,0,"Inzynierka v_f");
	_delay_ms(3000);
	//LCD_Clear();
	//LCD_String_xy(1,0,"AT v2");
	GSM_Inicjalizacja();								// sprawdzanie lacznosci gsm i inicjalizacja gsm
	LCD_Clear();

	status_flag=0;

	 set_sleep_mode(SLEEP_MODE_IDLE);
	 sleep_enable();

	while (1)
	{
		sleep_mode();



		//sprawdzanie czy przyszla nowa wiadomosc
		if(status_flag == 1)
		{
			LCD_Clear();
			LCD_String_xy(1,0,"nowa wiadomosc");		//nowa wiadomosc
			_delay_ms(1000);
			LCD_Clear();

			//sprawdzenie czy jest jakas nowa wiadomosc -> at+cmgr=1 (odczytuje z pamieci wiad o num 1)
			//jesli nie ma to wyskoczy error, jak nie to odczyta
			//odczytanie wiadomosci
			LCD_String_xy(1,0,"AT+CMGR=1");
			USART_SendString("AT+CMGR=1\r");
			_delay_ms(2000);

			if(strstr(buffor,"OK"))
			{
				GSM_Odpowiedz();
				memset(buffor,0,160);
				GSM_Msg_Read(1);

				if(strstr(message_received,"Meas"))
				{
					temp = BMP180_gett();
					press = BMP180_getp();
					display_pressure();
					display_humidity();

					LCD_Clear();
					_delay_ms(2000);

					GSM_Msg_Read(position);					//odczytanie nowej wiadomosci
					_delay_ms(3000);

					char measurement[50];
					sprintf(measurement,"Temperature: \%d.\%d C \n"
										"Humidity: \%d %% RH\n"
										"Pressure: \%ld hPA",
										cel, cel_fract_bits,
										I_RH,
										press/98);

					//LCD_Clear();
					//LCD_String_xy(1,0,Mobile_no);
					//_delay_ms(2000);

					//wysy³anie smsa na moj numer
					LCD_String_xy(1,0,"Wysylanie sms");
					//GSM_Send_Msg("791279407",measurement);
					GSM_Send_Msg(Mobile_no,measurement);
					_delay_ms(3000);

					LCD_Clear();
					//break;
				}
			}
			else
			{
				LCD_String("Error");
			}

			LCD_String_xy(2,0,"usuwanie");
			USART_SendString("AT+CMGD=1,4\r");
			_delay_ms(2000);
			status_flag=0;
			LCD_Clear();

		}
		memset(Mobile_no, 0, 12);
		memset(message_received, 0, 10);

		if(s1_flag) {	/* sprawdzanie flagi tykniêæ timera programowego co 1 sekundê */

					/* co trzy sekundy gdy reszta z dzielenia modulo 3 == 0 sprawdzaj iloœæ dostêpnych czujników */
					if( 0 == (sekundy%3) ) {

						uint8_t *cl=(uint8_t*)gSensorIDs;	// pobieramy wskaŸnik do tablicy adresów czujników
						for( uint8_t i=0; i<MAXSENSORS*OW_ROMCODE_SIZE; i++) *cl++ = 0; // kasujemy ca³¹ tablicê
						czujniki_cnt = search_sensors();	// ponownie wykrywamy ile jest czujników i zape³niamy tablicê
					}

					/* co trzy sekundy gdy reszta z dzielenia modulo 3 == 1 wysy³aj rozkaz pomiaru do czujników */
					if( 1 == (sekundy%3) ) DS18X20_start_meas( DS18X20_POWER_EXTERN, NULL );

					/* co trzy sekundy gdy reszta z dzielenia modulo 3 == 2 czyli jedn¹ sekundê po rozkazie konwersji
					 *  dokonuj odczytu i wyœwietlania temperatur z 2 czujników jeœli s¹ pod³¹czone, jeœli nie
					 *  to poka¿ komunikat o b³êdzie
					 */
					if( 2 == (sekundy%3) ) {
						if( DS18X20_OK == DS18X20_read_meas(gSensorIDs[0], &subzero, &cel, &cel_fract_bits) ) display_temp(0);
						else {
							LCD_String_xy(2,0,"Error");
						}

						if( DS18X20_OK == DS18X20_read_meas(gSensorIDs[1], &subzero, &cel, &cel_fract_bits) ) display_temp(9);
						else {
							LCD_String_xy(2,9,"Error");
						}
					}

					/* zerujemy flagê aby tylko jeden raz w ci¹gu sekundy wykonaæ operacje */
					s1_flag=0;

				} /* koniec sprawdzania flagi */

	}
}



void GSM_Inicjalizacja()
{
	while(1)
	{
		USART_SendString("AT\r");
		_delay_ms(2000);
		USART_SendString("ATE0\r"); //wylaczenie echa
		_delay_ms(2000);
		if(strstr(buffor,"OK"))
		{
			GSM_Odpowiedz();
			memset(buffor,0,160);
			break;
		}
	}
	_delay_ms(1000);

	//wlaczanie trybu tekstowego wysylania i odbierania wiadomosci
	USART_SendString("AT+CMGF=1\r");
	_delay_ms(1000);

	//ustawienie powiadamiania o nadchodzacym smsie
	USART_SendString("AT+CNMI=1,1,0,0,1\r");
	_delay_ms(1000);

	//wlaczenie trybu uspienia
	USART_SendString("ATS24=5\r");
	_delay_ms(2000);

	//usuniecie wszystkich wiadomosci z pamieci karty SIM
	USART_SendString("AT+CMGD=1,4\r");
	_delay_ms(2000);



}

bool GSM_Wait_for_Msg()
{
	char msg_location[4];
	int i;
	_delay_ms(500);
	buffer_pointer=0;

	while(1)
	{
		if(buffor[buffer_pointer]=='\r' || buffor[buffer_pointer]== '\n') /*eliminate "\r \n" which is start of string */
		{
			buffer_pointer++;
		}
		else
			break;
	}

	if(strstr(buffor,"CMTI:"))                                          /* "CMTI:" to check if any new message received */
	{
		while(buffor[buffer_pointer]!= ',')
		{
			buffer_pointer++;
		}
		buffer_pointer++;

		i=0;
		while(buffor[buffer_pointer]!= '\r')
		{
			msg_location[i]=buffor[buffer_pointer];				      /* copy location of received message where it is stored */
			buffer_pointer++;
			i++;
		}

		/* convert string of position to integer value */
		position = atoi(msg_location);

		memset(buffor,0,strlen(buffor));
		buffer_pointer=0;

		return true;
	}
	else
	{
		return false;
	}
}


void GSM_Send_Msg(char *num,char *sms)
{
	char sms_buffer[35];
	buffer_pointer=0;
	sprintf(sms_buffer,"AT+CMGS=\"%s\"\r",num);
	USART_SendString(sms_buffer);                       /*send command AT+CMGS="Mobile No."\r */
	_delay_ms(200);
	while(1)
	{
		if(buffor[buffer_pointer]==0x3e)                  /* wait for '>' character*/
		{
			buffer_pointer = 0;
			memset(buffor,0,strlen(buffor));
			USART_SendString(sms);                      /* send msg to given no. */
			USART_TxChar(0x1a);                         /* send Ctrl+Z then only message will transmit*/
			break;
		}
		buffer_pointer++;
	}
	_delay_ms(300);
	buffer_pointer = 0;
	memset(buffor,0,strlen(buffor));
	memset(sms_buffer,0,strlen(sms_buffer));
}

void GSM_Odpowiedz()
{
	unsigned int timeout=0;
	int CRLF_Found=0;
	char CRLF_buff[2];
	int Response_Length=0;
	while(1)
	{
		if(timeout>=60000)								/*if timeout occur then return */
		return;
		Response_Length = strlen(buffor);
		if(Response_Length)
		{
			_delay_ms(2);
			timeout++;
			if(Response_Length==strlen(buffor))
			{
				for(int i=0;i<Response_Length;i++)
				{
					memmove(CRLF_buff,CRLF_buff+1,1);
					CRLF_buff[1]=buffor[i];
					if(strncmp(CRLF_buff,"\r\n",2))
					{
						if(CRLF_Found++==2)				/* search for \r\n in string */
						{
							GSM_Response_Display();		/* display response */
							return;
						}
					}

				}
				CRLF_Found = 0;

			}

		}
		_delay_ms(1);
		timeout++;
	}
	status_flag=0;
}


void GSM_Response_Display()
{
	buffer_pointer = 0;
	int lcd_pointer = 0;
	while(1)
	{
		if(buffor[buffer_pointer]== '\r' || buffor[buffer_pointer]== '\n')
		{
			buffer_pointer++;
		}
		else
			break;
	}


	LCD_Command(0xc0);
	while(buffor[buffer_pointer]!='\r')
	{
		LCD_Char(buffor[buffer_pointer]);
		buffer_pointer++;
		lcd_pointer++;
		if(lcd_pointer==15)
		LCD_Command(0x80);
	}
	buffer_pointer=0;
	memset(buffor,0,strlen(buffor));
}

void GSM_Msg_Read(int position)
{
	char read_cmd[10];
	sprintf(read_cmd,"AT+CMGR=%d\r",position);
	USART_SendString(read_cmd);							/* read message at specified location/position */
	GSM_Msg_Display();									/* display message */
}


void GSM_Msg_Display()
{
	_delay_ms(500);
	if(!(strstr(buffor,"+CMGR")))                         /*check for +CMGR response */
	{
		//LCD_String_xy(1,0,"No message");
	}
	else
	{
		buffer_pointer = 0;

		while(1)
		{
			if(buffor[buffer_pointer]=='\r' || buffor[buffer_pointer]== 'n')  /*wait till \r\n not over*/
			{
				buffer_pointer++;
			}
			else
			break;
		}

		/* search for 1st ',' to get mobile no.*/
		while(buffor[buffer_pointer]!=',')
		{
			buffer_pointer++;
		}
		buffer_pointer = buffer_pointer+2;

		/* extract mobile no. of message sender */
		for(int i=0;i<=11;i++)
		{
			Mobile_no[i] = buffor[buffer_pointer];
			buffer_pointer++;
		}

		do
		{
			buffer_pointer++;
		}while(buffor[buffer_pointer-1]!= '\n');

		//LCD_Command(0xC0);
		int i=0;

		/* display and save message */
		while(buffor[buffer_pointer]!= '\r' && i<31)
		{
				//LCD_Char(buffor[buffer_pointer]);
				message_received[i]=buffor[buffer_pointer];

				buffer_pointer++;
				i++;
				//if(i==16)
					//LCD_Command(0x80);						/* display on 1st line */
		}

		buffer_pointer = 0;
		memset(buffor,0,strlen(buffor));
	}
	status_flag = 0;
}

void Request()						/* Microcontroller send start pulse or request */
{
	DDRB |= (1<<DHT11_PIN);
	PORTB &= ~(1<<DHT11_PIN);		/* set to low pin */
	_delay_ms(20);					/* wait for 20ms */
	PORTB |= (1<<DHT11_PIN);		/* set to high pin */
}

void Response()						/* receive response from DHT11 */
{
	DDRB &= ~(1<<DHT11_PIN);
	while(PINB & (1<<DHT11_PIN));
	while((PINB & (1<<DHT11_PIN))==0);
	while(PINB & (1<<DHT11_PIN));
}

uint8_t Receive_data()							/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PINB & (1<<DHT11_PIN)) == 0);	/* check received bit 0 or 1 */
		_delay_us(30);
		if(PINB & (1<<DHT11_PIN))				/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);						/* then its logic HIGH */
		else									/* otherwise its logic LOW */
		c = (c<<1);
		while(PINB & (1<<DHT11_PIN));
	}
	return c;
}

void get_humidity(void)
{
	Request();				/* send start pulse */
	Response();				/* receive response */
	I_RH=Receive_data();	/* store first eight bit in I_RH */
	D_RH=Receive_data();	/* store next eight bit in D_RH */
	I_Temp=Receive_data();	/* store next eight bit in I_Temp */
	D_Temp=Receive_data();	/* store next eight bit in D_Temp */
	CheckSum=Receive_data();/* store next eight bit in CheckSum */
}

void display_humidity()
{
	char data[5];


	if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
	{
		LCD_String_xy(1,12,"Error");
	}

	else
	{
		itoa(I_RH,data,10);
		LCD_String_xy(1,10,data);
		LCD_String(".");

		itoa(D_RH,data,10);
		LCD_String(data);
		LCD_String("%");
	}

	_delay_ms(1000);
}


void display_pressure(void)
{
	char bufor[50];
	sprintf(bufor,"%ld",press/98);
	LCD_String_xy(1,0,bufor);
	LCD_String("hPa");

}

/* wyœwietlanie temperatury na pozycji X w drugiej linii LCD */
void display_temp(uint8_t x) {
	if(subzero)
	{
		LCD_String_xy(2,x,"-"); 	//wstawianie "-", temperatura ujemna
	}
	else
	{
		LCD_String_xy(2,x," "); 	//pusty znak jesli temperatura wieksza >= 0
	}
	char jednosci[2];
	sprintf(jednosci,"%d",cel);
	LCD_String_xy(2,x+1,jednosci);
	//lcd_int(cel);	/* wyœwietl dziesiêtne czêœci temperatury  */
	LCD_String_xy(2,x+3,".");
	char czesci_dziesietne[2];
	sprintf(czesci_dziesietne,"%d",cel_fract_bits);
	LCD_String_xy(2,x+4,czesci_dziesietne);
	LCD_String_xy(2,x+6,"C"); /* wyœwietl znak jednostek (C - stopnie Celsiusza) */
}


/* ================= PROCEDURA OBS£UGI PRZERWANIA – COMPARE MATCH */
/* pe³ni funkcjê timera programowego wyznaczaj¹cego podstawê czasu = 1s */
ISR(TIMER0_COMP_vect)
{
	static uint8_t cnt=0;	/* statyczna zmienna cnt do odliczania setnych ms */

	if(++cnt>99) {	/* gdy licznik ms > 99 (minê³a 1 sekunda) */
		s1_flag=1;	/* ustaw flagê tykniêcia sekundy */
		sekundy++;	/* zwiêksz licznik sekund */
		if(sekundy>59) sekundy=0; /* jeœli iloœæ sekund > 59 - wyzeruj */
		cnt=0;	/* wyzeru licznik setnych ms */
	}
}

// przerwanie od odbioru danej
ISR(USART_RXC_vect)
{
	buffor[buffer_pointer] = UDR;
	buffer_pointer++;
	status_flag = 1;
}


//GSM-msg read
	//sprintf(read_cmd,"AT+CMGR=%d\r",position);
	//USART_SendString(read_cmd);
	//GSM_Msg_Display();

