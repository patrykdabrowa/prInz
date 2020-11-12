/*
 * BMP180.h
 *
 *  Created on: 18 pa� 2018
 *      Author: patry
 */

#ifndef BMP180_BMP180_H_
#define BMP180_BMP180_H_

#define BMP180_ADDR 0xEE  	// Adres I2C
#define BMP180_MODE 3		// oversampling setting (0-3)

void BMP180_init( void );	// Inicjalizacja czujnika
void BMP180_getut(void);	// Odczyt temperatury bez kompensacji
void BMP180_getup(void);	// odczyt ci�nienia bez kompensacji
long BMP180_gett(void);		// Wylicza temperatur�
long BMP180_getp(void);		// Wylicza ci�nienie

#endif /* BMP180_H_ */
