/* 
 * File:   controller_p2.c
 * Author: Carlos
 *
 * Created on 2 de noviembre de 2022, 08:05 PM
 */
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "osc.h"
//CONSTANTES
#define _XTAL_FREQ 8000000
//VARIABLES
uint8_t PB1 = 0;
uint8_t PB2 = 0;
uint8_t PB3 = 0;
//PROTOTIPO DE FUNCIONES
void setup(void);
//INTERRUPCIONES
void __interrupt() isr(){
    if(RBIF){
        if (PORTBbits.RB0 == 0){
            PB1 = 1; // Valor de cuando el botón 1 esta presionado (Izquierda)
            PORTD = PB1; // Enviando el estado del botón al puerto D
            PORTAbits.RA0 = 1;
        }
        else if (PORTBbits.RB1 == 0){
            PB2 = 2; // Valor cuando el botón 2 esta presionado (Derecha)
            PORTD = PB2; // Enviando el estado del botón al puerto D
            PORTAbits.RA1 = 1;
        }
        else if (PORTBbits.RB2 == 0){
            PB3 = 3; // Valor cuando el botón 3 esta presionado (Disparo)
            PORTD = PB3; // Enviando el estado del botón al puerto D
            PORTAbits.RA2 = 1;
        }
        else{
            //Limpieza de variables y banderas
            PB1 = 0;
            PB2 = 0;
            PB3 = 0;
            PORTD = 0;
            PORTA = 0;
        }
        RBIF = 0;
    }
    return;
}
//CÓDIGO PRINCIPAL
void main(void){
    setup();
    while(1){
        //Bloques de error del USART
        if(RCSTAbits.FERR){
            PORTEbits.RE0 = 1;
        }
        else{ 
            PORTEbits.RE0 = 0;
        }
        if(RCSTAbits.OERR){
            PORTEbits.RE1 = 1;
        }
        else{
            PORTEbits.RE1 = 0;
        }
        //Loop principal
        //Protocolo de envío de datos
        TXREG = PORTD;//Envió del estado actual de los botones del control
        __delay_ms(10);
    }
}
//FUNCIONES
void setup(void){
    //I/O digitales
    ANSEL = 0;
    ANSELH = 0;
    TRISD = 0;
    TRISA = 0;
    TRISE = 0;
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    
    PORTA = 0;
    PORTB = 0;
    PORTD = 0;
    PORTE = 0;
    //Configuraciónd de pull up en el puerto B
    OPTION_REGbits.nRBPU = 0;
    WPUBbits.WPUB0 = 1;
    WPUBbits.WPUB1 = 1;
    WPUBbits.WPUB2 = 1;
    IOCBbits.IOCB0 = 1;
    IOCBbits.IOCB1 = 1;
    IOCBbits.IOCB2 = 1;
    RBIE = 1;
    //Iniciación del oscilador
    int_osc_MHz(8);
    //Configuración del generador de Baudíos (9600)
    BRG16 = 0;
    BRGH = 1;
    SYNC = 0;
    SPBRG = 51;
    //Configuración del USART (Transmisor)
    SPEN = 1;
    TX9 = 0;
    TXEN = 1;
    //Limpieza de banderas
    RBIF = 1;
    //Habilitación de interrupciones generales y perifericas
    PEIE = 1;
    GIE = 1;
}
