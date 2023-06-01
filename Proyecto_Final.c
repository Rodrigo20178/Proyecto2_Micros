/*
 * File:   Proyecto_Final.c
 * Author: Rodrigo García
 *
 * Created on 18 de mayo de 2023, 11:38 AM
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
#include <stdint.h>
#include<stdio.h>
#include <stdlib.h>
#include <string.h>

//Constantes

#define _XTAL_FREQ 4000000          // Frecuencia oscilador (4MHz)



//Variables
uint8_t cont;
uint8_t Pot3_valor;
uint8_t pot3;
uint8_t Pot4_valor;
uint8_t pot4;



//Prototipo de Funciones
void setup (void);
void tmr0_setup(void);

unsigned char mapeo(unsigned char pot);

//Interrupción
void __interrupt() isr (void)
{
    if (PIR1bits.ADIF)  // Interrupción del ADC
    {
        if (ADCON0bits.CHS == 0b0000){
            CCPR1L = (ADRESH>>1)+40;
            CCP1CONbits.DC1B1 = ADRESH & 0b01;
            CCP1CONbits.DC1B0 = ADRESL >> 7;
         }
           
        else if (ADCON0bits.CHS == 0b0001) {
            CCPR2L = (ADRESH>>1)+40;
            CCP2CONbits.DC2B1 = ADRESH & 0b01;
            CCP2CONbits.DC2B0 = ADRESL >> 7;
        }
        else if (ADCON0bits.CHS == 0b0010){
            Pot3_valor = mapeo(ADRESH);
            pot3 = (uint8_t)(Pot3_valor>>2);
            
        }
        else if (ADCON0bits.CHS == 0b0011){
            Pot4_valor = mapeo(ADRESH);
            pot4 = ADRESH;
        }
        PIR1bits.ADIF = 0; 
    }
    if (INTCONbits.T0IF)    // Interrupción del TMR0
    {
        cont++;
        if (cont == Pot3_valor){  //Si el contador es igual al mapeo del Pot3, apagar el RC3
            PORTCbits.RC3 = 0;
        }
        if (cont == Pot4_valor){  //Si el contador es igual al mapeo del Pot4, apagar el RC4
            PORTCbits.RC4 = 0; 
        }
        if (cont == 40){         //Si el contador es igual a 4 ms, encender RC3 y/o RC4
            PORTCbits.RC3 = 1;
            PORTCbits.RC4 = 1;
            cont = 0;           //Reinicio del contador
        }
        INTCONbits.T0IF = 0;
        TMR0 = 240;
    }
    
}
//MAIN
void main(void) {
    setup();
    tmr0_setup();
    
    while(1){
        if (ADCON0bits.GO == 0) {
            switch (ADCON0bits.CHS){
                case(0):
                    ADCON0bits.CHS = 0b0001;   //Se cambia al segundo canal
                    break;
                case(1):
                    ADCON0bits.CHS = 0b0010;   //Se cambia al tercer canal
                    break;
                case(2):
                    ADCON0bits.CHS = 0b0011;   //Se cambia al cuarto canal
                    break;
                case (3):
                    ADCON0bits.CHS = 0b0000;   //Se cambia al primer canal
                    break;
            }
            __delay_us(40);
            ADCON0bits.GO = 1;
        }
    }        
    return;
}

//Configuraciones
void setup (void){
    
   // Configuración de los puertos
    ANSEL = 0b00001111;          //AN0, AN1, AN2, AN3 como entradas analógicas
    ANSELH = 0;
    
    TRISA = 0b00001111;         // PORTA como salida, RA0 & RA1 como entradas 
    PORTA = 0;                  // Limpiamos PORTA 
    
    TRISC = 0;                  // PORTC como salida analógica
    PORTC = 0;                  // Limpiamos PORTC
    
    // Configuración del oscilador
    OSCCONbits.IRCF = 0b0110;    // IRCF <2:0> -> 4 MHz
    OSCCONbits.SCS = 1;         // Oscilador interno
    
    
    
    //COnfiguración del ADC
    ADCON1bits.ADFM = 0;        // Justificado a la izquierda
    ADCON1bits.VCFG0 = 0;       // VDD
    ADCON1bits.VCFG1 = 0;       // VSS
    
    ADCON0bits.ADCS = 0b10;     // ADCS <1:0> -> 10 FOSC/32
    ADCON0bits.CHS = 0b0000;    // Seleccionamos el AN0
    
    ADCON0bits.ADON = 1;        // Encender ADC
    __delay_us(320);

    
    // Configuración del PWM
    
    TRISCbits.TRISC2 = 1;       // RC2 como entrada
    TRISCbits.TRISC1 = 1;       // RC1 como entrada
    
    CCP1CONbits.P1M = 0;        // Salida simple
    CCP1CONbits.CCP1M = 0b1100; // asignación del modo a PWM1
    CCP2CONbits.CCP2M = 0b1100; // asignación del modo a PWM2
            
    CCPR1L = 0x0F;              // Valor inicial del duty cycle
    CCP1CONbits.DC1B1 = 0;       // CONFIG bits menos significativos
    CCP1CONbits.DC1B0 = 0;
    
    CCPR2L = 0x0F;              // Valor inicial del duty cycle
    CCP2CONbits.DC2B1 = 0;       // CONFIG bits menos significativos
    CCP2CONbits.DC2B0 = 0;       // CONFIG bits menos significativos                
    
    // Configuración del TIMER2
    
    PR2 = 255;                  // Periodo del TIMER2
    T2CONbits.T2CKPS = 0b11;    // Prescaler 1:16
    T2CONbits.TMR2ON = 1;       // Encendemos TIMER2
    PIR1bits.TMR2IF = 0;        // Flag del TIMER2 apagado
    
    while (PIR1bits.TMR2IF == 0); // Esperamos una interrupción del TIMER2
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;       // RC2 como salida del PWM
    TRISCbits.TRISC1 = 0;       // RC1 como salida
    
    //Configuración de las interrupciones
    INTCONbits.GIE = 1;         // Habilitamos interrupciones globales
    INTCONbits.PEIE = 1;        // Habilitamos interrupciones de los puertos
    
    PIE1bits.ADIE = 1;          // Habilitamos interrupciones del ADC
    
    PIR1bits.ADIF = 0;          // Flag del ADC en 0
}

void tmr0_setup(void){
    OPTION_REGbits.T0CS = 0;        // Utilizar ciclo interno
    OPTION_REGbits.PSA = 0;         // 
    OPTION_REGbits.PS0 = 0;         // Prescaler 1:2
    OPTION_REGbits.PS1 = 0;         //
    OPTION_REGbits.PS2 = 0;         //

    INTCONbits.T0IF = 0;            // Interrupciones del TMR0
    INTCONbits.T0IE = 1;
    TMR0 = 240;                // Valor del TMR0
    return;
}



//Interpolación del TMR0
unsigned char mapeo(unsigned char pot){
    unsigned char tmr0reset;
    if (pot <= 13) tmr0reset = 5;
    else if (pot > 13 && pot <= 26) tmr0reset = 6;
    else if (pot > 26 && pot <= 39) tmr0reset = 7;
    else if (pot > 39 && pot <= 62) tmr0reset = 8;
    else if (pot > 52 && pot <= 65) tmr0reset = 9;
    else if (pot > 65 && pot <= 78) tmr0reset = 10;
    else if (pot > 78 && pot <= 91) tmr0reset = 11;
    else if (pot > 91 && pot <= 104) tmr0reset = 12;
    else if (pot > 104 && pot <= 117) tmr0reset = 13;
    else if (pot > 117 && pot <= 130) tmr0reset = 14;
    else if (pot > 130 && pot <= 143) tmr0reset = 15;
    else if (pot > 143 && pot <= 156) tmr0reset = 16;
    else if (pot > 156 && pot <= 169) tmr0reset = 17;
    else if (pot > 169 && pot <= 182) tmr0reset = 18;
    else if (pot > 195 && pot <= 208) tmr0reset = 19;
    else if (pot > 208 && pot <= 221) tmr0reset = 20;
    else if (pot > 221 && pot <= 234) tmr0reset = 21;
    else if (pot > 247 && pot <= 255) tmr0reset = 22;
    else if (pot > 255) tmr0reset = 23;
    
    return tmr0reset;
}

