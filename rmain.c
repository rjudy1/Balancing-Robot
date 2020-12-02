
/* 
 * File:   rmain.c
 * Author: Rachael Judy, Jonathan Claire, McKenzie Barlow
 *
 * Created on October 28, 2020, 8:03 PM
 * Balancing robot code
 */

#include <p18cxxx.h>
#include <delays.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


#pragma config FOSC = INTIO67, PLLCFG = OFF, PRICLKEN = ON,FCMEN = ON, PWRTEN = ON
#pragma config BOREN = SBORDIS, BORV = 250, WDTEN = OFF, WDTPS = 4096, PBADEN = OFF
#pragma config HFOFST = OFF, MCLRE = EXTMCLR, STVREN = ON, LVP = OFF, DEBUG = ON

// i2c address for the mpu
#define MPU_Low_Addr 0x68
#define MPU_High_Addr 0x69
#define ACCEL_X_H 0x3B
#define ACCEL_X_L 0x3C
#define ACCEL_Y_H 0x3D
#define ACCEL_Y_L 0x3E
#define ACCEL_Z_H 0x3F
#define ACCEL_Z_L 0x40
#define GYRO_X_H 0x43
#define GYRO_X_L 0x44
#define GYRO_Y_H 0x45
#define GYRO_Y_L 0x46
#define GYRO_Z_H 0x47
#define GYRO_Z_L 0x48
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACC_CONFIG 0x1C
#define INT_EN 0x38
#define PWR_MGMT_1 0x6B
#define SAMPLE_RATE_DIV 0x19
#define MOT_THR 0x1F
#define FIFO_EN 0x23

#define ABS(x) ((x) > 0 ? x : -x)

void DelayFor18TCY(void) {
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
    Nop();
}

void DelayPORXLCD(void) {
    Delay1KTCYx(60); // Delay of 15ms
    // Cycles = (TimeDelay * Fosc) / 4
    // Cycles = (15ms * 16MHz) / 4
    // Cycles = 60,000
    return;
}


void I2C_write(unsigned char reg, unsigned char data)
{
	SSP1CON2bits.SEN = 1; // start bit
	while (SSP1CON2bits.SEN);
    SSP1BUF = MPU_Low_Addr << 1; // write slave address
    while (SSP1STATbits.R_NOT_W);
	//if (SSP1CONbots.ACKSTAT == 1) 
	//	exit(-1);

	SSP1BUF = reg;
	while (SSP1STATbits.R_NOT_W);
	SSP1BUF =data;
	while (SSP1STATbits.R_NOT_W);

	SSP1CON2bits.PEN = 1; // stop bit
	while (SSP1CON2bits.PEN);
}


unsigned char I2C_read(unsigned char reg){   
	SSP1CON2bits.SEN = 1; // start enable
	while (SSP1CON2bits.SEN); // wait for start bit to end
    SSP1BUF = MPU_Low_Addr << 1; // write to this register to clock data out ? writes address ? 0 for write
    while (SSP1STATbits.R_NOT_W);
    // wait for ack here
	//if (SSP1CONbits.ACKSTAT == 1) 
	//	exit(-1);

	SSP1BUF = reg; // register address to read
	while (SSP1STATbits.R_NOT_W); // wait to send
	
    // no restart, just start bit again
//  SSP1CON2bits.RSEN = 1;
//	while (SSP1CON2bits.RSEN);// = 1; // restart enable
    SSP1CON2bits.SEN = 1; // start enable repeat
	while (SSP1CON2bits.SEN); // wait for start bit to end
	SSP1BUF = ((MPU_Low_Addr << 1) | 0x01); // read
	while (SSP1STATbits.R_NOT_W); // wait to send

	SSP1CON2bits.RCEN = 1;
	while (SSP1CON2bits.RCEN); // data on bus, rcv enable
    

	SSP1CON2bits.PEN = 1; // stop bit
	while (SSP1CON2bits.PEN);

	return SSP1BUF; // should return byte at this address
}

// use this function to find the i2c address
unsigned char surf_channels() {
    unsigned char i = 0x68;
    for (i = 0; i < 128; i++) {
        SSP1CON2bits.SEN = 1; // start enable
        while (SSP1CON2bits.SEN); // wait for start bit to end
        SSP1BUF = i << 1; // write to this register to clock data out ? writes address ? 0 for write
        while (SSP1STATbits.R_NOT_W);
        // wait for ack here
        if (SSP1CON2bits.ACKSTAT == 1) 
        	continue;
        return i;
    }
}

void UART_WriteMsg(unsigned char* msg) {
    int i;
    
    for (i = 0; i < strlen(msg); i++) {
        TXREG1 = msg[i];
        Delay10KTCYx(20);
    }
}

void UART_WriteChar(unsigned char msg) {
    TXREG1 = msg;
    Delay10KTCYx(20);
}

// doesn't work right now - needs modification to parse by digit
void UART_WriteFloat(float f) {
    unsigned char space = ' ';
    unsigned char neg = '-';
    unsigned char dot = '.';

    if (f >= 0) {
        UART_WriteChar(space);
        f = f;
    } else {
        UART_WriteChar(neg);
        f = -f;
    }
    
    UART_WriteChar('0' + ((int)f / 100));
    UART_WriteChar('0' + (((int)f / 10) % 10));
    UART_WriteChar('0' + ((int)f % 10));
    UART_WriteChar(dot);
    UART_WriteChar('0' + ((int)(f*10) % 10));
    UART_WriteChar('0' + ((int)(f*100) % 10));    
    UART_WriteChar('0' + ((int)(f*1000) % 10));
}


void UART_Clear() {
    TXREG1 = 0xFE;
    TXREG1 = 0x01;
}

// drive motor takes positive or negative duty cycle and uses PWM2
void drive_motor(float speed) {
//    int THRESH = 77;
    unsigned char value;
    //if(PORTCbits.RC0); speed = 1 + speed;
    PORTCbits.RC0 = (speed < 0); // use C0 for direction
    value = (unsigned char)((PORTCbits.RC0 + speed) * (PR2 + 1) * 4);
    CCPR2L = value >> 2;
    CCP2CON = CCP2CON & (0xCF | (value & 0x03) << 4);

    // alternate value calculation based off 0 to 127 speeds - has some integer multiplication issues
//    value = ((THRESH *128 + ABS(speed) * (127-THRESH)) * (PR2+1) ) >> 12;
}

// on pin C1
void PWM2_Cycle(float duty_cycle) {
    unsigned int value;
    value = (duty_cycle * (PR2 + 1) * 4);
    CCPR2L = value >> 2;
    CCP2CON = CCP2CON & (0xCF | (value & 0x03) << 4);
}

// B5
void PWM3_Cycle(float duty_cycle) {
    unsigned int value;
    value = (duty_cycle * (PR2 + 1) * 4);
    CCPR3L = value >> 2;
    CCP3CON = CCP3CON & (0xCF | (value & 0x03) << 4);
}


void main(void) {
    // variables
    unsigned char counter = 0;
    unsigned char hello[] = "Hello MPU6050";
    unsigned char space[] = " * ";
    signed char i, speed;
    int value;
    unsigned char cycle;
    
    unsigned char channel;
    int Axr, Azr, Gyr; // raw values
    float Axt, Azt, Gyt, angle; // true values
    int THRESH  = 60;
    
    // 64 MHz - timing
    OSCTUNEbits.PLLEN = 1; //no 4X PLL
    OSCCON = 0x7C;    // 16MHZ Operation
    
    // for i2c
    SSP1ADD = 0x9E; // set to 100 kHz (MPU speed), based on 64 MHz Fosc
    SSP1CON1 = 0x28;
    
    // configure pic for output on TX1, input on SDA and SCL, output of PWM
    TRISC = 0x18;          //Make Port C Digital outputs 
    ANSELC = 0x00;          //Disable Analog Inputs
    TRISB = 0x00;          //Make Port C Digital outputs 
    ANSELB = 0x00;          //Disable Analog Inputs
    
    // configure for display - not used in final product
//    TXSTA1bits.TXEN = 1; // set TXEN to 1
//    TXSTA1bits.SYNC = 0; // set SYNC to 0
//    RCSTA1bits.SPEN = 1;
//    SPBRGH1 = 0x00;
//    SPBRG1 = 0x67; // set to 9600 baud based on 64 MHz
//    Delay10KTCYx(1000); // allow set up time
    
    // configure PIC for PWM - using 2 and 3
    CCPTMRS0 = 0x00;
    CCPTMRS1 = 0x00; // use TMR2
    T2CON = 0x06; // x16 prescaler

    CCP2CON = 0x0C; // PWM mode
    CCP3CON = 0x0C;

    PR2 = 0x37;// 20 kHz PWM period

    // configure I2C registers
    I2C_write(SAMPLE_RATE_DIV, 0x00);
    I2C_write(PWR_MGMT_1, 0x00);
    I2C_write(CONFIG, 0x00);
    I2C_write(GYRO_CONFIG, 0x00); // highest resolution    
    I2C_write(ACC_CONFIG, 0x00);
    I2C_write(INT_EN, 0x01);
    I2C_write(FIFO_EN, 0x00);
    
    
    while (1) {
        // iterating over possible speeds
//        for (i = 0; i < 128; i++) {
//            //PORTCbits.RC0 = 0;
//            //PWM2_Cycle((float)i/100.0);
//            //PWM3_Cycle(0);
//            drive_motor(i);
//            i++;
//            
//            Delay10KTCYx(500);
//        }
//        for (i = 0; i >128; i--) {
//            //PORTCbits.RC0 = 1;
//            //PWM2_Cycle(0);
//            //PWM3_Cycle((float)i/100.0);
//            drive_motor(-i);
//            Delay10KTCYx(500);
//        }
        
        // use this to find the address (0x68 for MPU 6050 with A0 grounded)
//        channel = surf_channels(); 

        Axr = ((int) (I2C_read(ACCEL_X_H)) << 8 | (int) (I2C_read(ACCEL_X_L))); // / (float)16384;
        Azr = ((int) (I2C_read(ACCEL_Z_H)) << 8 | (int) (I2C_read(ACCEL_Z_L))); // / (float)(131) * 9.81;
        Gyt = (int) (I2C_read(GYRO_Y_H)) << 8 | (int) (I2C_read(GYRO_Y_L));
                
        Axt = Axr / (float)16384 * 9.81;
        Azt = Azr / (float)16384 * 9.81;
        Gyt = Gyr / (float)(131); // for gyroscope
       
        angle = atan2(Axt, Azt) * 180 / 3.14159265358;
        
        speed = angle /ABS(angle) * (ABS(angle) + 55);// + 1.5*Gyt;
        drive_motor(speed / 100);
        //UART_WriteFloat(angle);

        
        // alternate method
//        cycle = THRESH + ABS(speed) * ((127 - THRESH) >> 7); // duty cycle alternate method        
//        drive_motor((float)cycle /100);
        
        // wait to clear screen
//        Delay10KTCYx(500);

//        UART_WriteFloat(Axt);
//        UART_WriteMsg(space);
//        
    }
}


