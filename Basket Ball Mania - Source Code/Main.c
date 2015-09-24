/*
 * File:   Main.c
 * Author: Group05
 *
 * Created on November 7, 2014, 4:13 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include<PIC24F_periph_features.h>
#include<string.h>
#include "ELB_PWM.h"
#include "p24FJ256GB206.h"
#include "CONFIGbits.h"  //to check
#include "Define.h"
#include "AMGPsensor.h"  //to check
#include "ELB_I2C.h"
#include "ELB_UART.h"
#include "ELBv21_HardwareConfig.h"
#include "ELB_OSC.h"
#include "PPS.h"
#include "math.h"
#include "ELB_Timer.h"
#include "ELB_LCD.h"
#include <ports.h>

/*
 *
 */

extern ts_ELB_Hardware Hardware;                        //Hardware Global Structure
extern ts_AMGPsensor AMGP;                              //Sensor Global Structure
extern U8 V_T2IntFlag_U8;                              //Set in TIMER ISR in file ELB_ISR.c

U8 A_Lcd_U8[50];
/*** GLOBAL VARIABLES ***/
F32 V_Pitch_F32 = 0;

int main(int argc, char** argv) {

/*** LOCAL VARIABLES ***/
    int Readbits,laser,no_laser,score,iterations;
    S16 v_AccX_U8 = 2;
    S16 v_AccY_U8 = 2;
    S16 v_AccZ_U8 = 2;
    S16 v_AccX2_U8 = 2;
    S16 v_AccY2_U8 = 2;
    S16 v_AccZ2_U8 = 2;
    S16 rotation=1;
    /*** CONFIGURE OSCILLATOR ***/
    SET_FreqOsc(FRCDIV_8MHZ);                         //Set frequency of 1 MHZ
    S16 velx,vely,velz;
    S16 limit=0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           ;
    S16 count;
    S16 a=20;
    S16 fixed,variable,speed,speed1;
    S16 iter=0;
    /*** CONFIGURE HARDWARE ****/
    speed=0;
    SET_FreqOsc( FRCDIV_1MHZ );
    Hardware_INIT();                                    //Initialise Hardware functions
    Hardware.ConfigPins_Default();                      //Configure Default Hardware for ELB
    Hardware.ConfigPins_Motor(USE1|USE2|USE3);
    // Enable_PB123Int();
    Hardware.ConfigPins_LED(USE1|USE2|USE3);

    LCD_INIT();
    TRISBbits.TRISB1 = 0;
   // AD1PCFG = 0xFFFE;
    AD1CON1 = 0x0000;
    AD1CHS = 0x0001;
    AD1CSSL = 0;
    AD1CON2 = 0;
    AD1CON3 = 0x0002;

    AD1CON1bits.ADON = 1;
    //TIMER2_INIT(V_Pitch_F32,v_AccX_U8);
    // INITIALIZE PERIPHERAL //
                        //Proivde timer period in millisecond
    UART1_INIT(M_9600Hz,M_BRGH_High,TX_INT_PRI0);// Initialize UART1
    I2C1_INIT(400000, MI2C_INT_PRI0);

    //I2C1_INIT( M_MASTER , M_SLEW_OFF , M_I2CBRG_FastMode) ;                 //Inilialize I2C1
    //Interrupts_INIT(M_9600Hz,MI2C_INT_PRI0);
    //_T1Interrupt();
    //Interrupts_INIT( M_TMR2IE ,M_NoPriority);   // ISR Enable

    //sprintf(A_Lcd_U8,"Values: ");
  // LCD_WriteString(1,1,A_Lcd_U8);
 
   AMGP_INIT(READ_HEADER1,SEND_UART1XBEE);              //Select Header to mount sensor card and...
    
   DELAY_mSec(4000);
     //...UART port to send data out (To be implemented, Modify this in AMGPSensor.h)
   LED2=C_ON;
   AMGP.Config(Acc);
    LED3=C_ON;
    //sprintf(A_Lcd_U8," Hi Gibran... ");
     //LCD_WriteString(1,2,A_Lcd_U8);
    AD1CON1bits.SAMP = 1;
        DELAY_mSec(0.1);
        DELAY_mSec(1000);
        AD1CON1bits.SAMP = 0;
        
       while(!AD1CON1bits.DONE);
       Readbits=0;
       //Readbits = ADC1BUF0;
        //LED2=C_ON;
        laser=Readbits;
        no_laser=laser+40;
        no_laser=10000;
        sprintf(A_Lcd_U8," %d ",Readbits);
        LCD_WriteString(2,4,A_Lcd_U8);
        score=0;
   // LED2=C_ON;
     //LED2=C_ON;
    //Select the sensors to work with eg.(Acc|Mag|Pres)
   // sprintf(A_Lcd_U8,"Val1 ");
  //LCD_WriteString(2,1,A_Lcd_U8);
    Hardware.ConfigPins_PWM(USE2);
    PWM2_INIT(PWMsrc_FOSC,2000);
    PWM2 = 0;
    PWM2_SET_PulseWidth(50);
  // sprintf(A_Lcd_U8,"Val ");
  // LCD_WriteString(2,1,A_Lcd_U8);
   
    while(1)
    {
        // READ SENSOR DATA AFTER TIMER PERIOD ELAPSED /
        //if(V_T2IntFlag_U8)                             // Timer period defined in the Timer init function...
        //    V_T2IntFlag_U8 = 0;// ...the timer interrupt flag is set in ELB_ISR.c
         AD1CON1bits.SAMP = 1;
        DELAY_mSec(0.1);
        AD1CON1bits.SAMP = 0;
        while(!AD1CON1bits.DONE);
       
        Readbits = ADC1BUF0;
        if(Readbits>no_laser)
        {
            score=score+1;
          //  sprintf(A_Lcd_U8," %d %d ",Readbits,score);
           // LCD_WriteString(2,4,A_Lcd_U8);
            DELAY_mSec(2000);
        }
        else
        {
        sprintf(A_Lcd_U8," %d %d ",Readbits,score);
        LCD_WriteString(2,4,A_Lcd_U8);
        }
            
            //
          
             //MotA1=C_ON;
            //MotA2=C_OFF;
            
           AMGP.Read(Acc);      // Read selected Sensors Data  Mag|Gyro|Pres|TempPres
            AMGP.Send(Acc);
            
            // Send sensors data Packet through selected UART   |Mag|Gyro|Pres|TempPres
          //  AMGP.Read(Acc);
           // sprintf(A_Lcd_U8,"Val1");
   //LCD_WriteString(2,5,A_Lcd_U8);
            v_AccX_U8 = AMGP.Data.AccX;
            v_AccY_U8 = AMGP.Data.AccY;
            v_AccZ_U8 = AMGP.Data.AccZ;
            iterations=0;
            LCD_Clear();
            speed1=speed;
            while(iterations<=(speed*speed)/10)
            {
              LED3=C_OFF;
              if(speed1!=speed)
                  break;
                //sprintf(A_Lcd_U8," %d %d ",iterations,(speed*speed)/10);
               // LCD_WriteString(2,4,A_Lcd_U8);
                //DELAY_mSec(500);
                if(a<speed)
                {
             if(iterations%7<(speed*1.0/8))
             {
                
                  MotA1=C_OFF;
                  MotA2=C_ON;
             }
             else
             {
            
                  MotA1=C_OFF;
                MotA2=C_OFF;
             }
             }
            DELAY_mSec(0.1);

            iterations=iterations+1;
            AD1CON1bits.SAMP = 1;
            DELAY_mSec(0.1);
            AD1CON1bits.SAMP = 0;
            while(!AD1CON1bits.DONE);
           
          //  Readbits = ADC1BUF0;
            if(Readbits>no_laser)
            {
            MotA1=C_ON;
                MotA2=C_ON;
                score=score+1;
           // sprintf(A_Lcd_U8," %d %d ",iterations,(speed));
            //LCD_WriteString(2,4,A_Lcd_U8);
            //sprintf(A_Lcd_U8," %d %d",iterations,(1000+speed*speed)/15);
            sprintf(A_Lcd_U8," %d %d ",Readbits,score);
            LCD_WriteString(2,4,A_Lcd_U8);
            sprintf(A_Lcd_U8,"%d %d %d %d %d %d",speed,a,variable,velx,vely,velz);
            LCD_WriteString(1,1,A_Lcd_U8);
             DELAY_mSec(2000);
             iterations=iterations+2000;
            }
            else
            {
              // sprintf(A_Lcd_U8," %d %d ",iterations,(speed));
            //LCD_WriteString(2,4,A_Lcd_U8);
                //sprintf(A_Lcd_U8," %d %d",iterations,(1000+speed*speed)/15);
               sprintf(A_Lcd_U8," %d %d ",Readbits,score);
                LCD_WriteString(2,4,A_Lcd_U8);
                sprintf(A_Lcd_U8,"%d %d %d %d %d %d",speed,a,variable,velx,vely,velz);
                LCD_WriteString(1,1,A_Lcd_U8);
            }
            }
            MotA1=C_ON;
                MotA2=C_ON;
                DELAY_mSec(2000);
            LCD_Clear();
           //   sprintf(A_Lcd_U8," %d %d ",iterations,(speed*speed)/10);
            //    LCD_WriteString(2,4,A_Lcd_U8);
           //LED3=C_ON;
                 
            AMGP.Read(Acc);      // Read selected Sensors Data  Mag|Gyro|Pres|TempPres
            AMGP.Send(Acc);
           //` LCD_Clear();
            v_AccX2_U8 = AMGP.Data.AccX;
            v_AccY2_U8 = AMGP.Data.AccY;
            v_AccZ2_U8 = AMGP.Data.AccZ;
            velx=abs(v_AccX2_U8-v_AccX_U8);
            vely=abs(v_AccY2_U8-v_AccY_U8);
            velz=abs(v_AccZ2_U8-v_AccZ_U8);
           // sprintf(A_Lcd_U8,"X:%d Y:%d Z:%d", velx,vely,velz);
           // LCD_WriteString(2,3,A_Lcd_U8);
            count=0;
           // a=(a+1)%100;
            fixed=9;
            variable=sqrt(velx*velx+vely*vely+velz*velz)/2;
            if(variable>0)
            {
                a=20;
            }
            speed=fixed+variable;
  
            if(velx>limit)
            {
                count=1;
                iter=0;
                         sprintf(A_Lcd_U8,"%d %d %d %d %d %d",speed,a,variable,velx,vely,velz);
   LCD_WriteString(1,1,A_Lcd_U8);
               
                   // a=(a+1)%100;
                if(a<speed)
                {
            
                    if(rotation==0)
                    {
                        MotA1=C_ON;
                        MotA2=C_OFF;

                    }
                    else
                    {
                        MotA1=C_OFF;
                        MotA2=C_ON;
                    }
                }
                else
                {
                MotA1=C_OFF;
                MotA2=C_OFF;
                }
            }
            else
            {
                
                MotA1=C_OFF;
                MotA2=C_OFF;
            }
            if(count==0)
            {
            if(vely>limit)
            {
                iter=0;
                    //a=(a+1)%100;
                if(a<speed)
                {
              
                    if(rotation==0)
                    {
                        MotA1=C_ON;
                        MotA2=C_OFF;
                    }
                    else
                    {
                        MotA1=C_OFF;
                        MotA2=C_ON;
                    }
                    }
                else
                {
                
                MotA1=C_OFF;
                MotA2=C_OFF;
                }
                
            }
            else
            {
              //  LED3=C_OFF;
                MotA1=C_OFF;
                MotA2=C_OFF;
            }
            }
            
            



    }

    return (EXIT_SUCCESS);
}

