//************************************************************************************//
//  Copyright 2017 Cláudio Câmara Junior                                              //
//                                                                                    //
//  Licensed under the Apache License, Version 2.0 (the "License");                   //
//  you may not use this file except in compliance with the License.                  //
//  You may obtain a copy of the License at                                           //
//                                                                                    //
//      http://www.apache.org/licenses/LICENSE-2.0                                    //
//                                                                                    //
//  Unless required by applicable law or agreed to in writing, software               //
//  distributed under the License is distributed on an "AS IS" BASIS,                 //
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.          //
//  See the License for the specific language governing permissions and               //
//  limitations under the License.                                                    //
//************************************************************************************//

// DSPIC33FJ128GP802 Configuration Bit Settings
// 'C' source line config statements

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = FRCPLL              // Oscillator Mode (Internal Fast RC (FRC)) w/PLL
#pragma config IESO = OFF               // Internal External Switch Over Mode (Start-up device with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function (OSC2 pin has digital I/O function)
#pragma config IOL1WAY = OFF            // Peripheral Pin Select Configuration (Allow Multiple Re-configurations)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR1             // POR Timer Value (Disabled)
#pragma config ALTI2C = ON              // Alternate I2C  pins (I2C mapped to ASDA1/ASCL1 pins)

// FICD
#pragma config ICS = PGD2               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/* Includes dsPIC libraries */
#define FCY 40000000UL

#define ANALYSIS_NUM_INPUTS 4    /* Number of analog inputs */

// constantes da FFT
#define PONTOS_FFT    256         //numero de pontos da FFT
#define LOG2_ESTAGIOS_BUTTERFLY 7    //numero de estagios "Butterfly" da FFT
#define TAXA_AMOSTRAGEM        15360    //taxa de amostragem do sinal de entrada 


#include <p33Fxxxx.h>
#include <xc.h>
#include <uart.h>
#include <libpic30.h>
#include <pps.h>
#include <dma.h>
#include <adc.h>
#include <dac.h>
#include <timer.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <spi.h>
#include "dsp.h"
//#include <dsp.h>


char print[10];
fractional voltageIN[256], currentIN[256], signalIN[256];
unsigned int *pt;
double voltageRMS = 0.0, currentRMS = 0.0, FP=0.0;//, voltcalc = 0.0, ampcalc = 0.0;
unsigned int trms[20], cont=0, x=0, y=0, j=0;
char sendingPacket[4]; //, packet[4]
float  send[18];//pi=3.1416,
int value=0;

char spiPacketCount=0, COMMAND;
char isSending=0;

// ------------- Declara constantes gravadas na flash -------------------------

// declara vetor dos coeficientes Twiddle da FFT na memoria de programa
// Define 256 constantes "twiddle" da FFT: WN (kn) = exp [-(j*2*pi*k*n)/N]
const fractcomplex twiddleFactors[] __attribute__ ((space(prog), aligned (PONTOS_FFT*2)))=
        {0x7FFF, 0x0000, 0x7FF6, 0xFCDC, 0x7FD9, 0xF9B8, 0x7FA7, 0xF695,
        0x7F62, 0xF374, 0x7F0A, 0xF055, 0x7E9D, 0xED38, 0x7E1E, 0xEA1E,
        0x7D8A, 0xE707, 0x7CE4, 0xE3F4, 0x7C2A, 0xE0E6, 0x7B5D, 0xDDDC,
        0x7A7D, 0xDAD8, 0x798A, 0xD7D9, 0x7884, 0xD4E1, 0x776C, 0xD1EF,
        0x7642, 0xCF04, 0x7505, 0xCC21, 0x73B6, 0xC946, 0x7255, 0xC673,
        0x70E3, 0xC3A9, 0x6F5F, 0xC0E9, 0x6DCA, 0xBE32, 0x6C24, 0xBB85,
        0x6A6E, 0xB8E3, 0x68A7, 0xB64C, 0x66CF, 0xB3C0, 0x64E8, 0xB140,
        0x62F2, 0xAECC, 0x60EC, 0xAC65, 0x5ED7, 0xAA0A, 0x5CB4, 0xA7BD,
        0x5A82, 0xA57E, 0x5843, 0xA34C, 0x55F6, 0xA129, 0x539B, 0x9F14,
        0x5134, 0x9D0E, 0x4EC0, 0x9B18, 0x4C40, 0x9931, 0x49B4, 0x9759,
        0x471D, 0x9592, 0x447B, 0x93DC, 0x41CE, 0x9236, 0x3F17, 0x90A1,
        0x3C57, 0x8F1D, 0x398D, 0x8DAB, 0x36BA, 0x8C4A, 0x33DF, 0x8AFB,
        0x30FC, 0x89BE, 0x2E11, 0x8894, 0x2B1F, 0x877C, 0x2827, 0x8676,
        0x2528, 0x8583, 0x2224, 0x84A3, 0x1F1A, 0x83D6, 0x1C0B, 0x831C,
        0x18F9, 0x8276, 0x15E2, 0x81E3, 0x12C8, 0x8163, 0x0FAB, 0x80F7,
        0x0C8C, 0x809E, 0x096B, 0x8059, 0x0648, 0x8028, 0x0324, 0x800A,
        0x0000, 0x8000, 0xFCDC, 0x800A, 0xF9B8, 0x8028, 0xF695, 0x8059,
        0xF374, 0x809E, 0xF055, 0x80F7, 0xED38, 0x8163, 0xEA1E, 0x81E3,
        0xE707, 0x8276, 0xE3F5, 0x831C, 0xE0E6, 0x83D6, 0xDDDC, 0x84A3,
        0xDAD8, 0x8583, 0xD7D9, 0x8676, 0xD4E1, 0x877C, 0xD1EF, 0x8894,
        0xCF04, 0x89BE, 0xCC21, 0x8AFB, 0xC946, 0x8C4A, 0xC673, 0x8DAB,
        0xC3A9, 0x8F1D, 0xC0E9, 0x90A1, 0xBE32, 0x9236, 0xBB85, 0x93DC,
        0xB8E3, 0x9593, 0xB64C, 0x975A, 0xB3C0, 0x9931, 0xB140, 0x9B18,
        0xAECC, 0x9D0E, 0xAC65, 0x9F14, 0xAA0A, 0xA129, 0xA7BD, 0xA34C,
        0xA57E, 0xA57E, 0xA34C, 0xA7BD, 0xA129, 0xAA0A, 0x9F14, 0xAC65,
        0x9D0E, 0xAECC, 0x9B18, 0xB140, 0x9931, 0xB3C0, 0x975A, 0xB64C,
        0x9593, 0xB8E3, 0x93DC, 0xBB85, 0x9236, 0xBE32, 0x90A1, 0xC0E9,
        0x8F1D, 0xC3A9, 0x8DAB, 0xC673, 0x8C4A, 0xC946, 0x8AFB, 0xCC21,
        0x89BF, 0xCF04, 0x8894, 0xD1EF, 0x877C, 0xD4E1, 0x8676, 0xD7D9,
        0x8583, 0xDAD8, 0x84A3, 0xDDDC, 0x83D6, 0xE0E6, 0x831C, 0xE3F5,
        0x8276, 0xE707, 0x81E3, 0xEA1E, 0x8163, 0xED38, 0x80F7, 0xF055,
        0x809E, 0xF374, 0x8059, 0xF695, 0x8028, 0xF9B8, 0x800A, 0xFCDC};


// Entrada do sinal para a FFT declarada na memoria Y
fractcomplex sinalComplexo[PONTOS_FFT]__attribute__ ((section (" data, ymemory"),aligned (PONTOS_FFT * 2 *2)));

fractional SquareMagnitude [PONTOS_FFT/2];

unsigned int analysis_adcBuffer[ANALYSIS_NUM_INPUTS] __attribute__((space(dma),aligned(4)));

fractional *p_real = &sinalComplexo[0].real ;
fractcomplex *p_complexo = &sinalComplexo[0] ;
fractional *p_fract = &sinalComplexo[0].real ;

// calcula a magnitude quadrada do vetor complexo
//extern void SquareMagnitudeCplx(int, fractcomplex*, fractional*); 





    void SetClock();
    void SetPins();
    void SetUart();
    void SetSPI();
    void SetTimer();
    void SetADC();
    //void CalcRMS();

    
    // INTERUPTION ROUTINES 

void __attribute__((__interrupt__,no_auto_psv)) _ADC1Interrupt(void) {
    
    //PORTBbits.RB11 = 1;
   /* 
    voltageIN[cont] = ADCBUF0; //RB3  respect order RB0, RB1..etc, if not used skip it but ADCBUFs are ordered
    IFS0bits.AD1IF = 0;  //After conversion ADIF is set to 1 and must be cleared
    cont++;
    
    if (cont>2){
        _AD1IE = 0;
        cont = 0;
    }*/
    //PORTBbits.RB11 = 0;

}

void __attribute__((__interrupt__,no_auto_psv)) _DMA0Interrupt() {
    // CleaR interruption flag 
    IFS0bits.DMA0IF = 0;

    signalIN[cont] = analysis_adcBuffer[1];
    voltageIN[cont] = analysis_adcBuffer[2];
    currentIN[cont] = analysis_adcBuffer[3];
    cont++;
   
    if (cont>255){
        _DMA0IE = 0;
        cont = 0;
    }
      PORTBbits.RB10 = 1;
}
/*
void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt (void) {
    
    IFS0bits.T3IF = 0;   
}*/

void __attribute__((__interrupt__,no_auto_psv)) _SPI1Interrupt(void) {

    IFS0bits.SPI1IF = 0;
    
    if(isSending){   //Send Float (Ok))
        while (SPI1STATbits.SPIRBF != 1) {
            }; //Receive is not complete
        COMMAND = ReadSPI1();
        while (SPI1STATbits.SPITBF != 0) {
            }; //1 = Transmission haven't started yet, SPIxTXB is full
        WriteSPI1(sendingPacket[x]);
        x++;
        if(x==5){
            isSending=0;
            x=0;
            y++;
            if(y==19) {
                y=0;     
            }
            }
    }else{
        isSending=1;
        memcpy((void*) sendingPacket, (void*) &send[y], 4);
        while (SPI1STATbits.SPIRBF != 1) {
            }; //Receive is not complete
            COMMAND = ReadSPI1();
    }
}
    
/*if(isSending){   //Send Float (Ok))
        while (SPI1STATbits.SPIRBF != 1) {
            }; //Receive is not complete
        COMMAND = ReadSPI1();
        while (SPI1STATbits.SPITBF != 0) {
            }; //1 = Transmission haven't started yet, SPIxTXB is full
        WriteSPI1(sendingPacket[x]);
        x++;
        if(x==5){
            isSending=0;
            x=0;
            }
    }else{
        isSending=1;
        memcpy((void*) sendingPacket, (void*) &(pi), 4);
        while (SPI1STATbits.SPIRBF != 1) {
            }; //Receive is not complete
            COMMAND = ReadSPI1();
    }*/
        
    
    //CONFIGURATION ROUTINES

void SetClock(){
  
    // Configure PLL prescaler(N1), PLL postscaler(N2), PLL divisor(M0)
    // FOSC = 7,37*(M/(N1*N2)) = 40,00857MHz
    
    PLLFBD=52; // M = 152 ( 2-513) 150                 //43
    CLKDIVbits.PLLPOST=0; // N2 = 4 ( 2, 4, 8 ) 1       //2
    CLKDIVbits.PLLPRE=0; // N1 = 7 ( 2-33 ) 5           //2

    while(OSCCONbits.LOCK != 1) {};  //Wait clock to stabilize
}

void SetPins(){
    
    /* Config LEDs */
    
    TRISBbits.TRISB10 = 0;
    TRISBbits.TRISB11 = 0;
    //TRISBbits.TRISB12 = 0;
    PORTBbits.RB10 = 0;
    PORTBbits.RB11 = 0;
    //PORTBbits.RB12 = 0;
    
    /* Config switchs */
    
   // TRISBbits.TRISB13 = 1;
   // TRISBbits.TRISB14 = 1;
   // TRISBbits.TRISB15 = 1;
    
}

void SetSPI(){
    unsigned int SPICON1;
    unsigned int SPICON2;
    unsigned int SPICON3;
    
    TRISBbits.TRISB12 = 1; //RP12 - INPUT (SLAVE SELECT)
    TRISBbits.TRISB13 = 1; //RP13 - INPUT (MOSI)
    TRISBbits.TRISB14 = 0; //RP14 - OUTPUT (MISO)
    TRISBbits.TRISB15 = 1; //RP15 - INPUT (CLOCK)
    
    PPSUnLock;
    
    PPSOutput(OUT_FN_PPS_SDO1, OUT_PIN_PPS_RP14); //RP14 - OUTPUT (MISO)
    PPSInput(IN_FN_PPS_SS1, IN_PIN_PPS_RP12);     //RP12 - INPUT (SLAVE SELECT)
    PPSInput(IN_FN_PPS_SDI1, IN_PIN_PPS_RP13);    //RP13 - INPUT (MOSI)
    PPSInput(IN_FN_PPS_SCK1, IN_PIN_PPS_RP15);    //RP15 - INPUT (CLOCK)

    PPSLock;
    
     CloseSPI1();
     
         
     SPICON1 =     DISABLE_SCK_PIN &        //Internal SPI clock is diabled, pin functions as I/O
                   ENABLE_SDO_PIN &         //SDO pin is  used by module
                   SPI_MODE16_OFF &         //Communication is byte wide 
                   SPI_SMP_OFF &            //Input data sampled at middle of data output time
                   SPI_CKE_ON &             //Transmit happens from active clock state to idle clock state
                   SLAVE_ENABLE_ON &        //Slave Select enable    
                   CLK_POL_ACTIVE_HIGH &    //Idle state for clock is low, active is high
                   MASTER_ENABLE_OFF ;      //Slave Mode
                   //SEC_PRESCAL_7_1 &      //FOR MASTER Prescale 1
                   //PRI_PRESCAL_64_1;      //FOR MASTER Prescale 2
     SPICON2 =      FRAME_ENABLE_OFF ;       //Frame SPI support Disable            
     SPICON3 =       SPI_ENABLE &             //Enable module
                     SPI_RX_OVFLOW_CLR;       //Clear receive overflow bit
     
 OpenSPI1(SPICON1,SPICON2,SPICON3);
 
 ConfigIntSPI1(SPI_INT_EN & SPI_INT_PRI_1);
 
 EnableIntSPI1;
 
 IFS0bits.SPI1IF = 0;
 
}

void SetUart(){
    
    /* Config UART */
    
    TRISBbits.TRISB8 = 1;      //Pin RP8 as input pin for UART RX
    TRISBbits.TRISB9 = 0;      //Pin RP9 as output pin for UART TX
    
    //Config Remappable Pins
    PPSUnLock;
    
    PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP9); //UART TX
    PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP8);    //UART RX

    PPSLock;
    
    //Config Uart
    CloseUART1();

    ConfigIntUART1(UART_RX_INT_EN &     //Enable RX Interrupt
        UART_RX_INT_PR2 &           //Set priority level
        UART_TX_INT_DIS &           //Disable TX Interrupt
        UART_TX_INT_PR0);           //Priority as none

    IFS0bits.U1RXIF = 0;    //reset interrupt flag  

    OpenUART1(UART_EN &                 //Enable UART
        UART_IDLE_CON &             //Continue working when Idle
        UART_IrDA_DISABLE &         //Not using IrDA
        UART_MODE_SIMPLEX &         //Not using Flow control
        UART_UEN_00 &               //Not using CTS/RTS pins by not setting it up
        UART_DIS_WAKE &             //Disable wakeup option
        UART_DIS_LOOPBACK &         //Not using in loopback mode
        UART_DIS_ABAUD &            //Disable ABAUD
        UART_NO_PAR_8BIT &          //8bit data with none for parity
        UART_BRGH_SIXTEEN &         //Clock pulse per bit
        UART_UXRX_IDLE_ONE&         // UART Idle state is 1
        UART_1STOPBIT,              // 1 stop bit
         UART_INT_TX_BUF_EMPTY  &   //Enable UART transmit
         UART_TX_ENABLE & 
         UART_IrDA_POL_INV_ZERO &    //sets the pin's idle state
         UART_SYNC_BREAK_DISABLED &
         UART_INT_RX_CHAR &          //Interrupt at every character received
         UART_ADR_DETECT_DIS &       //Disabled since not using addressed mode
         UART_RX_OVERRUN_CLEAR,      //UART Overrun Bit Clear
          160);                       //BAUDRATE 10 = 115200/21 = 57600/ 64 = 19200/ 129 = 9600
}

void SetTimer(){
    
    /* Configures the timer to Timer Mode
       (Refer to "timer.h" and "Section 11. Timers" for further information) */
    WriteTimer3(0);

    OpenTimer3(T3_ON & T3_IDLE_CON & T3_GATE_OFF &
               T3_SOURCE_INT & T3_PS_1_1, 810);     //7812,5 = 100ms(PS=256) ,64 = 1200hz(PS=256), 40 = 7680hz(PS=64), 1282 = 15600Hz (PS=1)
    
    SetPriorityIntT3(5);

    /* Disables timer interruption */
    ConfigIntTimer3(T3_INT_OFF);
    
}
void SetDMA(){
        
    CloseDMA0();
    OpenDMA0(DMA0_MODULE_ON &
            DMA0_SIZE_WORD & 
            PERIPHERAL_TO_DMA0 &
             DMA0_NORMAL & 
            DMA0_INTERRUPT_BLOCK & 
            DMA0_REGISTER_POST_INCREMENT &
             DMA0_CONTINUOUS, 
            DMA0_AUTOMATIC & 
            13, __builtin_dmaoffset(analysis_adcBuffer),
             0, (volatile unsigned int) &
            ADC1BUF0, ANALYSIS_NUM_INPUTS-1);

    // Enables DMA interruption 
    ConfigIntDMA0(DMA0_INT_ENABLE & DMA0_INT_PRI_6);
}

void SetADC(){
    unsigned int config1;
    unsigned int config2;
    unsigned int config3;
    unsigned int config4;
    unsigned int configport_l;
    unsigned int configport_h;
    unsigned int configscan_h;
    unsigned int configscan_l;
    
    config1 = ADC_MODULE_ON &           // A/D Converter on 
            ADC_IDLE_STOP &             // A/D Stop in Idle mode 
            ADC_ADDMABM_ORDER &         // DMA buffers are written in the order of conversion 
            ADC_AD12B_10BIT &           // 10 bit, 4-channel ADC operation 
            ADC_FORMAT_SIGN_FRACT &     // A/D data format signed fractional     /ADC_FORMAT_INTG = A/D data format integer 
            ADC_CLK_TMR &               // GP Timer compare ends sampling and starts conversion (Changed to AUTO)
            ADC_AUTO_SAMPLING_ON &      // Sampling begins immediately after last conversion 
            ADC_MULTIPLE ;              // Samples multiple channels individually in sequence 
 
    config2 = ADC_VREF_AVDD_AVSS &      // A/D Voltage reference configuration Vref+ is AVdd and Vref- is AVss
            ADC_SCAN_OFF &              // A/D Doesn't Scan Input Selections for CH0 during SAMPLE A
            ADC_SELECT_CHAN_0123 &      // Converts CH0, CH1, CH2 and CH3
            ADC_DMA_ADD_INC_1 ;         // DMA address increment after conversion of each sample
 
    config3 = ADC_CONV_CLK_SYSTEM &     // A/D Conversion Clock Source Clock derived from system clock
            ADC_SAMPLE_TIME_3 &         // A/D Auto Sample Time 3 Tad
            ADC_CONV_CLK_127Tcy;         // A/D conversion clock select bit ADCS<7:0> //63
 
    config4 = ADC_DMA_BUF_LOC_32;       // Allocates words of buffer to each analog input
 
    configport_h = ENABLE_ALL_DIG_16_31;// Enable AN16-AN31 in Digital mode 
 
    configport_l = ENABLE_ALL_DIG_0_15;// Enable AN0-AN15 in Digital mode 
    //configport_l = ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN2_ANA & ENABLE_AN3_ANA;      // Enable AN0 in analog mode
 
    configscan_h = SCAN_NONE_16_31;
            
    configscan_l = SCAN_NONE_0_15;
    
    CloseADC1();
    
    ConfigIntADC1(ADC_INT_DISABLE & ADC_INT_PRI_1);
    
    OpenADC1(config1,config2,config3,config4,configport_l,configport_h, configscan_h,configscan_l);
    
    //AD1CSSL = 0x000F; // Scan AIN0, AIN1, AIN2, AIN3 inputs
    AD1PCFGL = 0xFF80;// Analogs AN0, AN1, AN2, AN3, AN4, AN5, AN6 inputs
    AD1CON2bits.SMPI  = 3; // Increments DMA address according to the number of inputs
    
    // Configures ADC input channels
    AD1CSSL = 0;
    AD1CHS0bits.CH0SA = 3;
    AD1CHS0bits.CH0NA = 0;
    AD1CHS123bits.CH123SA = 0;
    AD1CHS123bits.CH123NA = 0;
    
    // Starts ADC module 
    AD1CON1bits.ADON = 1;
    
    
      
}

    //NORMAL ROUTINES

void CalcRMS(){
    int i;
    double voltage = 0.0, current = 0.0, voltcalc = 0.0, ampcalc = 0.0, VCCcorrect;
    
    for(i=0;i<256;i++){
        
        //if(signalIN[i]== -32767){
        //    signalIN[i]= 16881;
        //}
        VCCcorrect = (float) signalIN[i];
        VCCcorrect = 65535/(VCCcorrect+32767);  //32767
        VCCcorrect *= 2.5;
        
        voltage =(float) voltageIN[i];
        voltage = voltage/32767;
        voltage *= VCCcorrect;
        voltage *= 82.0492;
        voltage = voltage*voltage;
        voltcalc = voltcalc + voltage;
        
               
        current =(float) currentIN[i];
        current = current/32767;
        current *= VCCcorrect;
        current *= 4.9437;  //*1.087613
        sprintf(print,"%.2f,",current);
        putsUART1((unsigned int*)print);
        current = current*current;
        ampcalc = ampcalc + current;
        
    }
    voltcalc = voltcalc/256.0;
    voltageRMS = sqrt(voltcalc);
    sprintf(print,"%.2fV,",voltageRMS);
    putsUART1((unsigned int*)print);
    send[0] = voltageRMS;
    
    ampcalc = ampcalc/256.0;
    currentRMS = sqrt(ampcalc);
    sprintf(print,"%.2fA,",currentRMS);
    putsUART1((unsigned int*)print);
    send[1] = currentRMS;
}

void calcFP(){
    int i;
    fractional maxV, maxI, locV=0, locI=0;
    
    maxV = voltageIN[0];
    maxI = currentIN[0];
    
    for(i=0; i<256; i++){
    
        if(voltageIN[i]>maxV){
            maxV = voltageIN[i];
            locV = i;
        }
        if(currentIN[i]>maxI){
            maxI = currentIN[i];
            locI = i;
        }
    }
    FP =(float) locI-locV;
    FP = FP*0.024544;
    FP = cos(FP);
    
    send[2] = FP;
    sprintf(print,"%.3f°,",FP);
    putsUART1((unsigned int*)print);
}

void FFT(fractional currentIN[256]){
    int i;
    
      p_real = &sinalComplexo[0].real ;              //inicializa ponteiro
      p_complexo = &sinalComplexo[0];                //inicializa ponteiro
    
      /* Move buffer do sinal de entrada (real)para vetor da FFT    */
      for ( j = 0; j < PONTOS_FFT; j++ )             
      {
          *p_real = currentIN [j] ;        
          *p_real++;                                 //incrementa o ponteiro   
          
      }
      
      /* Escalona o vetor para o range [-0.5, +0.5]                */
      p_real = &sinalComplexo[0].real ;             //inicializa ponteiro
      for ( j = 0; j < PONTOS_FFT; j++ )         
      {
          *p_real = *p_real >>1 ;                     //desloca 1 bit para a direita
          *p_real++; 
          
      }
      
      /* Converte vetor real para vetor complexo                    */
      p_real = &sinalComplexo[(PONTOS_FFT/2)-1].real ;    //inicializa o ponteiro para parte real do vetor  
      p_complexo = &sinalComplexo[PONTOS_FFT-1] ; 
      
      //PORTBbits.RB11 = ~PORTBbits.RB11;
      
      for ( j = PONTOS_FFT; j > 0; j-- )         
      {
          (*p_complexo).real = (*p_real--);    
          (*p_complexo--).imag = 0x0000;            //NÃ£o possui parte imaginaria, escreve valor zero
                     
      }
      /* Calcula a FFT - Transformada Rapida de Fourier             */
        FFTComplexIP (LOG2_ESTAGIOS_BUTTERFLY, &sinalComplexo[0], (fractcomplex*) __builtin_psvoffset(&twiddleFactors[0]), (int) __builtin_psvpage(&twiddleFactors[0]));
    

      /* Organiza dados da saída da FFT - Bit-reverso               */
        BitReverseComplex (LOG2_ESTAGIOS_BUTTERFLY, &sinalComplexo[0]);
        
        
      /* Calcula a magnitude quadrada do vetor de saída complexo do FFT, a saída é um vetor real */
        SquareMagnitudeCplx(PONTOS_FFT, &sinalComplexo[0], &sinalComplexo[0].real);
        
    
       /* Move buffer do vetor complexo para vetor comum            */
        p_fract = &sinalComplexo[0].real ;                //inicia ponteiro
        for ( j = 0; j < PONTOS_FFT/2; j++ )             
        {
            
            SquareMagnitude [j] = *p_fract;
            //sprintf(test,"%.3f,",(float)*p_fract);
            //putsUART1((unsigned int*)test);
            *p_fract++;                                   //incrementa o ponteiro
        }
        
        for(i=1;i<15;i++){
            
            send[i+2] = SquareMagnitude[i];
            sprintf(print,"%d,",SquareMagnitude[i]);
            putsUART1((unsigned int*)print);
            
        }
    
}

    //MAIN FUNCTION

int main() {
    
    
    
    SetClock();
    SetPins();
    SetUart();
    SetTimer();
    SetDMA();
    SetADC();
    SetSPI();
    __delay_ms(1000);
        
         
    
    
    while(1){
        
      PORTBbits.RB11 = 1;
    
      __delay32(40000000);
      
      PORTBbits.RB10 = 0;
      
      CalcRMS();
      
      calcFP();
      
      FFT(currentIN);
       
      //PORTBbits.RB10 = 1;
      /* 
      p_real = &sinalComplexo[0].real ;              //inicializa ponteiro
      p_complexo = &sinalComplexo[0];                //inicializa ponteiro
    
      // Move buffer do sinal de entrada (real)para vetor da FFT    
      for ( j = 0; j < PONTOS_FFT; j++ )             
      {
          *p_real = voltageIN [j] ;        
          *p_real++;                                 //incrementa o ponteiro   
          
      }
      
      // Escalona o vetor para o range [-0.5, +0.5]                
      p_real = &sinalComplexo[0].real ;             //inicializa ponteiro
      for ( j = 0; j < PONTOS_FFT; j++ )         
      {
          *p_real = *p_real >>1 ;                     //desloca 1 bit para a direita
          *p_real++; 
          
      }
      
      // Converte vetor real para vetor complexo                    
      p_real = &sinalComplexo[(PONTOS_FFT/2)-1].real ;    //inicializa o ponteiro para parte real do vetor  
      p_complexo = &sinalComplexo[PONTOS_FFT-1] ; 
      
      //PORTBbits.RB11 = ~PORTBbits.RB11;
      
      for ( j = PONTOS_FFT; j > 0; j-- )         
      {
          (*p_complexo).real = (*p_real--);    
          (*p_complexo--).imag = 0x0000;            //NÃ£o possui parte imaginaria, escreve valor zero
                     
      }
        // Calcula a FFT - Transformada Rapida de Fourier             
        FFTComplexIP (LOG2_ESTAGIOS_BUTTERFLY, &sinalComplexo[0], (fractcomplex*) __builtin_psvoffset(&twiddleFactors[0]), (int) __builtin_psvpage(&twiddleFactors[0]));
    

        // Organiza dados da saÃ­da da FFT - Bit-reverso                
        BitReverseComplex (LOG2_ESTAGIOS_BUTTERFLY, &sinalComplexo[0]);
        
        
        // Compute the square magnitude of the complex FFT output array so we have a Real output vetor 
        SquareMagnitudeCplx(PONTOS_FFT, &sinalComplexo[0], &sinalComplexo[0].real);
        PORTBbits.RB10 = ~PORTBbits.RB10;
    
        // Move buffer do vetor complexo para vetor comum             
        p_fract = &sinalComplexo[0].real ;                //inicia ponteiro
        for ( j = 0; j < PONTOS_FFT/2; j++ )             
        {
            
            SquareMagnitude [j] = *p_fract;
            //sprintf(test,"%.3f,",(float)*p_fract);
            //putsUART1((unsigned int*)test);
            *p_fract++;                                   //incrementa o ponteiro
        }
       */
       //PORTBbits.RB10 = 0;
        
       
          
       /*for(j=0;j<20;j++){
       
           a = trms[j];
           a = a/310.0f;
           sprintf(test,"%.3f",a);
           putsUART1((unsigned int*)test);
           WriteUART1(32);
       } */
       
       //sprintf(test,"%.3f",voltageRMS);
       //WriteUART1("V");
       //putsUART1((unsigned int*)test);
        
       
       __delay_ms(2000);
       //__delay32(20000000);
       _DMA0IE = 1;
              
    }
       
    return(0);
}