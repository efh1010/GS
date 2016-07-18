



/*
 * File:   main.c
 * Author: Edoardo Contini
 *
 * Created on 13 May 2016, 3.48
 */

// #######################################################################################
// #######################################################################################

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PRJ NOTES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    -> rx_data and tx_data must be unified in the same array

    -> while the pointers of tx and rx have to be different (read below)

    -> on the unified array data must be poped from the bottom and pushed from the top

    -> gps and serial comm will be driven on 2 different serial ports

    -> time needed for trm1 overflow must be estimated

    -> an estimate of overflow TMR0 time is needed 

    -> i2c comm (if hw) must be managed in the low priority routine

    -> a string assembler fuction for the data to be trasmitted is needed

    -> remember data will be transmitted char by char

    -> using TMR0 as counter of square wave's rising edges X-ray sensor must be on RA4

    -> errata corrige: sampling time, as in 16bit as in 8bit mode, is too long using TMR0 as trigger, TMR1 prescaled can be a solution. 

    -> using only TMR1 interrupt can be critical because too fast averages can give non significant results

    -> consider the idea of measure the width of square wave by using tmr2 interrupt on rising edges and tmr3 interrupt on falling edges using time_ms incresed by TMR1 INTERRUPT (which in this case in better to be used in 8bit mode) 

    -> Save_Data function needs data lenght as input

    -> add at the end of every sampling function the Add_Time_Stamp() function

    -> in the isr must be verified  p_tx_data < p_rx_data 


~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

// #######################################################################################
// #######################################################################################

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ INCLUDES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #include <xc.h>
    #include <stdint.h>         /* For uint8_t definition */
    #include <stdbool.h> 

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ INITILIZATION PRAGMAS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // CONFIG1H
    #pragma config FOSC = INTIO7    // Oscillator Selection bits (Internal oscillator block, CLKOUT function on OSC2)
    #pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
    #pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock enabled)
    #pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
    #pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

    // CONFIG2L
    #pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
    #pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
    #pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

    // CONFIG2H
    #pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
    #pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

    // CONFIG3H
    #pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
    #pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
    #pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
    #pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
    #pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
    #pragma config P2BMX = PORTB5   // ECCP2 B output mux bit (P2B is on RB5)
    #pragma config MCLRE = INTMCLR  // MCLR Pin Enable bit (RE3 input pin enabled; MCLR disabled)

    // CONFIG4L
    #pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
    #pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
    #pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

    // CONFIG5L
    #pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000200-000FFFh) not code-protected)
    #pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (001000-001FFFh) not code-protected)

    // CONFIG5H
    #pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0001FFh) not code-protected)
    #pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

    // CONFIG6L
    #pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000200-000FFFh) not write-protected)
    #pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (001000-001FFFh) not write-protected)

    // CONFIG6H
    #pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
    #pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
    #pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

    // CONFIG7L
    #pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

    // CONFIG7H
    #pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DEFINITIONS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #define _XTAL_FREQ 16000000
    #define ACK '@' 
    #define DIMTIME 10
    #define TIMESTAMPLENGHT 9

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ PROTOTYPES ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
    void ISR_General_Configurator(void);

    void ISR_Serial_Configurator(void);

    void ISR_TMR0_Configurator(void);

    void ISR_TMR1_Configurator(void);

    uint8_t EUSART_Read(uint8_t rx_data);

    void EUSART_Write(uint8_t tx_data);
    
    void Receive_Data_Buffer(void);
    
    void Transmit_Data_Buffer(void);
    
    void Start_Serial_Trasmission(void);
    
    void EUSART_Initialize(void);

    float X_Ray_Read(void);

    uint16_t TMR0_Read16bitTimer(void);

    void Save_Data(uint8_t data_to_be_saved[], uint8_t data_lenght,char identifier);
    
    void Add_Time_Stamp(void);

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ STRUCT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    /*
    typedef struct {
            uint8_t GPS_time[DIMTIME];
            int time_ms;
        } time_full;
    */

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ VARIABLES DECLARATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    int p_tx_data;
    int p_rx_data;
    int time_ms;
    int X_clock;
    uint8_t tx_data[1000];
    uint8_t rx_data[1000];
    uint8_t GPS_time[DIMTIME];
    float X_Ray_Value;
    int counter;

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MAIN ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    void main(void) {
        
        ConfigureOscillator();
            
        ISR_General_Configurator();

        ISR_Serial_Configurator();

        ISR_TMR0_Configurator();

        ISR_TMR1_Configurator();

        EUSART_Initialize();

        while(1){
                
            // BMP180_Read();
                
            // HIH7120_Read();
                
            // GPS_Read_coordinates();
                
            // GPS_time = GPS_Read_time();   ricordarsi di pulire time_ms
                
            X_Ray_Value = X_Ray_Read();
                
            // Refill_Data_Array();

            Start_Serial_Trasmission(); 
        }
            
        return;
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ OSCILLATOR CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~~~~~

    void ConfigureOscillator(void){
       
       // ~~~~~~~~~~~~~~~ Configure Oscillator ~~~~~~~~~~~~

       OSCCONbits.IDLEN = 0;        // bit 7 IDLEN: Idle Enable bit, 1 = Device enters an Sleep mode when a SLEEP instruction is executed
       OSCCONbits.IRCF = 7;         // bit 6-4 IRCF<2:0>: Internal Oscillator Frequency Select bits(2), 111 = HF-INTOSC output frequency is used (16 MHz)
       OSCCONbits.OSTS = 0;         //bit 3 OSTS: Oscillator Start-up Timer Time-out Status bit(1), 0 = Oscillator Start-up Timer (OST) time-out is running: primary oscillator is not ready; device is running from an internal oscillator (HF-INTOSC, MF-INTOSC or LF-INTOSC)
       OSCCONbits.HFIOFS = 1;       // bit 2 HFIOFS: INTOSC Frequency Stable bit, 1 = HF-INTOSC oscillator frequency is stable
       OSCCONbits.SCS = 2;          // bit 1-0 SCS<1:0>: System Clock Select bits(4), 1x = Internal oscillator block (LF-INTOSC, MF-INTOSC or HF-INTOSC)
       OSCCON2bits.SOSCRUN = 0 ;    // bit 6 SOSCRUN: SOSC Run Status bit, 0 = System clock comes from an oscillator other than SOSC
       OSCCON2bits.SOSCGO = 1 ;     // bit 3 SOSCGO: Oscillator Start Control bit, 1 = Oscillator is running, even if no other sources are requesting it
       OSCCON2bits.MFIOFS = 1 ;     // bit 1 MFIOFS: MF-INTOSC Frequency Stable bit, 1 = MF-INTOSC is stable
       OSCCON2bits.MFIOSEL = 0 ;    // bit 0 MFIOSEL: MF-INTOSC Select bit, 0 = MF-INTOSC is not used
       OSCTUNEbits.INTSRC = 1;      // bit 7 INTSRC: Internal Oscillator Low-Frequency Source Select bit, 1 = 31.25 kHz device clock is derived from 16 MHz INTOSC source (divide-by-512 enabled, HF-INTOSC)
       OSCTUNEbits.PLLEN = 1;       // bit 6 PLLEN: Frequency Multiplier PLL Enable bit, 1 = PLL is enabled
       OSCTUNEbits.TUN = 0;         // bit 5-0 TUN<5:0>: Fast RC Oscillator (INTOSC) Frequency Tuning bits, 000000 = Center frequency. Fast RC oscillator is running at the calibrated frequency.

    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ISR CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void ISR_General_Configurator(void){
        
        // ~~~~~~~ GENERAL INTERRUPT INITIALIZATION ~~~~~~~~

            RCONbits.IPEN = 1;          // Enable Priority Interrupt
            INTCONbits.INT0IE = 1;      // Enable Interrupt on external input
            INTCONbits.GIE = 1;         // Enable Global Interrupt 
            INTCONbits.PEIE = 1;        // Enable Peripheral Interrupt 
            INTCONbits.GIEL = 1;        // Enables all low priority interrupts
            INTCONbits.GIEH = 1;        // Enables all high priority interrupts
            
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ISR SERIAL CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~~~~~

    void ISR_Serial_Configurator(void){

        // ~~~~~~~ SERIAL INTERRUPT INITIALIZATION ~~~~~~~~~
        
            PIE1bits.TX1IE = 1;         // Enable the interrupt on serial transmission buffer
            PIE1bits.RC1IE = 1;         // Enable the interrupt on serial recieve buffer
            IPR1bits.TX1IP = 1;         // Set priority of reciver buffer interrupt as high
            IPR1bits.RC1IP = 1;         // Set priority of reciver buffer interrupt as high     
            PIR1bits.RC1IF = 0;         // Clear rx interrupt flag
            PIR1bits.TX1IF = 0;         // Clear tx interrupt flag
       
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ISR TMR0 CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void ISR_TMR0_Configurator(void){   //TMR0 is used as counter
        
        // ~~~~~~~~ TIMER0 INTERRUPT INITIALIZATION ~~~~~~~~~
        
            INTCON2bits.TMR0IP = 1;     // Set priority of TMR0 overflow interrupt as high
            INTCONbits.TMR0IF = 0;      // Clear Timer overflow's flag        
            T0CONbits.TMR0ON = 1;       // Enable Timer
            T0CONbits.T08BIT = 0;       // Timer0 is configured as a 16-bit timer/counter
            T0CONbits.T0CS = 1;         // Set Transition on T0CKI pin as Clock Source 
            T0CONbits.T0SE = 0;         // Increment on low-to-high transition on T0CKI pin
            T0CONbits.PSA = 1;          // TImer0 prescaler is NOT assigned. Timer0 clock input bypasses prescaler
            TMR0H = 0x00;               // Clear the timer 
            TMR0L = 0x00;               // Clear the timer
            INTCONbits.TMR0IF = 0;      // Clear TMR0 interrupt flag
            INTCONbits.TMR0IE = 1;      // Enable TMR0 interrupt 
            INTCONbits.TMR0IE = 1;      // Timer0 overflow Interrupt Enable

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ISR TMR1 CONFIGURATION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    
    void ISR_TMR1_Configurator(void){           
                                
        // ~~~~~~~~ TIMER1 INTERRUPT INITIALIZATION ~~~~~~~~~
                        
            T1CONbits.TMR1ON = 1;       // Enable Timer1
            T1CONbits.T1RD16 = 1;       // Enable register read/write of Timer1 in two 8-bit operation
            T1CONbits.T1SYNC = 1;       // Do not synchronize external clock input
            T1CONbits.T1SOSCEN= 0;      // Disalbe Dedicated Secondary oscillator circuit
            T1CONbits.TMR1CS1 = 0;      // Set system clock (FOSC) as clock source  
            T1CONbits.TMR1CS0 = 1;      // Set system clock (FOSC) as clock source
            T1CONbits.T1CKPS1 = 0;      // Set 1:1 prescaled clock
            T1CONbits.T1CKPS0 = 0;      // Set 1:1 prescaled clock
            T1GCON = 0x00;              // Disable gate functiob
            TMR1H = 0x00;               // Clear the timer 
            TMR1L = 0x00;               // Clear the timer
            IPR1bits.TMR1IP = 1;        // Set priority of TMR0 overflow interrupt as high    
            PIR1bits.TMR1IF = 0;        // Clear TMR1 interrupt flag
            PIE1bits.TMR1IE = 1;        // Enable TMR1 interrupt

        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~           
    
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ HIGH PRIOTIRY ISR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    
    void interrupt high_priority High_Int(void){
        
        // ~~~~~~~~ SERIAL TRANSMISSION ~~~~~~~~~
        
        if (PIR1bits.TX1IF == 1 && PIE1bits.TX1IE == 1) {
            PIE1bits.TX1IE = 0;
            
            Transmit_Data_Buffer();
            
            PIR1bits.TX1IF = 0; 
            PIE1bits.TX1IE = 1;
        }
        
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     
        // ~~~~~~~~ SERIAL RECEPTION ~~~~~~~~~~~~~
        
        if (PIR1bits.RC1IF == 1 && PIE1bits.RC1IE == 1) {
            PIE1bits.RC1IE = 0;
            
            Receive_Data_Buffer();
            
            PIR1bits.RC1IF = 0; 
            PIE1bits.RC1IE = 1;
        }
        
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~    
        
        // ~~~~~~~~~~~~ TIMER0 ~~~~~~~~~~~~~~~~~~~
        
        if (INTCONbits.TMR0IE == 1 && INTCONbits.TMR0IF == 1) {
            INTCONbits.TMR0IE = 0;
            
            X_Ray_Value = X_Ray_Read();
            
            INTCONbits.TMR0IF = 0; 
            INTCONbits.TMR0IE = 1;
        }
        
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~   
        
        // ~~~~~~~~~~~~ TIMER1 ~~~~~~~~~~~~~~~~~~ (clock) 
        
        if (PIR1bits.TMR1IF == 1  && PIE1bits.TMR1IE == 1) {

            PIE1bits.TMR1IE == 0;

            time_ms++;              // time from the beginning of execution
            X_clock++;              // time from the last X_ray sample
            
            
            PIR1bits.TMR1IF == 0;
            PIE1bits.TMR1IE == 1;
        }
        
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~       
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ HIGH PRIOTIRY ISR ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
    void interrupt low_priority Low_Int(void){

    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TIME ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    int Get_Time(void);

    Get_Time(void){

        return time_ms;
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ASYNCHRONOUS EUSART INITIALIZE ~~~~~~~~~~~~~~~~~~~~
    
    void EUSART_Initialize(void){
        
        TXSTA1bits.SYNC = 0;        // Configures the EUSART for Asynchronous Operation
        TXSTA1bits.TXEN = 1;        // Enable the Serial Trasmission Module
        RCSTA1bits.CREN = 1;        // Enable the Serial Receiver Module
        RCSTA1bits.SPEN = 1;        // Enable the Serial Port
        
        TRISCbits.RC6 = 1;          // Set the RC6 as digital input
        TRISCbits.RC7 = 1;          // Set the RC7 as digital input
        
        BAUDCON1bits.BRG16 = 0;     // Enable the 8-bit Baud Rate Generator
        TXSTA1bits.BRGH = 0;        // Select Low Spedd Baud Rate 9600
        
        ANSELCbits.ANSC6=0;         // Disable the analog I/O function on RC6
        ANSELCbits.ANSC7=0;         // Disable the analog I/O function on RC7 

    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ EUSART ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    void EUSART_Write(uint8_t tx_data){

        TXREG1 = tx_data;

        }

    uint8_t EUSART_Read(uint8_t rx_data){  
        
        if (1 == RC1STAbits.OERR) {                 // EUSART1 error - restart
            
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;    

        }

        return RCREG1;

    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SERIAL TRASMISSION ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void Transmit_Data_Buffer(void){
       
       if (p_tx_data < 1000){
     
            TXREG1=tx_data[p_tx_data];
            p_tx_data++;
        
        }    

    }

    void Receive_Data_Buffer(void){
       
       if (p_rx_data < 1000){
     
            rx_data[p_rx_data]=RCREG1;
            p_rx_data++;

        }
        
        if(p_rx_data ==1000){                       // when the array is full overwrite the oldest data by the first line

            p_rx_data=0;
            rx_data[p_rx_data]=RCREG1;

        }

    }

 // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ STRAT SERIAL TRASMISSION ~~~~~~~~~~~~~~~~~~~~~~~~~~

    void Start_Serial_Trasmission(void){            // have to be executed only one time

        int begin = 0;                           
        
        if (begin == 0){
        
            PIE1bits.TX1IE = 0;                     // Disable TX inturrupt
            PIE1bits.RC1IE = 0;                     // Disable RX interrupt
            int i=0;
        
            for (i=0;i++;i<10){                     

                TXREG1 = GPS_time[i];               // trasmit one characeter of time given by the gps
                while (0 == PIR1bits.TX1IF);        // wait until the character is completly trasmitted        }
                i++;                                // step on the next char
                PIR1bits.TX1IF = 0;                 // clear interrupt flag
            }
            
            // wait for acknowledge from the receiver
            
            while (0 == PIR1bits.RC1IF);            // wait until something in received on serial buffer
            char time_received_ack = RCREG1;
                
            if (time_received_ack == ACK){          // check if the ground station gives the correct time_received_ack

                PORTBbits.RB0 = 1;                  // switch on trasmission gps_time succesfully completed notifier led
                PORTBbits.RB1 = 0;                  // switch off error notifier led

            }
                
            if (time_received_ack != ACK){

                PORTBbits.RB1 = 1;                  // switch on error notifier led
                void Start_Serial_Trasmission(void);// retry      

            }
            
        }
                                        
        PIE1bits.TX1IE = 1;                         // Enable TX inturrupt
        PIE1bits.RC1IE = 1;                         // Enable RX interrupt

        TXREG1 = tx_data[0];                          // trasmit the first character
        p_tx_data++;                                // step on the next char
        begin = 1;                                  // Disable Start_Serial_Trasmission fuction
                
        }
        
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ X-RAY SENSOR DRIVER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    float X_Ray_Read(){
                                                // the sensor gives a square wave with variable width from 40 us to 150                                         
        counter = (int)TMR0_Read16bitTimer();
        X_Ray_Value = counter/X_clock;         // Avg sensor Value as (#fronts/observation_time)
        X_clock = 0;                           // Clear the timer used for average
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GPS DRIVER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 
   
    /* 
        void GPS_Read(void);

        void GPS_Read_Data(void);

        -> must be introduced a function that write the data read on the array tx_data[]
    */

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ BPM180 DRIVER ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

        /*
            readBMP180(BPM180_data[]){}

            -> must be introduced a function that write the data read on the array tx_data[]

        */

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ TIMERS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    uint16_t TMR0_Read16bitTimer(void) {
        
        uint16_t readVal;
        uint8_t readValLow;
        uint8_t readValHigh;

        readValLow = TMR0L;
        readValHigh = TMR0H;
        readVal = (uint16_t)(readValLow | readValHigh << 8);

        return readVal;
    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ SAVE DATA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void Save_Data(uint8_t data_to_be_saved[], uint8_t data_lenght, char identifier){

        int i=0;

        //tx_data[p_tx_data] = '$';
        //p_tx_data++;
        
        //tx_data[p_tx_data] = identifier;   // verifica che l'assegnamento sia corretto
        //p_tx_data++;

        for (i=0;i<data_lenght;i++){
            
            tx_data[p_tx_data] = data_to_be_saved[i];
            p_tx_data++; 

        }

    }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ADD TIMESTAMP ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 

    void Add_Time_Stamp(void){

        for (i=0;i<TIMESTAMPLENGHT;i++){
            tx_data[p_tx_data] = time_stamp[i];
            p_tx_data++; 
        }    
    }
    