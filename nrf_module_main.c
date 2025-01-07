/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC16LF18344
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"

/*
                         Main application
 */
/* Memory Map */

#define check_in PORTAbits.RA4
#define Smoke_inp PORTBbits.RB6
#define FAULT 0x02
#define NORMAL 0x10
#define FIRE 0x1a
#define FIRE_FAULT 0x04 

#define CONFIG      0x00
#define EN_AA       0x01
#define EN_RXADDR   0x02
#define SETUP_AW    0x03
#define SETUP_RETR  0x04
#define RF_CH       0x05
#define RF_SETUP    0x06
#define STATUS      0x07
#define OBSERVE_TX  0x08
#define CD          0x09
#define RX_ADDR_P0  0x0A
#define RX_ADDR_P1  0x0B
#define RX_ADDR_P2  0x0C
#define RX_ADDR_P3  0x0D
#define RX_ADDR_P4  0x0E
#define RX_ADDR_P5  0x0F
#define TX_ADDR     0x10
#define RX_PW_P0    0x11
#define RX_PW_P1    0x12
#define RX_PW_P2    0x13
#define RX_PW_P3    0x14
#define RX_PW_P4    0x15
#define RX_PW_P5    0x16
#define FIFO_STATUS 0x17
#define DYNPD       0x1C

/* Bit Mnemonics */

/* configuratio nregister */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define EN_CRC      3
#define CRCO        2
#define PWR_UP      1
#define PRIM_RX     0

/* enable auto acknowledgment */
#define ENAA_P5     5
#define ENAA_P4     4
#define ENAA_P3     3
#define ENAA_P2     2
#define ENAA_P1     1
#define ENAA_P0     0

/* enable rx addresses */
#define ERX_P5      5
#define ERX_P4      4
#define ERX_P3      3
#define ERX_P2      2
#define ERX_P1      1
#define ERX_P0      0

/* setup of address width */
#define AW          0 /* 2 bits */

/* setup of auto re-transmission */
#define ARD         4 /* 4 bits */
#define ARC         0 /* 4 bits */

/* RF setup register */
#define PLL_LOCK    4
#define RF_DR       3
#define RF_PWR      1 /* 2 bits */

/* general status register */
#define RX_DR       6
#define TX_DS       5
#define MAX_RT      4
#define RX_P_NO     1 /* 3 bits */
#define TX_FULL     0

/* transmit observe register */
#define PLOS_CNT    4 /* 4 bits */
#define ARC_CNT     0 /* 4 bits */

/* fifo status */
#define TX_REUSE    6
#define FIFO_FULL   5
#define TX_EMPTY    4
#define RX_FULL     1
#define RX_EMPTY    0

/* dynamic length */
#define DPL_P0      0
#define DPL_P1      1
#define DPL_P2      2
#define DPL_P3      3
#define DPL_P4      4
#define DPL_P5      5

/* Instruction Mnemonics */
#define R_REGISTER    0x00 /* last 4 bits will indicate reg. address */
#define W_REGISTER    0x20 /* last 4 bits will indicate reg. address */
#define REGISTER_MASK 0x1F
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define NOP           0xFF

//#ifndef NRF24
#define NRF24

//#include "nRF24L01.h"
//#include <stdint.h>

#define LOW 0
#define HIGH 1

#define nrf24_ADDR_LEN 5
#define nrf24_CONFIG ((1<<EN_CRC)|(0<<CRCO)|(1<<MASK_TX_DS)|(1<<MASK_MAX_RT))

#define NRF24_TRANSMISSON_OK 0
#define NRF24_MESSAGE_LOST   1

unsigned int check_for_1t_time = 0;
unsigned int check_for_3t_high_time = 0;
unsigned char check_for_2t_low_time = 0;
unsigned int check_for_g3t_high_time = 0;
unsigned int g3t = 0;
unsigned int g3t_condition_checked = 0;
unsigned char byte_send = 1,num_of_cycle = 0,data_read = 0,bit_send = 0;
unsigned int a = 0,Timer_Count = 0;
unsigned char module_status = NORMAL,tmp_module_status = NORMAL;
unsigned int T5_Overflow = 0;
unsigned int mode;
    unsigned char A[8],fire_values[3] ={0x37,0x3b,0x32};
    unsigned int AP = 0;
    unsigned char Command_bits = 0;
    unsigned int Parity_Count = 0;
    unsigned int Address = 0x00;
    unsigned int Switch_Address = 0x00;
    unsigned int No_Of_Ones_Return_Data = 0;
    unsigned int Status_byte   = 0x08;
    int i = 0;
    int j = 0;


unsigned int temp;
unsigned int q = 0;
unsigned int data_array[5];
unsigned int tx_address[5] = {0,0,0,1,0x2};
unsigned int rx_address[5] = {0,0,0,1,0x1};

unsigned int count = 0;
/*void blink_led()
{
    if(a == 1 )
    {
        a = 0;
        PORTBbits.RB4 = 1;
    }
    else if (a == 0)
    {
        a = 1;
        PORTBbits.RB4 = 0;
    }
}
*/
/* checking Pos_cycle for 3 condition & the variation can be (-5% to +5%)
  1t_high = 97.5 & 3t_high = 292.5
 1) check the pulse greater than 3t high
 2) check the pulse for 3t high
 3) check the pulse for 1t high*/

/*condition check for falling or rising edge. And counting the timer count*/
void check_timer()
{
    
    T0CON0bits.T0EN = 0;
    Timer_Count = ((unsigned int)(256*TMR0H)+TMR0L);
    
    if  (/*371 < Timer_Count && */ Timer_Count <= 410)
    {
        check_for_1t_time = 1;
    }
    else if( Timer_Count > 410 && Timer_Count < 819 )
    {
        check_for_2t_low_time = 1;
    }
    else if (1000 < Timer_Count && Timer_Count < 1229 )
    {
        check_for_3t_high_time = 1;
    }
    else if (1229 < Timer_Count) 
    {
        check_for_g3t_high_time = 1;
        g3t = 1;
    }
    TMR0H = 0x00;
    TMR0L = 0x00;
    T0CON0bits.T0EN = 1;
    
}

unsigned char parity_checker(unsigned char parity_data)
{	
	unsigned char j, k = 0x00;
    for(j = 0x00; j < 8; j++)
	{
		if((parity_data & 0x01) == 0x01)
	 	{
	 		k++;
		}
		parity_data = parity_data >> 1;
	}
	if((k % 0x02) == 0x00)
		return 1;	
	else
		return 0;	
}

/*configuration of timer0 setting it as 16 bit timer.
 Pre-scaler & Post-scaler is set as 1:1*/
void Timer0_initialize()
{
    //PIE0bits.TMR0IE = 1;
    T0CON0 = 0x10;
    T0CON1 = 0x40;
    TMR0H = 0x00;
    TMR0L = 0x00;
}

/*configuration of timer1 & it is a 16bit timer*/
void Timer1_initialize()
{
    TMR1IF = 0;
    T1CON = 0x0C;
    TMR1H = 0x00;
    TMR1L = 0x00;
}


/*Initializing the RA2 as a constant current source
 *RA5 has a ADC to check the conditions
*/


void PORTA_initialize()
{
    TRISA = 0b00110000;
    ANSELA = 0b00100000; 
}

/* Initializing the RB4 has a test led. It will glow when we drive it low*/

void PORTB_initialize()
{
    TRISB = 0x40;
    ANSELB = 0x00;
}

/*Initializing the ADC for checking the condition.
 RA5 is used as ADC pin*/

void relay_on()
{ 
    PORTBbits.RB7 = 1;
    __delay_ms(50);
    PORTBbits.RB7 = 0;

}

/* software spi routine */
unsigned short spi_transfer(unsigned short tx)
{
    short int rs = 0;
    rs = SPI1_Exchange8bit(tx);
    return rs;
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(unsigned short* dataout,unsigned short* datain,unsigned short len)
{
    unsigned int i;

    for(i=0;i<len;i++)
    {
       datain[i] = spi_transfer(dataout[i]);
    }

}

// send multiple bytes over SPI
void nrf24_transmitSync(unsigned short* dataout,unsigned short len)
{
    unsigned int i;

    for(i=0;i<len;i++)
    {
      spi_transfer(dataout[i]);
    }

}
 //Clocks only one byte into the given nrf24 register */



void nrf24_configRegister(unsigned short reg, unsigned short value)
{
    CSN_SetLow();
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
    CSN_SetHigh();
}

/* Read single register from nrf24 */
void nrf24_readRegister(unsigned short reg, unsigned short* value, unsigned short len)
{
    CSN_SetLow();
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    nrf24_transferSync(value,value,len);
    CSN_SetHigh();
}

/* Write to a single register of nrf24 */
void nrf24_writeRegister(unsigned short reg, unsigned short* value, unsigned short len)
{
    CSN_SetLow();
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    nrf24_transmitSync(value,len);
    CSN_SetHigh();
}



unsigned short payload_len;

// init the hardware pins
void nrf24_init()
{
    //nrf24_setupPins();
    CS_SetLow();
    CSN_SetHigh();
}

void nrf24_powerUpRx()
{
    CS_SetLow();
    CSN_SetLow();
    spi_transfer(FLUSH_RX);
    CSN_SetHigh();

    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    CS_SetLow();
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));
    CS_SetHigh();
}

void nrf24_config(unsigned short channel, unsigned short pay_length)
{
    /* Use static payload length ... */
    payload_len = pay_length;

    // Set RF channel
    nrf24_configRegister(RF_CH,channel);

    // Set length of incoming payload
    nrf24_configRegister(RX_PW_P0, 0x00); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, payload_len); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, 0x00); // Pipe not used
    nrf24_configRegister(RX_PW_P3, 0x00); // Pipe not used
    nrf24_configRegister(RX_PW_P4, 0x00); // Pipe not used
    nrf24_configRegister(RX_PW_P5, 0x00); // Pipe not used

    // 250 Kbps, TX gain: 0dbm
    nrf24_configRegister(RF_SETUP, (1<<RF_DR)|((0x03)<<RF_PWR));

    // CRC enable, 1 byte CRC length
   nrf24_configRegister(CONFIG,nrf24_CONFIG);

    // Auto Acknowledgment
    nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(0<<ENAA_P2)|(0<<ENAA_P3)|(0<<ENAA_P4)|(0<<ENAA_P5));

    // Enable RX addresses
    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(0<<ERX_P2)|(0<<ERX_P3)|(0<<ERX_P4)|(0<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_configRegister(SETUP_RETR,(0x02<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
    nrf24_configRegister(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

    // Start listening
    nrf24_powerUpRx();
}




void nrf24_powerUpTx()
{
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT));

    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(0<<PRIM_RX)));
}
/*
void nrf24_powerDown()
{
    CS_SetLow();
    nrf24_configRegister(CONFIG,nrf24_CONFIG);
}
*/
/* Set the RX address */
void nrf24_rx_address(unsigned short * adr)
{
    CS_SetLow();
    nrf24_writeRegister(RX_ADDR_P1,adr,nrf24_ADDR_LEN);
    CS_SetHigh();
}


/* Set the RX address
void nrf24_rx_address(unsigned short * adr)
{
    CS_SetLow();
    nrf24_writeRegister(RX_ADDR_P1,adr,nrf24_ADDR_LEN);
    CS_SetHigh();
}
*/
/* Returns the payload length */
/*unsigned short nrf24_payload_length()
{
    return payload_len;
}*/

/* Set the TX address */
void nrf24_tx_address(unsigned short* adr)
{
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_writeRegister(RX_ADDR_P0,adr,nrf24_ADDR_LEN);
    nrf24_writeRegister(TX_ADDR,adr,nrf24_ADDR_LEN);
}

/* Checks if receive FIFO is empty or not */
unsigned short nrf24_rxFifoEmpty()
{
    unsigned short fifoStatus;

    nrf24_readRegister(FIFO_STATUS,&fifoStatus,1);

    return (fifoStatus & (1 << RX_EMPTY));
}
/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */

/* Returns the length of data waiting in the RX fifo */
/*
unsigned short nrf24_payloadLength()
{
    unsigned short status;
    CSN_SetLow();
    spi_transfer(R_RX_PL_WID);
    status = spi_transfer(0x00);
    CSN_SetHigh();
    return status;
}*/

/* Reads payload bytes into data array */
void nrf24_getData(unsigned int* data1)
{
    /* Pull down chip select */
    CSN_SetLow();

    /* Send cmd to read rx payload */
    spi_transfer( R_RX_PAYLOAD );

    /* Read payload */
    nrf24_transferSync(data1,data1,payload_len);

    /* Pull up chip select */
    CSN_SetHigh();

    /* Reset status register */
    nrf24_configRegister(STATUS,(1<<RX_DR));
}

/* Returns the number of retransmissions occured for the last message */
/*
unsigned short nrf24_retransmissionCount()
{
    unsigned short rv;
    nrf24_readRegister(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}*/

// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(unsigned short* value)
{
    /* Go to Standby-I first */
    CS_SetLow();

    /* Set to transmitter mode , Power up if needed */
    nrf24_powerUpTx();
    
    __delay_us(150);

    /* Do we really need to flush TX fifo each time ? */
   // #if 1
        /* Pull down chip select */
        CSN_SetLow();

        /* Write cmd to flush transmit FIFO */
        spi_transfer(FLUSH_TX);

        /* Pull up chip select */
        CSN_SetHigh();
  //  #endif

    /* Pull down chip select */
    CSN_SetLow();

    /* Write cmd to write payload */
    spi_transfer(W_TX_PAYLOAD);

    /* Write payload */
    nrf24_transmitSync(value,payload_len);

    /* Pull up chip select */
    CSN_SetHigh();

    /* Start the transmission */
    CS_SetHigh();
}



unsigned short nrf24_getStatus()
{
    unsigned short rv;
    CSN_SetLow();
    rv = spi_transfer(NOP);
    CSN_SetHigh();
    return rv;
}

unsigned short nrf24_lastMessageStatus()
{
    unsigned short rv;

    rv = nrf24_getStatus();

    /* Transmission went OK */
    if((rv & ((1 << TX_DS))))
    {
        return NRF24_TRANSMISSON_OK;
    }
    /* Maximum retransmission count is reached */
    /* Last message probably went missing ... */
    else if((rv & ((1 << MAX_RT))))
    {
        return NRF24_MESSAGE_LOST;
    }
    /* Probably still sending ... */
    else
    {
        return 0xFF;
    }
}

unsigned short nrf24_isSending()
{
    unsigned short status;

    /* read the current status */
    status = nrf24_getStatus();

    /* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
    if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
    {
        return 0; /* false */
    }

    return 1; /* true */

}
unsigned short nrf24_dataReady()
{
    // See note in getData() function - just checking RX_DR isn't good enough
    unsigned short status = nrf24_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) )
    {
        return 1;
    }

    return !nrf24_rxFifoEmpty();
}
void check_smk()
{
            /*    IRQ_SetLow();
                __delay_ms(10);
                IRQ_SetHigh();       
                __delay_ms(10);
                IRQ_SetLow();
                __delay_ms(10);
                IRQ_SetHigh();*/
/* Fill the data buffer */
        data_array[0] = 0x55;
        data_array[1] = 0x00;
        data_array[2] = 0x00;
        data_array[3] = 0x00;
       // data_array[18] = 0x61;
        count = 0;

         send_again:
        CLRWDT();
        /* Automatically goes to TX mode */
        nrf24_send(data_array);

        /* Wait for transmission to end */
        while(nrf24_isSending());

        /* Make analysis on last tranmission attempt */
        if(nrf24_lastMessageStatus() == NRF24_MESSAGE_LOST)
        {
            count++;
            if(count > 3)
            {
                if(tmp_module_status != FIRE)
                    module_status = FAULT;
                return;
            }
            goto send_again;
        }
         
        nrf24_powerUpRx();
        __delay_us(150);
        count = 0;
        data_array[2] = 0x00;
        while(!nrf24_dataReady())
        {
                count++;
                if(count > 1000)
                {
                    if(tmp_module_status != FIRE)
                        module_status = FAULT;
                    return;
                    //break;
                }
        }
        nrf24_getData(data_array);
        if (data_array[1] != 0)
        {
            if((data_array[1] & 1) == 1)
                module_status = fire_values[0];
            else if(((data_array[1] >> 1) & 1) == 1)
                module_status = fire_values[1];
            else if(((data_array[1] >> 2) & 1) == 1)
                module_status = fire_values[2];
            tmp_module_status = FIRE; 
        }
        else if(data_array[2] > 0)
        {
            if(tmp_module_status != FIRE)
                module_status = NORMAL; 
        }
}

void check_address_from_panel(unsigned char address_bit)
{
    if(check_for_3t_high_time == 1)
    {
        check_for_3t_high_time = 0;
        A[address_bit] = 1;
        Parity_Count++;
        while(check_in == 0);
        check_timer();
    }
    else if(check_for_1t_time == 1)
    {
        check_for_1t_time = 0;
        A[address_bit] = 0;
        while (check_in == 0);
        check_timer();
    }
    else 
    {
        bit_send = 0xff;
    }

    if((check_for_1t_time == 1) && (A[address_bit] == 1))
    {
        check_for_1t_time = 0;
        while (check_in == 1);
        check_timer();
    }
    else if((check_for_1t_time == 1) && (A[address_bit] == 0))
    {
        check_for_1t_time = 0;
        while (check_in == 1);
        check_timer();

        if(check_for_1t_time == 1)
        {
            check_for_1t_time = 0;
            while (check_in == 0);
            check_timer();
        }
        else
        {
        bit_send = 0xff;
        }

        if (check_for_1t_time == 1)
        {
            check_for_1t_time = 0;
            while (check_in == 1);
            check_timer();
        }
        else 
        {
         bit_send = 0xff;
        }
    } 
    else
    {
        bit_send = 0xff;
    }
        
}
void respond_for_panel()
{    
        // Add your application code        
        here:
        CLRWDT();
         Parity_Count = 0;
                T0CON0bits.T0EN = 0; 
                PORTA = 0b00000010;
//              PORTAbits.RA1 = 1;
                TMR0IF = 0;
                check_for_g3t_high_time = 0;
                g3t = 0;
                Address = 0x00;
                A[7] = 0; A[6] = 0; A[5] = 0; A[4] = 0; A[3] = 0;
                A[2] = 0; A[1] = 0; A[0] = 0; AP = 0;
                TMR0H = 0x00;
                TMR0L = 0x00;
      //  Switch_Address = ((128*S8)+(64*S7)+(32*S6)+(16*S5)+(8*S4)+(4*S3)+(2*S2)+(1*S1));
      //  Switch_Address = 255-Switch_Address;
        
                
        Switch_Address = 19;
        if (Switch_Address == 255)
        {
       //     PORTBbits.RB4 = 0;
            __delay_ms(2000);
       //     PORTBbits.RB4 = 1;
        }
        
        while (check_in == 0);
        
        TMR0H = 0x00;
        TMR0L = 0x00;
        
        T0CON0bits.T0EN = 1;
               
        while (check_in == 1);
        check_timer();
        
        check_3gt:  
            if(g3t == 1) 
            {
               if (check_for_g3t_high_time == 1)
                {
                    check_for_g3t_high_time = 0;
                    while(check_in == 0);
                    check_timer();
                }
                else 
                {
                    goto wrong_frame ;
                }
     
            neg_pulse:   
                if (check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                }
                       
            clock:
                if (check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    while(check_in == 0);
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                }
        
                if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                }
                     
                    
            start_trans: 
                if (check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    while(check_in == 0);
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                }
        
                if (check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                }
        
            Mode:
                if(check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    mode = 1;
                    Parity_Count++;
                    while(check_in == 0);
                    check_timer();
                }
                else if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    mode = 0;
                    while (check_in == 0);
                    check_timer();
                }
                else 
                {
                    goto wrong_frame;
                }
        
                if((check_for_1t_time == 1) && (mode == 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                }
                else if((check_for_1t_time == 1) && (mode == 0))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                         
                    if(check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 0);
                        check_timer();
                    }
                    else
                    {
                        goto wrong_frame;
                    }
                         
                    if (check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 1);
                        check_timer();
                    }
                    else 
                    {
                        goto wrong_frame;
                    }
                } 
                else
                {
                    goto wrong_frame;
                }
        
        A7_BIT:
            bit_send = 0;
            check_address_from_panel(7);
            if(bit_send == 0xff)
                goto wrong_frame;
        A6_BIT: 
            bit_send = 0;
            check_address_from_panel(6);
            if(bit_send == 0xff)
                goto wrong_frame;
        A5_BIT:
            bit_send = 0;
            check_address_from_panel(5);
            if(bit_send == 0xff)
                goto wrong_frame;
        A4_BIT:
            bit_send = 0;
            check_address_from_panel(4);
            if(bit_send == 0xff)
                goto wrong_frame;
        A3_BIT:   
            bit_send = 0;
            check_address_from_panel(3);
            if(bit_send == 0xff)
                goto wrong_frame;
        A2_BIT:
            bit_send = 0;
            check_address_from_panel(2);
            if(bit_send == 0xff)
                goto wrong_frame;
        A1_BIT:
            bit_send = 0;
            check_address_from_panel(1);
            if(bit_send == 0xff)
                goto wrong_frame;
        A0_BIT:
            bit_send = 0;
            check_address_from_panel(0);
            if(bit_send == 0xff)
                goto wrong_frame;
        AP_BIT: 
                if(check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    AP = 1;
                    while(check_in == 0);
                    check_timer();
                }
                else if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    AP = 0;
                    while (check_in == 0);
                    check_timer();
                }
                else 
                {
                    goto wrong_frame;
                }
        
                if((check_for_1t_time == 1) && (AP == 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                }
                else if((check_for_1t_time == 1) && (AP == 0))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                         
                    if(check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 0);
                        check_timer();
                    }
                    else
                    {
                        goto wrong_frame;
                    }
                         
                    if (check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 1);
                        check_timer();
                    }
                    else 
                    {
                        goto wrong_frame;
                    }
                } 
                else
                {
                    goto wrong_frame;
                }
        
        
            Check_Parity_Cond:  
                if((Parity_Count % 2 == 0) && (AP == 1))
                {
                    goto COMMAND_MSB;
                }
                else if((Parity_Count % 2 == 1) && (AP == 0))
                {
                    goto COMMAND_MSB;
                }
                else
                {
                    goto wrong_frame;
                }
        
            COMMAND_MSB:
            if((mode == 1) || (mode ==0) )
            {
                if(check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    Command_bits = 1;
                    while(check_in == 0);
                    check_timer();
                }
                else if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    Command_bits = 0; 
                    while (check_in == 0);
                    check_timer();
                }
                else 
                {   
                    goto wrong_frame;
                }
        
                if((check_for_1t_time == 1) && (Command_bits == 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                }
                else if((check_for_1t_time == 1) && (Command_bits == 0))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                        
                    if(check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 0);
                        check_timer();
                    }
                    else
                    {
                        goto wrong_frame;
                    }
                         
                    if (check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 1);
                        check_timer();
                    }
                    else 
                    {
                        goto wrong_frame;
                    }
                } 
                else
                {
                    goto wrong_frame;
                } 
                Command_bits  = (Command_bits << 1);
                
                if(check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    
                    Command_bits |= 0x01;  
                    while(check_in == 0);
                    check_timer();
                }
                else if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    Command_bits |= 0; 
                    while (check_in == 0);
                    check_timer();
                }
                else 
                {
                    goto wrong_frame;
                }
        
                if((check_for_1t_time == 1) && ((Command_bits & 0x01) == 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                }
                else if((check_for_1t_time == 1) && ((Command_bits & 0x01) != 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                         
                    if(check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 0);
                        check_timer();
                    }
                    else
                    {
                        goto wrong_frame;
                    }
                    if (check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 1);
                        check_timer();
                    }
                    else 
                    {
                        goto wrong_frame;
                    }
                } 
                else
                {
                    goto wrong_frame;
                }
            
            }
            if(mode == 0)
            {
                Command_bits = Command_bits <<1;
                
                if(check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    Command_bits |= 1;
                    while(check_in == 0);
                    check_timer();
                }
                else if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    Command_bits |= 0; 
                    while (check_in == 0);
                    check_timer();
                }
                else 
                {   
                    goto wrong_frame;
                }
        
                if((check_for_1t_time == 1) && ((Command_bits & 0x01) == 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                }
                else if((check_for_1t_time == 1) && ((Command_bits & 0x01) != 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                        
                    if(check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 0);
                        check_timer();
                    }
                    else
                    {
                        goto wrong_frame;
                    }
                         
                    if (check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 1);
                        check_timer();
                    }
                    else 
                    {
                        goto wrong_frame;
                    }
                } 
                else
                {
                    goto wrong_frame;
                }  
                Command_bits = Command_bits <<1;
                if(check_for_3t_high_time == 1)
                {
                    check_for_3t_high_time = 0;
                    Command_bits |= 1;  
                    while(check_in == 0);
                    check_timer();
                }
                else if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    Command_bits |= 0; 
                    while (check_in == 0);
                    check_timer();
                }
                else 
                {
                    goto wrong_frame;
                }
        
                if((check_for_1t_time == 1) && ((Command_bits & 0x01) == 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                }
                else if((check_for_1t_time == 1) && ((Command_bits & 0x01) != 1))
                {
                    check_for_1t_time = 0;
                    while (check_in == 1);
                    check_timer();
                         
                    if(check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 0);
                        check_timer();
                    }
                    else
                    {
                        goto wrong_frame;
                    }
                    if (check_for_1t_time == 1)
                    {
                        check_for_1t_time = 0;
                        while (check_in == 1);
                        check_timer();
                    }
                    else 
                    {
                        goto wrong_frame;
                    }
                } 
                else
                {
                    goto wrong_frame;
                }
                
            }
        
            CHECK_ADD:  
                Address = ((128*A[7])+(64*A[6])+(32*A[5])+(16*A[4])+(8*A[3])+(4*A[2])+(2*A[1])+(1*A[0]));
                                           
                
                if(Address == Switch_Address)
                {
                    goto END_CLOCK;
                }
                else
                {
                    goto wrong_frame;
                }
                     
        
            END_CLOCK: 
                if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    while(check_in == 0);
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                }
                        
                if(check_for_1t_time == 1)
                {
                    check_for_1t_time = 0;
                    while (check_in == 1); 
                    check_timer();
                }
                else
                {
                    goto wrong_frame;
                } 
                      
                        
            Start_Return_Data:  
            if(mode == 0)
            {
                if(Command_bits == 0b1110)
                {
                  //  test();
                    goto here;
                }
                if(Command_bits == 0b1011)
                {
                    relay_on();
                    goto here;
                }
                if(Command_bits == 0b1000)
                { 
                    PORTBbits.RB4 = 0;
                    PORTBbits.RB5 = 1;
                    __delay_ms(20);
                    PORTBbits.RB5 = 0; 
                    PORTBbits.RB4 = 1;
                  //  RESET();
                }
                if(Command_bits == 0b0101)
                { 
                    RESET();
                }
            }
            if(Command_bits == 0b01)
            {
                Status_byte =  module_status;
                Status_byte = Status_byte << 1;
                Status_byte |= 0x101;
                goto start_send;
            }
            start_send:
            if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                    PORTA = 0b00000110;
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                    
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_DI:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                    
                    if ((Status_byte & 0x40) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                    
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                    
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_D4:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                    
                    if ((Status_byte & 0x20) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                    
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_D3:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                    
                    if ((Status_byte & 0x10) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_D2:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                
                    if ((Status_byte & 0x08) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_D1:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
                
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                
                    if ((Status_byte & 0x04) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_D0:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
            
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                
                    if ((Status_byte & 0x02) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Return_Data_Parity:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
            
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                    
                    if ((Status_byte & 0x01) == 0)
                    {
                        PORTA = 0b00000010;
                    }
                    else
                    {
                        No_Of_Ones_Return_Data++;
                        PORTA = 0b00000110;
                    }
                    
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
        
            Stop_Return_Data:
                //Condition Failed as the timer overflow before edge change
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                }
            
                if( check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    __delay_us(15);
                    PORTA = 0b00000110;
                    while(check_in == 0);
                    PORTA = 0b00000010;
                    check_timer();
                
                    if( check_for_2t_low_time != 1  )
                    {
                        goto wrong_frame;                
                    }
                    else
                    {
                        check_for_2t_low_time = 0;
                        while(check_in == 1);
                        check_timer();
                    }
            
                    if( check_for_1t_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_1t_time = 0;
                        while(check_in == 0);
                        check_timer();
                    }
                }
                
            Stop_Transaction:
                if(check_for_1t_time != 1)
                {
                    goto wrong_frame;
                }
                else
                {
                    check_for_1t_time = 0;
                    while(check_in == 1);
                    check_timer();
                
                    if(check_for_3t_high_time != 1 )
                    {
                        goto wrong_frame;
                    }
                    else
                    {
                        check_for_3t_high_time = 0;
                        while(check_in == 0);
                        check_timer();
                        
                        if(check_for_1t_time != 1)
                        {
                            goto wrong_frame;
                        }
                        else
                        {
                            check_for_1t_time = 0;
                            T0CON0bits.T0EN = 0;
                            TMR0H = 0;
                            TMR0L = 0;
                        }
                    }   
                }
            data_read++;
            goto here;
                
            wrong_frame:
            
            if(data_read > 1)
            {
                data_read = 0;
                check_smk();
            }
        }
}

//before main initilization

void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    __delay_ms(1500);
    CLRWDT();
    nrf24_init();

    /* Channel #2 , payload length: 4 */
    nrf24_config(2,4);

    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    //INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();

    Timer0_initialize();
    Timer1_initialize();
   // PORTC_initialize();
    PORTA_initialize();
    //ADC_initialize();
    PORTB_initialize();
    
    
    PORTAbits.RA1 = 1;
    
    PORTBbits.RB4 = 0;
    __delay_ms(500);
    PORTBbits.RB4 = 1;
        
    if (Switch_Address == 255)
    {
    //    PORTBbits.RB4 = 0;
      //  __delay_ms(2000);
    //    PORTBbits.RB4 = 1;
    }
    
    while (1)
    {        
        CLRWDT();
        respond_for_panel();
    }
}
/**
 End of File            
*/