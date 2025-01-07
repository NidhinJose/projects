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

#define detector_no 1
#define last_detector 2
/* Memory Map */
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
#define rx_data 18         //suraj

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


#define nrf24_ADDR_LEN 5
#define nrf24_CONFIG ((1<<EN_CRC)|(0<<CRCO)|(1<<MASK_TX_DS)|(1<<MASK_MAX_RT))

#define NRF24_TRANSMISSON_OK 0
#define NRF24_MESSAGE_LOST   1

#define detected 1

unsigned int temp;
unsigned int q = 0;
unsigned int data_array[20];
unsigned int tx_address[5] = {0,0,0,0,0x1};
unsigned int rx_address[5] = {0,0,0,0,0x2};
unsigned int UCSmoke_Detect = 0,T1_Count = 0;
unsigned int ir = 0, blue = 0, dark_voltage = 0, red_count = 3, count = 0, UCsmoke_detect = 0;
unsigned int blue_init3 =0, blue_init2 =0,blue_init1 =0,blue_init =0;

unsigned int blue_recent[11],ascii[5];
unsigned int ir_recent[11],initial = 15;
 
unsigned char blue_count = 0,ir_count = 0,Smk_Status = 0,status_count = 0,ir_status_count = 0,blue_status_count = 0;

unsigned int Detector_No = 10,ir_init = 0;
unsigned int ir_init3 =0, ir_init2 =0, ir_init1 = 0;
unsigned char re_trans_count_from_prev_dev = 0,tmp_count = 0;
unsigned int ackDataArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     //suraj
unsigned int send_to_main[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     //suraj
unsigned int get_data = 0, poledSwitchStatus = 0, pollingSwitchTimeout = 0;
unsigned int countRespSwb = 2, prevStoredSwb = 0, pollinStatus = 0,tempVariable = 0,switchStatus = 0;
unsigned int pollingSwbArray[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned int initiallSwbNo = 3;
unsigned char shiftRegPinArr[5] = {11, 15, 14, 13, 12};
unsigned int led_data = 0;
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

//send multiple bytes over SPI
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
    nrf24_configRegister(SETUP_RETR,(0x03<<ARD)|(0x0F<<ARC));

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

void nrf24_powerDown()
{
    CS_SetLow();
    nrf24_configRegister(CONFIG,nrf24_CONFIG);
}
void nrf24_standby1()
{
    CS_SetLow();
}
void nrf24_standby1_to_active()
{
    CS_SetHigh();
}
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
unsigned short nrf24_payload_length()
{
    return payload_len;
}

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
unsigned short nrf24_payloadLength()
{
    unsigned short status;
    CSN_SetLow();
    spi_transfer(R_RX_PL_WID);
    status = spi_transfer(0x00);
    CSN_SetHigh();
    return status;
}

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
unsigned short nrf24_retransmissionCount()
{
    unsigned short rv;
    nrf24_readRegister(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

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
    __delay_ms(10);

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

void txMode (unsigned int *txData) // transmitting data function
{
    get_data = 0;
    count = 0;
    __delay_us(200);
    //    if (txData[1] != 0x40)
    //    {
    //        EUSART_Write(0x60);
    //        EUSART_Write(tx_address[4]);
    //        for (unsigned int i = 0; i < txData[1] + 2; i++)
    //        {
    //            EUSART_Write(txData[i]);
    //        }
    //
    //    }

    nrf24_send(txData); // sending data
    CLRWDT();
    while (nrf24_isSending())
        ;
    if (nrf24_lastMessageStatus() == NRF24_TRANSMISSON_OK) // check auto acknowledge status if send successfully means it give ok
    {
        EUSART_Write(0x79);
        EUSART_Write(tx_address[4]);
    }
    else
    {
    send_again:
        nrf24_send(txData);
        CLRWDT();
        while (nrf24_isSending())
            ;
        __delay_us(150);
        if (nrf24_lastMessageStatus() == NRF24_TRANSMISSON_OK) // check auto acknowledge status if send successfully means it give ok
        {
            EUSART_Write(0x76);
            EUSART_Write(tx_address[4]);
        }
        else
        {
            count++;

            if (count > 5) // sending three times data
            {
                if ((txData[0] == 0x80) && (txData[3] == rx_data) && (txData[2] == 0x02) || txData[1] == 0x40) // this for if it polling command or path through different board send check ACK status and info to nearest board this information
                {
                    //                    EUSART_Write(0x52);
                    //                    EUSART_Write(send_to_main[get_data]);
                    goto here;
                }
                else if (txData[0] == 0x80 && txData[1] == 0x01 && txData[2] == 0x01 && send_to_main[get_data] == 0x02)
                {
                    //                    EUSART_Write(0x53);
                    //                    EUSART_Write(send_to_main[get_data]);
                    goto here;
                }

                else if (send_to_main[get_data] != 0) // using the nearest  board address stored in array using that pass the information to main board
                {
                    count = 0;
                    //                    EUSART_Write(0x51);
                    //                    EUSART_Write(send_to_main[get_data]);
                    tx_address[4] = send_to_main[get_data];
                    nrf24_tx_address(tx_address);
                    get_data = ++get_data;
                    goto send_again;
                }
                else
                {
                    goto here;
                }
            }
            goto send_again;
        }
    here:
        count = 0;
    }
}

switchStatusSendSuff(unsigned int length, unsigned int command, unsigned int data)       //suraj
{
    // transmintAgain:
    ackDataArray[0] = 0x80;
    ackDataArray[1] = length;
    ackDataArray[2] = command;
    ackDataArray[3] = data;
    ackDataArray[length] = rx_data;
    ackDataArray[length + 1] = 0x81;
    ackDataArray[length + 2] = 0x00;
    tx_address[4] = send_to_main[0];
    //    tx_address[4] = 0x11;
    nrf24_tx_address(tx_address);
    txMode(ackDataArray);
    //    if (nrf24_lastMessageStatus() == NRF24_TRANSMISSON_OK) // if cant accessible that board then send the NACK to main board
    //    {
    //        goto scussfullyTransmited;
    //    }
    //    else
    //    {
    //        goto transmintAgain;
    //    }
    // scussfullyTransmited:
    nrf24_powerUpRx();
}


void pollingRspDiffBoard(unsigned int pollSwb) // this function is for polling response to polled board
{
    unsigned int tempArray[5];

    unsigned int pathExistOrNot = 0;
    //    //    EUSART_Write(0x22);
    tempArray[0] = 0x80;
    tempArray[1] = 0x03;
    tempArray[2] = 0x02;
    tempArray[3] = rx_data;
    tempArray[4] = pollSwb;
    tempArray[5] = 0x00;
    tx_address[4] = pollSwb;
    nrf24_tx_address(tx_address);
    txMode(tempArray);
    if (nrf24_lastMessageStatus() == NRF24_TRANSMISSON_OK) // if send data successfully send then it will stored in array
    {
        for (unsigned int i = 0; i <= 10; i++)
        {
            if (send_to_main[i] == pollSwb)
            {
                pathExistOrNot = 1;
                break;
            } // clear
        }
        if (pathExistOrNot != 1)
        {
            for (int i = 0; i <= 10; i++)
            {
                if (send_to_main[i] == 0)
                {
                    send_to_main[i] = pollSwb;
                    EUSART_Write(send_to_main[i]);
                    EUSART_Write(0x22);
                    break;
                }
            }
        }
    }
}

void pollingDiffBoard(unsigned int *otherBoardData) // using this board passing data to next board
{
    unsigned int tempArray[20];
    unsigned int length = 0;
    length = otherBoardData[1];
    if (otherBoardData[2] == 0x02 || otherBoardData[2] == 0x01) // 0x02 is polling cmd
    {
        length = --length;
        otherBoardData[1] = length;

        tx_address[4] = otherBoardData[length + 3];    // next board data for switch status data
        for (unsigned int i = 0; i <= length + 4; i++) // self storing data//
        {
            tempArray[i] = otherBoardData[i]; // load the data information
        }
        prevStoredSwb = otherBoardData[length + 5];

        nrf24_tx_address(tx_address);
        txMode(tempArray);
        if (nrf24_lastMessageStatus() == NRF24_MESSAGE_LOST) // if cant accessible that board then send the NACK to main board
        {
            tempArray[0] = 0x80;
            tempArray[1] = 0x03;
            tempArray[2] = 0xe;
            tempArray[3] = otherBoardData[2];
            tempArray[4] = tx_address[4];
            tx_address[4] = send_to_main[0];
            nrf24_tx_address(tx_address);
            txMode(tempArray);
        }
        for (unsigned int i = 0; i <= 10; i++) // self storing data//
        {
            tempArray[i] = otherBoardData[i]; // load the data information
        }
    }
}

//void clock()
//{
//    input_shift_clk_SetHigh(); // clock to the shift register
//    __delay_us(200);
//    input_shift_clk_SetLow();
//    __delay_us(200);
//}

void led_on(unsigned int led_position)
{
    led_data |= (1 << (led_position % 16));
}

void led_off(unsigned int led_position)
{
    led_data &= ~(1 << (led_position % 16));
}

void clear_data_array()
{
    for (unsigned int i = 0; i <= 20; i++)
    {
        data_array[i] = 0;
    }
}

//void left_shift_data(unsigned int send_data)
//{
//    unsigned int i;
//    // send_data = ~send_data;
//    clk_SetLow();
//    strobe_SetLow();
//
//    for (i = 0; i <= 15; i++) // loop for sending all bit serially
//    {
//        if ((send_data & 0x01) == 0x01) // sending msb first
//            data_SetHigh();
//        else
//            data_SetLow();
//        clk_SetHigh();
//        __delay_us(50);
//        clk_SetLow();
//        send_data = (send_data >> 1); // left shift data
//    }
//
//    strobe_SetHigh();
//    __delay_us(50);
//    strobe_SetLow();
//}

//before main initilization

void adc_read(void)
{
    ADCON0 = 0x0B;
    while (ADCON0 == 0x0B);
}

void blink_red_led()
{
    red_SetLow();
    __delay_ms(10);
    red_SetHigh();
    
}
void read_blue()
{
    FVRCONbits.FVREN = 1;
    while(!FVRCONbits.FVRRDY);
    ADCON0 = 0x09;
    blue_SetHigh();
    __delay_us(300);
    for(blue_count = 0; blue_count < 10;blue_count++)
    {
        adc_read();
        blue_recent[blue_count] = (unsigned int)(ADRESH << 8) | ADRESL;
    }    
    blue_SetLow();
}
 
void read_ir()
{

    ir_SetHigh();
    __delay_ms(1);
    
    for(ir_count = 0; ir_count < 10;ir_count++)
    {
         adc_read();
         ir_recent[ir_count] = (unsigned int)(ADRESH << 8) | ADRESL;
    }
    ir_SetLow();
    ADCON0 = 0x08;
    FVRCONbits.FVREN = 0;
    
} 

void read_dark_voltage()
{
    blue_init = 0;
    ir_init = 0;
    read_blue();
    read_blue();
    read_ir();
    for(blue_count = 2; blue_count < 10;blue_count++)
    {
        blue_init += blue_recent[blue_count];
    }
    blue_init = (blue_init/8)+3;
    for(ir_count = 2; ir_count < 10;ir_count++)
    {
        ir_init += ir_recent[ir_count];
    }
    ir_init = (ir_init/8)+2;    
}
void check_status()
{
    Smk_Status = 0;
    ir_status_count = 0;
    blue_status_count = 0;
    for(status_count = 2;status_count < 10; status_count++)
    {
        if(ir_recent[status_count] > ir_init)
            ir_status_count++;
        if(blue_recent[status_count] > blue_init)
            blue_status_count++; 
    }
        if((ir_status_count > 3)&&(blue_status_count > 3))
        Smk_Status = detected;  
}
void check_smk()
{
    if(UCSmoke_Detect == 0) //&& interrupt_detected != 1) //(UCSmoke_Detect==0)
    {
        read_blue();     
        read_ir();  
        check_status();

        if(Smk_Status == detected)
        {
            read_blue();     
            read_ir();  
            check_status();
            if(Smk_Status == detected)
            {
                
                UCSmoke_Detect = 1;
                red_SetLow();
            }
        }
    }   
}
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    __delay_ms(100);
    nrf24_init();

    /* Channel #2 , payload length: 4 */
    nrf24_config(2,20);

    /* Set the device addresses */
    nrf24_tx_address(tx_address);
    nrf24_rx_address(rx_address);
    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    PMD0 &= (~0x40);
    FVRCON = 0x01;
    TRISA |= 0x04; 
    ANSELA |= 0x04;
    CM1CON0= 0x00;
    CM2CON0= 0x00;
    CMOUT = 0x00;
    ADCON0 = 0x08;
    ADCON1 = 0xE3;
    //blink_red_led();
   //   read_dark_voltage();
   // while(!nrf24_dataReady());
    nrf24_powerUpRx();
    while (1)
    {
        CLRWDT();
        
        if(nrf24_dataReady())
        {
            nrf24_getData(data_array);
            if (data_array[0] == 0x80) // if the start bit start from 0x80 then passing into condition else consider as garbage data
            {

                if (data_array[0] == 0x80 && data_array[1] == 0x40 && data_array[2] == rx_data && data_array[4] == 0x00) // this condition is for polled form different board consider this board no 3 data is  0x80 0x40 0x03 0x05  (0x05)is polled board
                {
                    //                    EUSART_Write(0x85);
                    pollingRspDiffBoard(data_array[3]);
                    nrf24_powerUpRx(); // come to receiving mode
                }
                else if (data_array[0] == 0x80 && data_array[1] == 0x03 && data_array[2] == 0x02 && data_array[3] != 0x01 && data_array[4] == rx_data && data_array[5] == 0x00) // this condition is for polled form different board consider this board no 3 data is  0x80 0x40 0x03 0x05  (0x05)is polled board
                {
                    //                    EUSART_Write(0x59);
                    //                    EUSART_Write(data_array[3]);
                    countRespSwb++;
                    pollingSwbArray[countRespSwb] = data_array[3];
                    poledSwitchStatus = 0;
                }
                else if (data_array[0] == 0x80 && (data_array[1] > 1 && data_array[1] != 0x40) && (data_array[2] == 0x02 || data_array[2] == 0x01) && data_array[data_array[1] + 3] == rx_data && data_array[data_array[1] + 4] != 0 && (data_array[data_array[1] + 1] != 0x81)) // polling path through which the destination to be reached. 0x80 0x02 0x02 0x05 0x03 0x02
                {
                    pollingDiffBoard(data_array);
                    nrf24_powerUpRx();
                }
                else if (data_array[0] == 0x80 && data_array[1] == 0x01 && data_array[2] == 0x02 && data_array[4] == rx_data) // this is for start this board start polling
                {
                    prevStoredSwb = data_array[5];
                    pollinStatus = 1;  // set polling flag to high
                    initiallSwbNo = 3; // initial switch board number start form 3
                }
                else if (data_array[0] == 0x80 && data_array[data_array[1] + 1] == 0x81 && data_array[data_array[1] + 2] == 0x00) // this condition is for polled response for this board 0x80 0x03 0x02 0x05 (0x05) is nears board for three like that
                {
                    if (prevStoredSwb != 0)
                    {
                        tx_address[4] = prevStoredSwb;
                    }
                    else
                    {
                        tx_address[4] = send_to_main[0];
                    }
                    //                    tx_address[4]=0x02;
                    //                    tx_address[4] = send_to_main[0];
                    nrf24_tx_address(tx_address);
                    txMode(data_array);
                    nrf24_powerUpRx();
                }

                clear_data_array();
            }
            if (data_array[0] == 0x80 && data_array[1] == 0x02)
            {
                re_trans_count_from_prev_dev = data_array[3];
                //IRQ_Toggle(); 
                PORTCbits.RC5 = 0x00;
                __delay_ms(100);
                 PORTCbits.RC5 = 0xff;
                __delay_ms(100);
               // if(UCSmoke_Detect == 1)
                   // data_array[1] |= (1 << (detector_no-1));// send fire  status
               // else 
                    //data_array[2] |= (1 << (detector_no-1));// send normal status
                // data_array[18] = 0x61;
                 /* Automatically goes to TX mode */
                 nrf24_send(data_array);
                
               send_again:
                CLRWDT();
                data_array[3] = count;
                /* Automatically goes to TX mode */
                nrf24_send(data_array);

                /* Wait for transmission to end */
                while(nrf24_isSending());

                CLRWDT();
                /* Make analysis on last tranmission attempt */
                if(nrf24_lastMessageStatus() == NRF24_MESSAGE_LOST)
                {
                    count++;
                    if(count > 3)
                    {   
                        goto power_down;
                    }
                    goto send_again;
                }
                power_down:
               //temp = nrf24_retransmissionCount();
                nrf24_powerDown();
                if(UCSmoke_Detect == 0) //&& interrupt_detected != 1) //(UCSmoke_Detect==0)
                {
                    blink_red_led();
                }
                CLRWDT();
                SLEEP();
                CLRWDT();
                check_smk();
                SLEEP();
                CLRWDT();
                /*for(tmp_count = re_trans_count_from_prev_dev;tmp_count < 8; tmp_count++)
                {
                    __delay_ms(30);
                }*/
                nrf24_powerUpRx();  
                check_smk();
            }
        }    
    }
}
/**
 End of File
*/