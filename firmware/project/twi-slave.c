/*
 * Copyright (c) 2004, Atmel Corporation
 *
 * The sourcecode was orginally from Atmel Application Note
 * -- AVR311: TWI Slave Implementation --
 *
 * Changes to make it compile with the GNU C Compiler with the avr-libc were made by Bernhard Walle
 *
 * -------------------------------------------------------------------------------------------------
 */
#include <avr/io.h>
//#include <C:\Programme\Atmel\WinAVR\avr\include\stdbool.h>
#include <stdio.h>
#include <string.h>

//#include <avr/signal.h>
#include <avr/interrupt.h>
//#include <compat/twi.h>


#include "twi-slave.h"

static unsigned char TWI_buf[TWI_BUFFER_SIZE];     // Transceiver buffer. Set the size in the header file
static unsigned char TWI_msgSize  = 0;             // Number of bytes to be transmitted.
static unsigned char TWI_state    = TWI_NO_STATE;  // State byte. Default set to TWI_NO_STATE.

union TWI_statusReg TWI_statusReg = {0};           // TWI_statusReg is defined in TWI_Slave.h

enum {
	TWI_MODE_NONE = 0,
	TWI_MODE_RECV,
	TWI_MODE_SEND,
};
static unsigned char TWI_mode = TWI_MODE_NONE;
static unsigned char TWI_recvBytes = 0;


/**
 * Call this function to set up the TWI slave to its initial standby state.
 * Remember to enable interrupts from the main application after initializing the TWI.
 * Pass both the slave address and the requrements for triggering on a general call in the
 * same byte. Use e.g. this notation when calling this function:
 * TWI_Slave_Initialise( (TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) );
 * The TWI module is configured to NACK on any requests. Use a TWI_Start_Transceiver function to
 * start the TWI.
 * ---------------------------------------------------------------------------------------------- */
void TWI_Slave_Initialise( unsigned char TWI_ownAddress )
{
  TWAR = TWI_ownAddress;                            // Set own TWI slave address. Accept TWI General Calls.
  TWDR = 0xFF;                                      // Default content = SDA released.
  TWCR = (1 << TWEN)|                                 // Enable TWI-interface and release TWI pins.
         (0 << TWIE) | (0 << TWINT)|                      // Disable TWI Interupt.
         (0 << TWEA) | (0 << TWSTA) | (0 << TWSTO)|           // Do not ACK on any requests, yet.
         (0 << TWWC);                                 //
  TWI_mode = TWI_MODE_NONE;
}


/**
 * Call this function to test if the TWI_ISR is busy transmitting.
 * ---------------------------------------------------------------------------------------------- */
unsigned char TWI_Transceiver_Busy( void )
{
    // IF TWI interrupt is enabled then the Transceiver is busy
    return ( TWCR & (1 << TWIE) );
}


/**
 * Call this function to fetch the state information of the previous operation. The function will
 * hold execution (loop) until the TWI_ISR has completed with the previous operation. If there was
 * an error, then the function will return the TWI State code.
 * ---------------------------------------------------------------------------------------------- */
unsigned char TWI_Get_State_Info( void )
{
  while ( TWI_Transceiver_Busy() );             // Wait until TWI has completed the transmission.
  return ( TWI_state );                         // Return error state.
}


/**
 * Call this function to send a prepared message, or start the Transceiver for reception. Include
 * a pointer to the data to be sent if a SLA+W is received. The data will be copied to the TWI
 * buffer.  Also include how many bytes that should be sent. Note that unlike the similar Master
 * function, the Address byte is not included in the message buffers.
 * The function will hold execution (loop) until the TWI_ISR has completed with the previous operation,
 * then initialize the next operation and return.
 * ---------------------------------------------------------------------------------------------- */
void TWI_Start_Transceiver_With_Data( unsigned char *msg, unsigned char msgSize )
{
    unsigned char temp;

    // Wait until TWI is ready for next transmission.
    while ( TWI_Transceiver_Busy() );

    // Number of data to transmit.
    TWI_msgSize = msgSize;

    // Copy data that may be transmitted if the TWI Master requests data.
    for ( temp = 0; temp < msgSize; temp++ )
    {
        TWI_buf[ temp ] = msg[ temp ];
    }

    TWI_statusReg.all = 0;
    TWI_state         = TWI_NO_STATE;
    TWCR = (1 << TWEN)|                             // TWI Interface enabled.
           (1 << TWIE) | (1 << TWINT)|                  // Enable TWI Interupt and clear the flag.
           (1 << TWEA) | (0 << TWSTA) | (0 << TWSTO)|       // Prepare to ACK next time the Slave is addressed.
           (0 << TWWC);                             //
    TWI_mode = TWI_MODE_NONE;
    TWI_recvBytes = 0;
}


/**
 * Call this function to start the Transceiver without specifing new transmission data.
 * Usefull for restarting a transmission, or just starting the transceiver for reception.
 * The driver will reuse the data previously put in the transceiver buffers. The function will
 * hold execution (loop) until the TWI_ISR has completed with the  previous operation, then
 * initialize the next operation and return.
* ----------------------------------------------------------------------------------------------- */
void TWI_Start_Transceiver( void )
{
    // Wait until TWI is ready for next transmission.
    while ( TWI_Transceiver_Busy() );

    TWI_statusReg.all = 0;
    TWI_state         = TWI_NO_STATE;
    TWCR = (1 << TWEN)|                             // TWI Interface enabled.
           (1 << TWIE) | (1 << TWINT)|                  // Enable TWI Interupt and clear the flag.
           (1 << TWEA) | (0 << TWSTA) | (0 << TWSTO)|       // Prepare to ACK next time the Slave is addressed.
           (0 << TWWC);                             //
    TWI_mode = TWI_MODE_NONE;
    TWI_recvBytes = 0;
}


/**
 * Call this function to read out the received data from the TWI transceiver buffer. I.e. first
 * call TWI_Start_Transceiver to get the TWI Transceiver to fetch data. Then Run this function to
 * collect the data when they have arrived. Include a pointer to where to place the data and
 * the number of bytes to fetch in the function call. The function will hold execution (loop)
 * until the TWI_ISR has completed with the previous operation, before reading out the data
 * and returning. If there was an error in the previous transmission the function will return
 * the TWI State code.
 * ---------------------------------------------------------------------------------------------- */
unsigned char TWI_Get_Data_From_Transceiver( unsigned char *msg, unsigned char msgSize )
{
    unsigned char i;

    // Wait until TWI is ready for next transmission.
    while ( TWI_Transceiver_Busy() );

    // Last transmission competed successfully.
    if( TWI_statusReg.lastTransOK ) {
        // Copy data from Transceiver buffer.
        for ( i=0; i<msgSize; i++ )
        {
            msg[i] = TWI_buf[i];
        }

        // Slave Receive data has been read from buffer.
        TWI_statusReg.RxDataInBuf = FALSE;
    }

    return TWI_statusReg.lastTransOK;
}


/**
 * Try to full fill the msg buffer
 * and discard all the rest data in TWI buffer
 */
int TWI_Get_Data_With_Busy(unsigned char* msg, unsigned size) {
    unsigned i;

    // Wait until TWI is ready for next transmission.
    if ( TWI_Transceiver_Busy() ) {
        return -1;
    }

    if (TWI_recvBytes == 0) {
        return 0;
    }

    if (size > TWI_recvBytes) {
        size = TWI_recvBytes;
    }
    for (i = 0; i < size; i++) {
        msg[i] = TWI_buf[i];
    }

    // clean recv bytes counter
    TWI_recvBytes = 0;

    return size;
}

/**
 * Fill recving buffer
 */
int TWI_Recv_Data(unsigned char* msg, unsigned size) {
    int i;

    for (i = 0; i < size; i++) {
        msg[i] = TWI_buf[i];
    }
    return 0;
}

/**
 * Fill sending buffer
 */
int TWI_Send_Data(unsigned char *msg, unsigned size) {
    int i;

    // Number of data to transmit.
    TWI_msgSize = size;

    // Copy data that may be transmitted if the TWI Master requests data.
    for (i = 0; i < size; i++) {
        TWI_buf[i] = msg[i];
    }
    return 0;
}

WEAK_DECL int TWI_Recv_Callback(unsigned char* buff, int bytes) {
    return 0;
}

WEAK_DECL int TWI_Request_Callback(int dummy) {
    return 0;
}

/**
 * This function is the Interrupt Service Routine (ISR), and called when the TWI interrupt is
 * triggered; that is whenever a TWI event has occurred. This function should not be called
 * directly from the main application.
 * ---------------------------------------------------------------------------------------------- */
//SIGNAL(SIG_2WIRE_SERIAL) alter Aufruf
ISR(TWI_vect)
{
    volatile int last_twsr;
    static   int TWI_bufPtr;
    int v;

    last_twsr = TWSR;
    switch (last_twsr) {
        // Own SLA+R has been received; ACK has been returned
        case TWI_STX_ADR_ACK:
            // Set buffer pointer to first data location
            TWI_bufPtr = 0;
            TWI_mode   = TWI_MODE_SEND;
        // Data byte in TWDR has been transmitted; ACK has been received
        case TWI_STX_DATA_ACK:
            // To-DO: Why TWI_txBuf didn't store user data
            TWDR = TWI_buf[TWI_bufPtr++];

            v = 0;
            if (TWI_bufPtr < TWI_msgSize) {
                v |= (1 << TWEA);
            }

            // TWI Interface enabled
            // Enable TWI Interupt and clear the flag to send byte
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | v;
            break;

        // Data byte in TWDR has been transmitted; NACK has been received.
        // I.e. this could be the end of the transmission.
        case TWI_STX_DATA_NACK:
            // Have we transceived all expected data?
            if (TWI_bufPtr == TWI_msgSize) {
                // Set status bits to completed successfully.
                TWI_statusReg.lastTransOK = TRUE;
            } else {
                // Master has sent a NACK before all data where sent.
                // Store TWI State as errormessage.
                TWI_state = TWSR;
            }

        // Last data byte in TWDR has been transmitted (TWEA = 0); ACK has been received
        case TWI_STX_DATA_ACK_LAST_BYTE:
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA); // ack
            break;

        // General call address has been received; ACK has been returned
        case TWI_SRX_GEN_ACK:
            TWI_statusReg.genAddressCall = TRUE;
        // Own SLA+W has been received ACK has been returned
        case TWI_SRX_ADR_ACK:
            // Dont need to clear TWI_S_statusRegister.generalAddressCall due to that it is the default state.
            TWI_statusReg.RxDataInBuf = TRUE;

            // Set buffer pointer to first data location
            TWI_bufPtr = 0;
            TWI_mode   = TWI_MODE_RECV;

            // Reset the TWI Interupt to wait for a new event.

            // TWI Interface enabled
            // Enable TWI Interupt and clear the flag to send byte
            // Expect ACK on this transmission
            //
            TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
        break;

    // Previously addressed with own SLA+W; data has been received; ACK has been returned
    // Previously addressed with general call; data has been received; ACK has been returned
    case TWI_SRX_ADR_DATA_ACK:
    case TWI_SRX_GEN_DATA_ACK:
        v = 0;
        if (TWI_bufPtr < TWI_BUFFER_SIZE) {
            TWI_buf[TWI_bufPtr++] = TWDR;
            v |= (1 << TWEA);
        }

        // Set flag transmission successfull.
        TWI_statusReg.lastTransOK = TRUE;

        // Reset the TWI Interupt to wait for a new event.
        TWCR = (1 << TWEN) |                         // TWI Interface enabled
               (1 << TWIE) | (1 << TWINT) |          // Enable TWI Interupt and clear the flag to send byte
               v;                                    // Send ACK after next reception
        break;

    // A STOP condition or repeated START condition has been received while still addressed as Slave
    case TWI_SRX_STOP_RESTART:
        // Put TWI Transceiver in passive mode.
        TWCR = (1 << TWEN) |                                // Enable TWI-interface and release TWI pins
               (1 << TWIE) | (1 << TWINT) |                 // Enable Interupt
               (1 << TWEA);                                 // Acknowledge on any new requests.
        if (TWI_mode == TWI_MODE_RECV) {
            TWI_recvBytes = TWI_bufPtr;
            if (TWI_recvBytes) {
                TWI_Recv_Callback(TWI_buf, TWI_recvBytes);
                TWI_recvBytes = 0;
            }
        } else if (TWI_mode == TWI_MODE_SEND) {
            ;
        }
        TWI_mode = TWI_MODE_NONE;
        TWI_statusReg.all = 0;
        break;

    // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
    case TWI_SRX_ADR_DATA_NACK:
    // Previously addressed with general call; data has been received; NOT ACK has been returned
    case TWI_SRX_GEN_DATA_NACK:
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT) | (1 << TWEA);
        break;

    // Bus error due to an illegal START or STOP condition
    case TWI_BUS_ERROR:
    default:
        // Store TWI State as errormessage, operation also clears the Success bit.
        TWCR = (1 << TWEN) |                          // Enable TWI-interface and release TWI pins
               (1 << TWIE) | (1 << TWINT) |           // Disable Interupt
               (1 << TWEA) | (0 << TWSTA) |           // Acknowledge on any new requests.
               (0 << TWWC) |
               (1 << TWSTO);                          // Recover from bus_error status
        TWI_mode = TWI_MODE_NONE;
        TWI_statusReg.all = 0;
        TWI_state         = TWSR;
        TWI_recvBytes     = 0;
        // wait for stop condition to be exectued on bus
        // TWINT is not set after a stop condition!
        while (TWCR & _BV(TWSTO)){
            continue;
        }
    }
    /* debug only */
    led_debug(last_twsr >> 4);
}
