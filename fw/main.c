#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

#include "board.h"
#include "adc.h"
#include "protocol.h"

uint8_t errorflag = 0;

ISR(TIMER1_COMPA_vect)
{
    adc_startConversion();
}

void cmd_write(uint8_t reg, uint8_t data)
{
    switch (reg) {
    case PROTO_LED: // Set LEDs
        PORTD = (PORTD & ~(_BV(IO_LED1)|_BV(IO_LED2)))
            | (data & 1 ? _BV(IO_LED1) : 0)
            | (data & 2 ? _BV(IO_LED2) : 0);
        break;

    case PROTO_MODE:
        adc_setMode(data);
        break;

    case PROTO_STRB_WR:
        adc_strobeWrite();
        break;

    case PROTO_STRB_RST:
        adc_strobeReset();
        break;

    case PROTO_POWER:
        adc_setPower(data);
        break;

    case PROTO_STANDBY:
        adc_setStandby(data);
        break;

    case PROTO_RANGE:
        adc_setRange(data);
        break;

    case PROTO_STARTCNV:
        adc_startConversion();
        break;

    case PROTO_REFEN:
        adc_setReference(data);
        break;

    /*case PROTO_BURST:
        burst = 1024;
        break;*/

    case PROTO_RUN:
        TCNT1 = 0;      // starta om timern för att undvika för tidig utlösning.
        TIFR1 = 0xff;   // nollställ alla interruptflaggor av samma skäl
        TIMSK1 = data ? _BV(OCIE1A) : 0;
        break;

    case PROTO_FSDIVH:
        OCR1AH = data;
        break;

    case PROTO_FSDIVL:
        OCR1AL = data;
        break;

    default:
        errorflag |= ERROR_INVALID_WRITE;
    }
}

uint8_t cmd_read(uint8_t reg)
{
    switch (reg) {
    case PROTO_VERSION:
        return PROTO_VERSION_BYTE;

    case PROTO_LED: //Get LEDs
        return (PORTD & _BV(IO_LED1) ? 0x01 : 0)
            | (PORTD & _BV(IO_LED2) ? 0x02 : 0);

    case PROTO_MODE:
        return adc_getMode();

    case PROTO_POWER:
        return adc_getPower();

    case PROTO_STANDBY:
        return adc_getStandby();

    case PROTO_RANGE:
        return adc_getRange();

    case PROTO_REFEN:
        return adc_getReference();

    case PROTO_STATUS:
        return errorflag;

    case PROTO_FSDIVH:
        return OCR1AH;

    case PROTO_FSDIVL:
        return OCR1AL;

    default:
        errorflag |= ERROR_INVALID_READ;
        return 0xaa;
    }
}

ISR(TWI_vect)
{
    static uint8_t state, reg;

    switch (TWSR) {
        // Own SLA+R has been received, ACK has been returned
        case TW_ST_SLA_ACK:
            TWDR = cmd_read(reg);
            break;

        // Data byte in TWDR has been transmitted, ACK has been received
        //case TW_ST_DATA_ACK:
            //TWDR = 0x55;
            //TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);
            //break;

        // Data byte in TWDR has been transmitted, NACK has been received.
        // i.e. this is the end of the transmission
        //case TW_ST_DATA_NACK:
            //TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);
            //break;

        // Own SLA+W has been received, ACK has been returned
        case TW_SR_SLA_ACK: // our address, ack
            state = 0;
            //PORTD |= _BV(5);
            //TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);
            //TWDR = 0x33;
            break;

        // Previously adressed with SLA+W, data has been received, ACK has been returned
        case TW_SR_DATA_ACK: // data while addressed, ack
            if (state == 0) {
                reg = TWDR;
                state++;
            } else {
                cmd_write(reg, TWDR);
            }
            //TWDR = 0xcc;
            //TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);
            break;

        // A STOP or RESTART condition has been received while addressed
        //case TW_SR_STOP:

        // Something happen! Someone set us up the bomb.
        //default:
        //    TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);
    }
    
    TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);
}

int main(void)
{
    /* Status LEDs */
    DDRD = _BV(IO_LED1)|_BV(IO_LED2);

    /* I2C to Raspberry */
    TWAR = (3 << 1);
    TWDR = 0xFF;
    TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);

    /* Init ADC */
    adc_init();

    /* Internal timekeeping, ~1000Hz */
#if 0
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS02)|_BV(CS00);
    OCR0A = 19;
    TIMSK0 = _BV(OCIE0A);
#endif

    TCCR1A = 0; //_BV(WGM01);
    TCCR1B = _BV(WGM12)|_BV(CS10); // clkio/1
    //OCR0A = 19;
    //TIMSK1 = _BV(OCIE1A);


    /* Go for it! */
    sei();

    while (1) ;
}

