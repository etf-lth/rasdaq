#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>

//#include "uart.h"
//#include "proto.h"

volatile uint8_t led_state = 0;

ISR(TIMER0_COMPA_vect)
{
    static uint8_t div = 0;

    if (++div == 50) {
        PORTD ^= _BV(1)|_BV(2);
        div = 0;
    }

    /*if (led_state == 2) {
        PORTD |= _BV(5);
        led_state = 3;
    } else if (led_state == 3) {
        PORTD &= ~_BV(5);
        led_state = 0;
    }*/
}

void cmd_write(uint8_t reg, uint8_t data)
{
    /*switch (reg) {
        default:
    }*/
}

uint8_t cmd_read(uint8_t reg)
{
    /*switch (reg) {
    case PROTO_CMD(GPIO_READ_PIN, GPIO_PORTB):
        return PINB;

    case PROTO_CMD(GPIO_READ_PIN, GPIO_PORTC):
        return PINC;

    case PROTO_CMD(GPIO_READ_PIN, GPIO_PORTD):
        return PIND;

    default:
        return 0xaa;
    }*/
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
    /* Status LED */
    DDRD = _BV(1)|_BV(2);
    PORTD = _BV(1);

    /* I2C Bus to IRMA */
    //PORTC = _BV(4)|_BV(5); // Pull-up on SDA and SCL

    /* I2C to IRMA */
    TWAR = (3 << 1);
    TWDR = 0xFF;
    TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA);

    /* UART to IRMA */
    //uart_init();

    /* Internal timekeeping, 100Hz */
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS02)|_BV(CS00);
    OCR0A = 195;
    TIMSK0 = _BV(OCIE0A);

    /* Go for it! */
    sei();

    while (1) ;
}

