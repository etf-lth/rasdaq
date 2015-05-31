#include <avr/io.h>

#include "board.h"
#include "adc.h"

void adc_init(void)
{
    DDRC |= _BV(ADC_RESET) | _BV(ADC_PARSEL) | _BV(ADC_STBY) | _BV(ADC_WR);
    PORTC |= _BV(ADC_RESET); // Parallel mode, standby (in hardware mode)
    PORTC &= ~_BV(ADC_WR); // If in hardware mode, disable internal reference

    DDRD |= _BV(ADC_RANGE) | _BV(ADC_SLEEP) | _BV(PSU_PWEN) | _BV(ADC_CONVALL);
    DDRD &= ~_BV(ADC_BUSYINT);
    PORTD &= ~(_BV(ADC_SLEEP)|_BV(ADC_RANGE));

    DDRB |= _BV(ADC_HWSEL) | _BV(ADC_CONVA) | _BV(ADC_CONVB) | _BV(ADC_CONVC) | _BV(ADC_CONVD);
    PORTB &= ~_BV(ADC_HWSEL); //  Hardware mode

    // release reset
    PORTC &= ~_BV(ADC_RESET);
}

int adc_getMode(void)
{
    return (PORTB & _BV(ADC_HWSEL)) != 0;
}

void adc_setMode(int sw)
{
    if (sw) {
        PORTB |= _BV(ADC_HWSEL); // software mode
    } else {
        PORTB &= ~_BV(ADC_HWSEL); // hardware mode
    }
}

void adc_strobeWrite(void)
{
    PORTC &= ~_BV(ADC_WR);
    PORTC |= _BV(ADC_WR);
}

void adc_strobeReset(void)
{
    PORTC |= _BV(ADC_RESET);
    PORTC &= ~_BV(ADC_RESET);
}

int adc_getPower(void)
{
    return (PORTD & _BV(PSU_PWEN)) != 0;
}

void adc_setPower(int on)
{
    if (on) {
        PORTD |= _BV(PSU_PWEN);
    } else {
        PORTD &= ~_BV(PSU_PWEN);
    }
}

int adc_getStandby(void)
{
    return (PORTC & _BV(ADC_STBY)) == 0;
}

void adc_setStandby(int stby)
{
    if (stby) {
        PORTC &= ~_BV(ADC_STBY);
    } else {
        PORTC |= _BV(ADC_STBY);
    }
}

int adc_getRange(void)
{
    return (PORTD & _BV(ADC_RANGE)) != 0;
}

void adc_setRange(int range)
{
    if (range) {
        PORTD |= _BV(ADC_RANGE);
    } else {
        PORTD &= ~_BV(ADC_RANGE);
    }
}

int adc_getReference(void)
{
    return (PORTC & _BV(ADC_REFEN)) != 0;
}

void adc_setReference(int on)
{
    if (on) {
        PORTC |= _BV(ADC_REFEN);
    } else {
        PORTC &= ~_BV(ADC_REFEN);
    }
}

void adc_startConversion(void)
{
    PORTB |= _BV(ADC_CONVA) | _BV(ADC_CONVB) | _BV(ADC_CONVC) | _BV(ADC_CONVD);
    PORTB &= ~(_BV(ADC_CONVA) | _BV(ADC_CONVB) | _BV(ADC_CONVC) | _BV(ADC_CONVD));
}

