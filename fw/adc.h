#ifndef ADC_H
#define ADC_H

void adc_init(void);

int adc_getMode(void);
void adc_setMode(int sw);

void adc_strobeWrite(void);
void adc_strobeReset(void);

int adc_getPower(void);
void adc_setPower(int on);

int adc_getStandby(void);
void adc_setStandby(int on);

int adc_getRange(void);
void adc_setRange(int range);

int adc_getReference(void);
void adc_setReference(int on);

void adc_startConversion(void);

#endif
