EESchema Schematic File Version 2  date Thu 14 Mar 2013 03:57:58 PM CET
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:raspi
LIBS:ti
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "14 mar 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L RASPI U?
U 1 1 513F6D96
P 3100 3050
F 0 "U?" H 3300 3700 60  0000 C CNN
F 1 "RASPI" H 3350 3600 60  0000 C CNN
	1    3100 3050
	1    0    0    -1  
$EndComp
$Comp
L ADS8568 U?
U 1 1 513F8222
P 7300 3900
F 0 "U?" H 7250 3800 60  0000 C CNN
F 1 "ADS8568" H 7250 4050 60  0000 C CNN
	1    7300 3900
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5141E2A2
P 9200 3500
F 0 "C?" H 9250 3600 50  0000 L CNN
F 1 "10µF" H 9250 3400 50  0000 L CNN
	1    9200 3500
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5141E2C2
P 7950 2050
F 0 "C?" H 8000 2150 50  0000 L CNN
F 1 "10µF" H 8000 1950 50  0000 L CNN
	1    7950 2050
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5141E2D2
P 6900 2100
F 0 "C?" H 6950 2200 50  0000 L CNN
F 1 "10µF" H 6950 2000 50  0000 L CNN
	1    6900 2100
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5141E2D8
P 5600 3500
F 0 "C?" H 5650 3600 50  0000 L CNN
F 1 "10µF" H 5650 3400 50  0000 L CNN
	1    5600 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 3700 9200 3700
Wire Wire Line
	8650 3400 8900 3400
Wire Wire Line
	8900 3400 8900 3300
Wire Wire Line
	8900 3300 9200 3300
Wire Wire Line
	6050 3400 5900 3400
Wire Wire Line
	5900 3400 5900 3300
Wire Wire Line
	5900 3300 5600 3300
Wire Wire Line
	6050 3700 5600 3700
Wire Wire Line
	6750 2250 6750 2550
Wire Wire Line
	6750 2250 6700 2250
Wire Wire Line
	6700 2250 6700 2100
Wire Wire Line
	7050 2550 7050 2250
Wire Wire Line
	7050 2250 7100 2250
Wire Wire Line
	7100 2250 7100 2100
Wire Wire Line
	7750 2550 7750 2050
Wire Wire Line
	8050 2550 8050 2250
Wire Wire Line
	8050 2250 8150 2250
Wire Wire Line
	8150 2250 8150 2050
$EndSCHEMATC
