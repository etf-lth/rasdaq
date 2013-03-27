EESchema Schematic File Version 2  date Wed 27 Mar 2013 08:49:56 PM CET
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
LIBS:ia0515s_
LIBS:lt3032
LIBS:bq32000
LIBS:curry2-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 6
Title ""
Date "27 mar 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L IA0515S_ U?
U 1 1 5151B8BD
P 8700 4050
F 0 "U?" H 8450 4050 60  0000 C CNN
F 1 "IA0515S_" H 8500 4150 60  0000 C CNN
	1    8700 4050
	0    1    -1   0   
$EndComp
$Comp
L LT3032 U?
U 1 1 5151B8CC
P 5800 3450
F 0 "U?" H 5800 3700 60  0000 C CNN
F 1 "LT3032" H 5800 3800 60  0000 C CNN
	1    5800 3450
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5151BA9D
P 4300 3400
F 0 "C?" H 4350 3500 50  0000 L CNN
F 1 "C" H 4350 3300 50  0000 L CNN
	1    4300 3400
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5151BAAC
P 5050 4100
F 0 "C?" H 5100 4200 50  0000 L CNN
F 1 "C" H 5100 4000 50  0000 L CNN
	1    5050 4100
	-1   0    0    1   
$EndComp
$Comp
L POT RV?
U 1 1 5151EBB9
P 6300 4350
F 0 "RV?" H 6300 4250 50  0000 C CNN
F 1 "POT" H 6300 4350 50  0000 C CNN
	1    6300 4350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EC02
P 6550 4400
F 0 "#PWR?" H 6550 4400 30  0001 C CNN
F 1 "GND" H 6550 4330 30  0001 C CNN
	1    6550 4400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EC1E
P 5700 4250
F 0 "#PWR?" H 5700 4250 30  0001 C CNN
F 1 "GND" H 5700 4180 30  0001 C CNN
	1    5700 4250
	1    0    0    -1  
$EndComp
$Comp
L POT RV?
U 1 1 5151EC34
P 4800 3300
F 0 "RV?" H 4800 3200 50  0000 C CNN
F 1 "POT" H 4800 3300 50  0000 C CNN
	1    4800 3300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EC5D
P 4800 3550
F 0 "#PWR?" H 4800 3550 30  0001 C CNN
F 1 "GND" H 4800 3480 30  0001 C CNN
	1    4800 3550
	1    0    0    -1  
$EndComp
Text HLabel 6800 3400 2    60   Input ~ 0
Enable
Wire Wire Line
	3750 3200 5050 3200
Wire Wire Line
	5050 3800 5050 3900
Wire Wire Line
	5050 4300 5050 4450
Wire Wire Line
	5050 4450 7200 4450
Wire Wire Line
	7200 4450 7200 3500
Wire Wire Line
	7200 3500 6600 3500
Wire Wire Line
	6100 4350 5250 4350
Wire Wire Line
	5250 4350 5250 3900
Wire Wire Line
	5250 3900 5050 3900
Wire Wire Line
	6300 4200 6300 4000
Wire Wire Line
	6300 4000 6600 4000
Wire Wire Line
	6600 4000 6600 3800
Wire Wire Line
	6550 4350 6550 4400
Wire Wire Line
	5700 4200 5700 4250
Wire Wire Line
	5050 3300 4950 3300
Wire Wire Line
	4800 3050 4950 3050
Wire Wire Line
	4950 3050 4950 3200
Connection ~ 4950 3200
Wire Wire Line
	6600 3600 6800 3600
Wire Wire Line
	6800 3600 6800 3400
Wire Wire Line
	6800 3400 6600 3400
Wire Wire Line
	8050 3950 8050 3200
Wire Wire Line
	8050 3200 6600 3200
Wire Wire Line
	8050 4250 7550 4250
Wire Wire Line
	7550 3700 7550 4600
Wire Wire Line
	7550 3700 6600 3700
Wire Wire Line
	5050 3700 4850 3700
Wire Wire Line
	4850 3700 4850 4600
Wire Wire Line
	4850 4600 7550 4600
Connection ~ 7550 4250
Wire Wire Line
	5800 4200 5800 4600
Connection ~ 5800 4600
$Comp
L GND #PWR?
U 1 1 5151ED14
P 7950 4150
F 0 "#PWR?" H 7950 4150 30  0001 C CNN
F 1 "GND" H 7950 4080 30  0001 C CNN
	1    7950 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 4100 7950 4100
Wire Wire Line
	7950 4100 7950 4150
$Comp
L INDUCTOR L?
U 1 1 5151ED3F
P 3450 3200
F 0 "L?" V 3400 3200 40  0000 C CNN
F 1 "INDUCTOR" V 3550 3200 40  0000 C CNN
	1    3450 3200
	0    -1   -1   0   
$EndComp
$Comp
L INDUCTOR L?
U 1 1 5151ED82
P 3450 3800
F 0 "L?" V 3400 3800 40  0000 C CNN
F 1 "INDUCTOR" V 3550 3800 40  0000 C CNN
	1    3450 3800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5050 3800 3750 3800
Wire Wire Line
	5050 3400 4500 3400
Wire Wire Line
	4100 3400 4100 3200
Connection ~ 4100 3200
$Comp
L C C?
U 1 1 5151EE2F
P 2900 3400
F 0 "C?" H 2950 3500 50  0000 L CNN
F 1 "C" H 2950 3300 50  0000 L CNN
	1    2900 3400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5151EE35
P 2900 4000
F 0 "C?" H 2950 4100 50  0000 L CNN
F 1 "C" H 2950 3900 50  0000 L CNN
	1    2900 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EE3B
P 2900 3650
F 0 "#PWR?" H 2900 3650 30  0001 C CNN
F 1 "GND" H 2900 3580 30  0001 C CNN
	1    2900 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EE41
P 2900 4300
F 0 "#PWR?" H 2900 4300 30  0001 C CNN
F 1 "GND" H 2900 4230 30  0001 C CNN
	1    2900 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3200 2900 3200
Wire Wire Line
	2900 3600 2900 3650
Wire Wire Line
	3150 3800 2900 3800
Wire Wire Line
	2900 4200 2900 4300
$Comp
L C C?
U 1 1 5151EEE9
P 7800 3400
F 0 "C?" H 7850 3500 50  0000 L CNN
F 1 "C" H 7850 3300 50  0000 L CNN
	1    7800 3400
	-1   0    0    1   
$EndComp
$Comp
L C C?
U 1 1 5151EEEF
P 7000 4800
F 0 "C?" H 7050 4900 50  0000 L CNN
F 1 "C" H 7050 4700 50  0000 L CNN
	1    7000 4800
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EEF5
P 7000 5000
F 0 "#PWR?" H 7000 5000 30  0001 C CNN
F 1 "GND" H 7000 4930 30  0001 C CNN
	1    7000 5000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EEFB
P 7800 3600
F 0 "#PWR?" H 7800 3600 30  0001 C CNN
F 1 "GND" H 7800 3530 30  0001 C CNN
	1    7800 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151EF2F
P 4550 3700
F 0 "#PWR?" H 4550 3700 30  0001 C CNN
F 1 "GND" H 4550 3630 30  0001 C CNN
	1    4550 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5050 3500 5050 3600
Wire Wire Line
	4550 3700 4550 3600
Wire Wire Line
	4550 3600 5050 3600
Text HLabel 2900 3200 0    60   Input ~ 0
+14
Text HLabel 2900 3800 0    60   Input ~ 0
-14
$Comp
L GND #PWR?
U 1 1 5151EF8D
P 7750 4500
F 0 "#PWR?" H 7750 4500 30  0001 C CNN
F 1 "GND" H 7750 4430 30  0001 C CNN
	1    7750 4500
	1    0    0    -1  
$EndComp
Text HLabel 8050 4550 0    60   Input ~ 0
+5
Wire Wire Line
	8050 4450 7750 4450
Wire Wire Line
	7750 4450 7750 4500
Text HLabel 7750 4450 0    60   Input ~ 0
GND
$Comp
L C C?
U 1 1 5151F0BF
P 3900 3400
F 0 "C?" H 3950 3500 50  0000 L CNN
F 1 "C" H 3950 3300 50  0000 L CNN
	1    3900 3400
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5151F0C5
P 3900 4000
F 0 "C?" H 3950 4100 50  0000 L CNN
F 1 "C" H 3950 3900 50  0000 L CNN
	1    3900 4000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151F0CB
P 3900 3650
F 0 "#PWR?" H 3900 3650 30  0001 C CNN
F 1 "GND" H 3900 3580 30  0001 C CNN
	1    3900 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5151F0D1
P 3900 4300
F 0 "#PWR?" H 3900 4300 30  0001 C CNN
F 1 "GND" H 3900 4230 30  0001 C CNN
	1    3900 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 3600 3900 3650
Wire Wire Line
	3900 4200 3900 4300
$EndSCHEMATC
