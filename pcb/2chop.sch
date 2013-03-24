EESchema Schematic File Version 2  date Thu 14 Mar 2013 05:20:35 PM CET
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
LIBS:curry2-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "noname.sch"
Date "14 mar 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L TL082 U1
U 1 1 5141E5BD
P 5000 1800
F 0 "U1" H 4950 2000 60  0000 L CNN
F 1 "TL082" H 4950 1550 60  0000 L CNN
	1    5000 1800
	1    0    0    -1  
$EndComp
$Comp
L TL082 U1
U 2 1 5141E5EA
P 5000 3500
F 0 "U1" H 4950 3700 60  0000 L CNN
F 1 "TL082" H 4950 3250 60  0000 L CNN
	2    5000 3500
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 5141E658
P 5250 2750
F 0 "R?" V 5330 2750 50  0000 C CNN
F 1 "R" V 5250 2750 50  0000 C CNN
	1    5250 2750
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5141E665
P 3850 2700
F 0 "R?" V 3930 2700 50  0000 C CNN
F 1 "R" V 3850 2700 50  0000 C CNN
	1    3850 2700
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5141E66B
P 3850 3400
F 0 "R?" V 3930 3400 50  0000 C CNN
F 1 "R" V 3850 3400 50  0000 C CNN
	1    3850 3400
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5141E671
P 3850 3650
F 0 "R?" V 3930 3650 50  0000 C CNN
F 1 "R" V 3850 3650 50  0000 C CNN
	1    3850 3650
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 5141E677
P 3500 4000
F 0 "R?" V 3580 4000 50  0000 C CNN
F 1 "R" V 3500 4000 50  0000 C CNN
	1    3500 4000
	-1   0    0    1   
$EndComp
$Comp
L R R?
U 1 1 5141E67D
P 3100 3650
F 0 "R?" V 3180 3650 50  0000 C CNN
F 1 "R" V 3100 3650 50  0000 C CNN
	1    3100 3650
	0    1    1    0   
$EndComp
Wire Wire Line
	5500 3500 5500 2750
Wire Wire Line
	5000 2750 4100 2750
Wire Wire Line
	4100 2750 4100 2700
Wire Wire Line
	4500 3400 4100 3400
Wire Wire Line
	4300 2750 4300 3400
Connection ~ 4300 3400
Connection ~ 4300 2750
Wire Wire Line
	4100 3650 4250 3650
Wire Wire Line
	4250 3650 4250 3600
Wire Wire Line
	4250 3600 4500 3600
Wire Wire Line
	3600 3650 3350 3650
Wire Wire Line
	3500 3750 3500 3650
Connection ~ 3500 3650
Wire Wire Line
	3600 3400 2850 3400
Wire Wire Line
	2850 3400 2850 3650
$EndSCHEMATC
