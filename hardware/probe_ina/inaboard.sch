EESchema Schematic File Version 2  date Tue 02 Apr 2013 06:46:00 PM CEST
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
LIBS:ina193
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "2 apr 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L INA193 U2
U 1 1 515B05E2
P 6650 3600
F 0 "U2" H 6650 3600 60  0000 C CNN
F 1 "INA193" H 6650 3500 60  0000 C CNN
F 2 "~" H 6650 3600 60  0000 C CNN
F 3 "~" H 6650 3600 60  0000 C CNN
	1    6650 3600
	1    0    0    -1  
$EndComp
$Comp
L INA193 U1
U 1 1 515B05EF
P 5800 3600
F 0 "U1" H 5800 3600 60  0000 C CNN
F 1 "INA193" H 5800 3500 60  0000 C CNN
F 2 "~" H 5800 3600 60  0000 C CNN
F 3 "~" H 5800 3600 60  0000 C CNN
	1    5800 3600
	1    0    0    -1  
$EndComp
$Comp
L CONN_5X2 P2
U 1 1 515B05F7
P 6200 5950
F 0 "P2" H 6200 6250 60  0000 C CNN
F 1 "CONN_5X2" V 6200 5950 50  0000 C CNN
F 2 "" H 6200 5950 60  0000 C CNN
F 3 "" H 6200 5950 60  0000 C CNN
	1    6200 5950
	0    1    1    0   
$EndComp
$Comp
L GND #PWR01
U 1 1 515B066D
P 6400 6500
F 0 "#PWR01" H 6400 6500 30  0001 C CNN
F 1 "GND" H 6400 6430 30  0001 C CNN
F 2 "" H 6400 6500 60  0000 C CNN
F 3 "" H 6400 6500 60  0000 C CNN
	1    6400 6500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 515B06D2
P 6700 4550
F 0 "#PWR02" H 6700 4550 30  0001 C CNN
F 1 "GND" H 6700 4480 30  0001 C CNN
F 2 "" H 6700 4550 60  0000 C CNN
F 3 "" H 6700 4550 60  0000 C CNN
	1    6700 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 515B06D8
P 5850 4500
F 0 "#PWR03" H 5850 4500 30  0001 C CNN
F 1 "GND" H 5850 4430 30  0001 C CNN
F 2 "" H 5850 4500 60  0000 C CNN
F 3 "" H 5850 4500 60  0000 C CNN
	1    5850 4500
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 515B0719
P 6650 2650
F 0 "C4" H 6650 2750 40  0000 L CNN
F 1 "C" H 6656 2565 40  0000 L CNN
F 2 "~" H 6688 2500 30  0000 C CNN
F 3 "~" H 6650 2650 60  0000 C CNN
	1    6650 2650
	0    -1   -1   0   
$EndComp
$Comp
L C C1
U 1 1 515B0726
P 5800 2650
F 0 "C1" H 5800 2750 40  0000 L CNN
F 1 "C" H 5806 2565 40  0000 L CNN
F 2 "~" H 5838 2500 30  0000 C CNN
F 3 "~" H 5800 2650 60  0000 C CNN
	1    5800 2650
	0    -1   -1   0   
$EndComp
$Comp
L R R1
U 1 1 515B072E
P 5600 2300
F 0 "R1" V 5680 2300 40  0000 C CNN
F 1 "R" V 5607 2301 40  0000 C CNN
F 2 "~" V 5530 2300 30  0000 C CNN
F 3 "~" H 5600 2300 30  0000 C CNN
	1    5600 2300
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 515B073B
P 6000 2300
F 0 "R3" V 6080 2300 40  0000 C CNN
F 1 "R" V 6007 2301 40  0000 C CNN
F 2 "~" V 5930 2300 30  0000 C CNN
F 3 "~" H 6000 2300 30  0000 C CNN
	1    6000 2300
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 515B0741
P 6450 2300
F 0 "R4" V 6530 2300 40  0000 C CNN
F 1 "R" V 6457 2301 40  0000 C CNN
F 2 "~" V 6380 2300 30  0000 C CNN
F 3 "~" H 6450 2300 30  0000 C CNN
	1    6450 2300
	1    0    0    -1  
$EndComp
$Comp
L R R6
U 1 1 515B0747
P 6850 2300
F 0 "R6" V 6930 2300 40  0000 C CNN
F 1 "R" V 6857 2301 40  0000 C CNN
F 2 "~" V 6780 2300 30  0000 C CNN
F 3 "~" H 6850 2300 30  0000 C CNN
	1    6850 2300
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 515B074D
P 6650 1850
F 0 "R5" V 6730 1850 40  0000 C CNN
F 1 "R" V 6657 1851 40  0000 C CNN
F 2 "~" V 6580 1850 30  0000 C CNN
F 3 "~" H 6650 1850 30  0000 C CNN
	1    6650 1850
	0    1    1    0   
$EndComp
$Comp
L R R2
U 1 1 515B075D
P 5800 1850
F 0 "R2" V 5880 1850 40  0000 C CNN
F 1 "R" V 5807 1851 40  0000 C CNN
F 2 "~" V 5730 1850 30  0000 C CNN
F 3 "~" H 5800 1850 30  0000 C CNN
	1    5800 1850
	0    1    1    0   
$EndComp
$Comp
L CONN_2 P1
U 1 1 515B0912
P 5800 1200
F 0 "P1" V 5750 1200 40  0000 C CNN
F 1 "CONN_2" V 5850 1200 40  0000 C CNN
F 2 "" H 5800 1200 60  0000 C CNN
F 3 "" H 5800 1200 60  0000 C CNN
	1    5800 1200
	0    -1   -1   0   
$EndComp
$Comp
L CONN_2 P3
U 1 1 515B091F
P 6650 1200
F 0 "P3" V 6600 1200 40  0000 C CNN
F 1 "CONN_2" V 6700 1200 40  0000 C CNN
F 2 "" H 6650 1200 60  0000 C CNN
F 3 "" H 6650 1200 60  0000 C CNN
	1    6650 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6850 4350 6850 5300
Wire Wire Line
	6850 5300 6400 5300
Wire Wire Line
	6400 5300 6400 5550
Wire Wire Line
	6000 4350 6000 5200
Wire Wire Line
	6000 5200 6300 5200
Wire Wire Line
	6300 5200 6300 5550
Wire Wire Line
	6000 6350 6400 6350
Connection ~ 6100 6350
Connection ~ 6200 6350
Connection ~ 6300 6350
Wire Wire Line
	6400 6350 6400 6500
Wire Wire Line
	6100 5550 6100 5300
Wire Wire Line
	6100 5300 5700 5300
Wire Wire Line
	5700 5300 5700 4350
Wire Wire Line
	5700 4750 6550 4750
Wire Wire Line
	6550 4750 6550 4350
Connection ~ 5700 4750
Wire Wire Line
	5850 4350 5850 4500
Wire Wire Line
	6700 4350 6700 4550
Wire Wire Line
	5600 3000 5700 3000
Wire Wire Line
	5600 2550 5600 3000
Connection ~ 5600 2650
Wire Wire Line
	6000 3000 5900 3000
Wire Wire Line
	6000 2550 6000 3000
Connection ~ 6000 2650
Wire Wire Line
	6050 2050 6000 2050
Wire Wire Line
	6050 1550 6050 2050
Wire Wire Line
	5550 2050 5600 2050
Wire Wire Line
	5550 1550 5550 2050
Wire Wire Line
	6400 2050 6450 2050
Wire Wire Line
	6400 1550 6400 2050
Wire Wire Line
	6900 2050 6850 2050
Wire Wire Line
	6900 1550 6900 2050
Wire Wire Line
	6450 2550 6450 3000
Wire Wire Line
	6450 3000 6550 3000
Connection ~ 6450 2650
Wire Wire Line
	6850 3000 6750 3000
Wire Wire Line
	6850 2550 6850 3000
Connection ~ 6850 2650
Wire Wire Line
	5700 1550 5550 1550
Connection ~ 5550 1850
Wire Wire Line
	5900 1550 6050 1550
Connection ~ 6050 1850
Wire Wire Line
	6550 1550 6400 1550
Connection ~ 6400 1850
Wire Wire Line
	6750 1550 6900 1550
Connection ~ 6900 1850
$Comp
L C C3
U 1 1 515B0A4D
P 6550 4950
F 0 "C3" H 6550 5050 40  0000 L CNN
F 1 "C" H 6556 4865 40  0000 L CNN
F 2 "~" H 6588 4800 30  0000 C CNN
F 3 "~" H 6550 4950 60  0000 C CNN
	1    6550 4950
	-1   0    0    1   
$EndComp
$Comp
L C C2
U 1 1 515B0A53
P 5850 4950
F 0 "C2" H 5850 5050 40  0000 L CNN
F 1 "C" H 5856 4865 40  0000 L CNN
F 2 "~" H 5888 4800 30  0000 C CNN
F 3 "~" H 5850 4950 60  0000 C CNN
	1    5850 4950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR04
U 1 1 515B0A59
P 5850 5200
F 0 "#PWR04" H 5850 5200 30  0001 C CNN
F 1 "GND" H 5850 5130 30  0001 C CNN
F 2 "" H 5850 5200 60  0000 C CNN
F 3 "" H 5850 5200 60  0000 C CNN
	1    5850 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 515B0A5F
P 6550 5200
F 0 "#PWR05" H 6550 5200 30  0001 C CNN
F 1 "GND" H 6550 5130 30  0001 C CNN
F 2 "" H 6550 5200 60  0000 C CNN
F 3 "" H 6550 5200 60  0000 C CNN
	1    6550 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 5150 6550 5200
Wire Wire Line
	5850 5150 5850 5200
Connection ~ 6550 4750
Connection ~ 5850 4750
$EndSCHEMATC