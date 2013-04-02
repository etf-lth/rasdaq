EESchema Schematic File Version 2  date Tue 02 Apr 2013 03:24:17 PM CEST
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
Sheet 1 6
Title "radaq"
Date "2 apr 2013"
Rev "R1A"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ADS8568 U103
U 1 1 513F8222
P 6450 3650
F 0 "U103" H 6400 3550 60  0000 C CNN
F 1 "ADS8568" H 6400 3800 60  0000 C CNN
F 2 "" H 6450 3650 60  0001 C CNN
F 3 "" H 6450 3650 60  0001 C CNN
	1    6450 3650
	1    0    0    -1  
$EndComp
$Comp
L C C125
U 1 1 5141E2A2
P 8350 3250
F 0 "C125" H 8400 3350 50  0000 L CNN
F 1 "10µF" H 8400 3150 50  0000 L CNN
F 2 "" H 8350 3250 60  0001 C CNN
F 3 "" H 8350 3250 60  0001 C CNN
	1    8350 3250
	1    0    0    -1  
$EndComp
$Comp
L C C122
U 1 1 5141E2C2
P 7100 1800
F 0 "C122" H 7150 1900 50  0000 L CNN
F 1 "10µF" H 7150 1700 50  0000 L CNN
F 2 "" H 7100 1800 60  0001 C CNN
F 3 "" H 7100 1800 60  0001 C CNN
	1    7100 1800
	0    1    1    0   
$EndComp
$Comp
L C C120
U 1 1 5141E2D2
P 6050 1850
F 0 "C120" H 6100 1950 50  0000 L CNN
F 1 "10µF" H 6100 1750 50  0000 L CNN
F 2 "" H 6050 1850 60  0001 C CNN
F 3 "" H 6050 1850 60  0001 C CNN
	1    6050 1850
	0    1    1    0   
$EndComp
$Comp
L C C119
U 1 1 5141E2D8
P 4750 3250
F 0 "C119" H 4800 3350 50  0000 L CNN
F 1 "10µF" H 4800 3150 50  0000 L CNN
F 2 "" H 4750 3250 60  0001 C CNN
F 3 "" H 4750 3250 60  0001 C CNN
	1    4750 3250
	1    0    0    -1  
$EndComp
$Sheet
S 950  5225 1400 350 
U 5151B8A3
F0 "power" 60
F1 "power.sch" 60
F2 "Enable" I L 950 5325 60 
F3 "+14" I L 950 5425 60 
F4 "-14" I L 950 5525 60 
F5 "+5" I R 2350 5325 60 
F6 "GND" I R 2350 5425 60 
F7 "agnd" I R 2350 5525 60 
$EndSheet
$Comp
L INDUCTOR L103
U 1 1 5151FA61
P 3350 1500
F 0 "L103" V 3300 1500 40  0000 C CNN
F 1 "INDUCTOR" V 3450 1500 40  0000 C CNN
F 2 "" H 3350 1500 60  0001 C CNN
F 3 "" H 3350 1500 60  0001 C CNN
	1    3350 1500
	-1   0    0    1   
$EndComp
$Comp
L INDUCTOR L101
U 1 1 5151FB55
P 1450 1125
F 0 "L101" V 1400 1125 40  0000 C CNN
F 1 "INDUCTOR" V 1550 1125 40  0000 C CNN
F 2 "" H 1450 1125 60  0001 C CNN
F 3 "" H 1450 1125 60  0001 C CNN
	1    1450 1125
	0    -1   -1   0   
$EndComp
$Comp
L C C118
U 1 1 5151FC21
P 4100 2150
F 0 "C118" H 4150 2250 50  0000 L CNN
F 1 "10µF" H 4150 2050 50  0000 L CNN
F 2 "" H 4100 2150 60  0001 C CNN
F 3 "" H 4100 2150 60  0001 C CNN
	1    4100 2150
	1    0    0    -1  
$EndComp
$Comp
L C C102
U 1 1 5151FC48
P 1050 1325
F 0 "C102" H 1100 1425 50  0000 L CNN
F 1 "10µF" H 1100 1225 50  0000 L CNN
F 2 "" H 1050 1325 60  0001 C CNN
F 3 "" H 1050 1325 60  0001 C CNN
	1    1050 1325
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 51520228
P 1050 1575
F 0 "#PWR02" H 1050 1575 30  0001 C CNN
F 1 "GND" H 1050 1505 30  0001 C CNN
F 2 "" H 1050 1575 60  0001 C CNN
F 3 "" H 1050 1575 60  0001 C CNN
	1    1050 1575
	1    0    0    -1  
$EndComp
Text GLabel 4175 1950 2    60   Input ~ 0
AVDD
Text GLabel 800  1125 0    60   Input ~ 0
DVDD
$Comp
L GND #PWR03
U 1 1 515208A0
P 3400 2100
F 0 "#PWR03" H 3400 2100 30  0001 C CNN
F 1 "GND" H 3400 2030 30  0001 C CNN
F 2 "" H 3400 2100 60  0001 C CNN
F 3 "" H 3400 2100 60  0001 C CNN
	1    3400 2100
	1    0    0    -1  
$EndComp
$Comp
L C C117
U 1 1 51520C5B
P 3900 2150
F 0 "C117" H 3950 2250 50  0000 L CNN
F 1 "10µF" H 3950 2050 50  0000 L CNN
F 2 "" H 3900 2150 60  0001 C CNN
F 3 "" H 3900 2150 60  0001 C CNN
	1    3900 2150
	1    0    0    -1  
$EndComp
$Comp
L C C101
U 1 1 51520C82
P 800 1325
F 0 "C101" H 850 1425 50  0000 L CNN
F 1 "10µF" H 850 1225 50  0000 L CNN
F 2 "" H 800 1325 60  0001 C CNN
F 3 "" H 800 1325 60  0001 C CNN
	1    800  1325
	1    0    0    -1  
$EndComp
Text GLabel 1000 7000 0    60   Input ~ 0
AVDD
$Comp
L C C103
U 1 1 51520FF3
P 1100 7200
F 0 "C103" H 1000 7475 50  0000 L CNN
F 1 "0.1µF" H 1025 6925 50  0000 L CNN
F 2 "" H 1100 7200 60  0001 C CNN
F 3 "" H 1100 7200 60  0001 C CNN
	1    1100 7200
	1    0    0    -1  
$EndComp
$Comp
L C C106
U 1 1 51521026
P 1350 7200
F 0 "C106" H 1250 7475 50  0000 L CNN
F 1 "0.1µF" H 1275 6925 50  0000 L CNN
F 2 "" H 1350 7200 60  0001 C CNN
F 3 "" H 1350 7200 60  0001 C CNN
	1    1350 7200
	1    0    0    -1  
$EndComp
$Comp
L C C109
U 1 1 5152102C
P 1600 7200
F 0 "C109" H 1500 7475 50  0000 L CNN
F 1 "0.1µF" H 1525 6925 50  0000 L CNN
F 2 "" H 1600 7200 60  0001 C CNN
F 3 "" H 1600 7200 60  0001 C CNN
	1    1600 7200
	1    0    0    -1  
$EndComp
$Comp
L C C110
U 1 1 51521032
P 1850 7200
F 0 "C110" H 1750 7475 50  0000 L CNN
F 1 "0.1µF" H 1775 6925 50  0000 L CNN
F 2 "" H 1850 7200 60  0001 C CNN
F 3 "" H 1850 7200 60  0001 C CNN
	1    1850 7200
	1    0    0    -1  
$EndComp
$Comp
L C C113
U 1 1 51521038
P 2100 7200
F 0 "C113" H 2000 7475 50  0000 L CNN
F 1 "0.1µF" H 2025 6925 50  0000 L CNN
F 2 "" H 2100 7200 60  0001 C CNN
F 3 "" H 2100 7200 60  0001 C CNN
	1    2100 7200
	1    0    0    -1  
$EndComp
Text GLabel 6100 2300 1    47   Input ~ 0
AVDD
Text GLabel 6000 2300 1    47   Input ~ 0
AGND
Text GLabel 6500 2300 1    47   Input ~ 0
AVDD
Text GLabel 7000 2300 1    47   Input ~ 0
AVDD
Text GLabel 6400 2300 1    47   Input ~ 0
AGND
Text GLabel 7100 2300 1    47   Input ~ 0
AGND
Text GLabel 7800 3350 2    47   Input ~ 0
AGND
Text GLabel 5200 3350 0    47   Input ~ 0
AGND
Text GLabel 5200 4350 0    47   Input ~ 0
AGND
Text GLabel 5200 3250 0    47   Input ~ 0
AVDD
Text GLabel 5200 4250 0    47   Input ~ 0
AVDD
Text GLabel 7800 3250 2    47   Input ~ 0
AVDD
Text GLabel 850  5425 0    47   Input ~ 0
HVDD
Text GLabel 850  5525 0    47   Input ~ 0
HVSS
Text GLabel 7800 2950 2    47   Input ~ 0
HVDD
Text GLabel 5200 2950 0    47   Input ~ 0
HVSS
Text GLabel 1000 5800 0    47   Input ~ 0
HVDD
Text GLabel 1000 6700 0    47   Input ~ 0
HVSS
Text GLabel 1000 6200 0    47   Input ~ 0
AGND
$Comp
L C C104
U 1 1 515232D4
P 1100 6000
F 0 "C104" H 1150 6100 50  0000 L CNN
F 1 "4.7µF" H 1150 5900 50  0000 L CNN
F 2 "" H 1100 6000 60  0001 C CNN
F 3 "" H 1100 6000 60  0001 C CNN
	1    1100 6000
	1    0    0    -1  
$EndComp
$Comp
L C C105
U 1 1 515234F6
P 1100 6500
F 0 "C105" H 1150 6600 50  0000 L CNN
F 1 "4.7µF" H 1150 6400 50  0000 L CNN
F 2 "" H 1100 6500 60  0001 C CNN
F 3 "" H 1100 6500 60  0001 C CNN
	1    1100 6500
	1    0    0    -1  
$EndComp
$Comp
L C C107
U 1 1 51523540
P 1400 6000
F 0 "C107" H 1450 6100 50  0000 L CNN
F 1 "4.7µF" H 1450 5900 50  0000 L CNN
F 2 "" H 1400 6000 60  0001 C CNN
F 3 "" H 1400 6000 60  0001 C CNN
	1    1400 6000
	1    0    0    -1  
$EndComp
$Comp
L C C108
U 1 1 51523546
P 1400 6500
F 0 "C108" H 1450 6600 50  0000 L CNN
F 1 "4.7µF" H 1450 6400 50  0000 L CNN
F 2 "" H 1400 6500 60  0001 C CNN
F 3 "" H 1400 6500 60  0001 C CNN
	1    1400 6500
	1    0    0    -1  
$EndComp
$Comp
L C C111
U 1 1 515235AF
P 1700 6000
F 0 "C111" H 1750 6100 50  0000 L CNN
F 1 "4.7µF" H 1750 5900 50  0000 L CNN
F 2 "" H 1700 6000 60  0001 C CNN
F 3 "" H 1700 6000 60  0001 C CNN
	1    1700 6000
	1    0    0    -1  
$EndComp
$Comp
L C C112
U 1 1 515235B5
P 1700 6500
F 0 "C112" H 1750 6600 50  0000 L CNN
F 1 "4.7µF" H 1750 6400 50  0000 L CNN
F 2 "" H 1700 6500 60  0001 C CNN
F 3 "" H 1700 6500 60  0001 C CNN
	1    1700 6500
	1    0    0    -1  
$EndComp
$Comp
L C C114
U 1 1 515235BB
P 2000 6000
F 0 "C114" H 2050 6100 50  0000 L CNN
F 1 "4.7µF" H 2050 5900 50  0000 L CNN
F 2 "" H 2000 6000 60  0001 C CNN
F 3 "" H 2000 6000 60  0001 C CNN
	1    2000 6000
	1    0    0    -1  
$EndComp
$Comp
L C C115
U 1 1 515235C1
P 2000 6500
F 0 "C115" H 2050 6600 50  0000 L CNN
F 1 "4.7µF" H 2050 6400 50  0000 L CNN
F 2 "" H 2000 6500 60  0001 C CNN
F 3 "" H 2000 6500 60  0001 C CNN
	1    2000 6500
	1    0    0    -1  
$EndComp
Text GLabel 9650 3100 2    47   Input ~ 0
HVDD
$Sheet
S 8800 3000 850  500 
U 5151799D
F0 "chAop" 60
F1 "2chop.sch" 60
F2 "OUT2" I L 8800 3150 60 
F3 "+14" I R 9650 3100 60 
F4 "-14" I R 9650 3400 60 
F5 "OUT1" I L 8800 3350 60 
F6 "AGND" I R 9650 3300 60 
F7 "5v" I R 9650 3200 60 
$EndSheet
Text GLabel 9650 3300 2    47   Input ~ 0
AGND
Text GLabel 9650 3400 2    47   Input ~ 0
HVSS
Text GLabel 9650 3200 2    47   Input ~ 0
AVDD
Text GLabel 8850 1150 2    47   Input ~ 0
HVDD
$Sheet
S 8000 1050 850  500 
U 5152710B
F0 "chBop" 60
F1 "2chop.sch" 60
F2 "OUT2" I L 8000 1200 60 
F3 "+14" I R 8850 1150 60 
F4 "-14" I R 8850 1450 60 
F5 "OUT1" I L 8000 1400 60 
F6 "AGND" I R 8850 1350 60 
F7 "5v" I R 8850 1250 60 
$EndSheet
Text GLabel 8850 1350 2    47   Input ~ 0
AGND
Text GLabel 8850 1450 2    47   Input ~ 0
HVSS
Text GLabel 8850 1250 2    47   Input ~ 0
AVDD
Text GLabel 7750 550  2    47   Input ~ 0
HVDD
$Sheet
S 6900 450  850  500 
U 5152811C
F0 "chCop" 60
F1 "2chop.sch" 60
F2 "OUT2" I L 6900 600 60 
F3 "+14" I R 7750 550 60 
F4 "-14" I R 7750 850 60 
F5 "OUT1" I L 6900 800 60 
F6 "AGND" I R 7750 750 60 
F7 "5v" I R 7750 650 60 
$EndSheet
Text GLabel 7750 750  2    47   Input ~ 0
AGND
Text GLabel 7750 850  2    47   Input ~ 0
HVSS
Text GLabel 7750 650  2    47   Input ~ 0
AVDD
Text GLabel 5400 800  2    47   Input ~ 0
HVDD
$Sheet
S 4550 700  850  500 
U 51529554
F0 "chDop" 60
F1 "2chop.sch" 60
F2 "OUT2" I L 4550 850 60 
F3 "+14" I R 5400 800 60 
F4 "-14" I R 5400 1100 60 
F5 "OUT1" I L 4550 1050 60 
F6 "AGND" I R 5400 1000 60 
F7 "5v" I R 5400 900 60 
$EndSheet
Text GLabel 5400 1000 2    47   Input ~ 0
AGND
Text GLabel 5400 1100 2    47   Input ~ 0
HVSS
Text GLabel 5400 900  2    47   Input ~ 0
AVDD
$Comp
L C C121
U 1 1 5152A2A3
P 6600 1950
F 0 "C121" H 6650 2050 50  0000 L CNN
F 1 "0.47µF" H 6650 1850 50  0000 L CNN
F 2 "" H 6600 1950 60  0001 C CNN
F 3 "" H 6600 1950 60  0001 C CNN
	1    6600 1950
	1    0    0    -1  
$EndComp
Text GLabel 6700 1750 1    47   Input ~ 0
AGND
Text Label 7950 4250 0    60   ~ 0
BUSY/INT
Text Label 1750 2000 2    60   ~ 0
BUSY/INT
Text Label 5000 4050 0    60   ~ 0
RD
Text Label 3450 3100 0    60   ~ 0
RD
$Comp
L ATMEGA88-A IC101
U 1 1 51531D4C
P 10300 5350
F 0 "IC101" H 9600 6600 50  0000 L BNN
F 1 "ATMEGA88-A" H 10600 3950 50  0000 L BNN
F 2 "TQFP32" H 9750 4000 50  0001 C CNN
F 3 "" H 10300 5350 60  0001 C CNN
	1    10300 5350
	-1   0    0    1   
$EndComp
$Comp
L CRYSTAL X102
U 1 1 515330C4
P 8450 5800
F 0 "X102" H 8450 5950 60  0000 C CNN
F 1 "CRYSTAL" H 8450 5650 60  0000 C CNN
F 2 "" H 8450 5800 60  0001 C CNN
F 3 "" H 8450 5800 60  0001 C CNN
	1    8450 5800
	0    -1   -1   0   
$EndComp
$Comp
L C C123
U 1 1 51533631
P 8150 5500
F 0 "C123" H 8200 5600 50  0000 L CNN
F 1 "22pF" H 8200 5400 50  0000 L CNN
F 2 "" H 8150 5500 60  0001 C CNN
F 3 "" H 8150 5500 60  0001 C CNN
	1    8150 5500
	0    -1   -1   0   
$EndComp
$Comp
L C C124
U 1 1 515336E5
P 8150 6100
F 0 "C124" H 8200 6200 50  0000 L CNN
F 1 "22pF" H 8200 6000 50  0000 L CNN
F 2 "" H 8150 6100 60  0001 C CNN
F 3 "" H 8150 6100 60  0001 C CNN
	1    8150 6100
	0    -1   -1   0   
$EndComp
Text Label 5000 4150 0    60   ~ 0
CS
Text Label 1800 3600 0    60   ~ 0
CS
Text Label 9150 5100 0    60   ~ 0
SCL
Text Label 9150 5200 0    60   ~ 0
SDA
Text Label 3375 3500 0    60   ~ 0
SCL
Text Label 1625 3500 0    60   ~ 0
SDA
$Comp
L RASPI U101
U 1 1 513F6D96
P 2550 2300
F 0 "U101" H 2750 2950 60  0000 C CNN
F 1 "RASPI" H 2800 2850 60  0000 C CNN
F 2 "" H 2550 2300 60  0001 C CNN
F 3 "" H 2550 2300 60  0001 C CNN
	1    2550 2300
	1    0    0    -1  
$EndComp
Text Label 4950 3650 0    60   ~ 0
PAR
Text Label 4950 3750 0    60   ~ 0
STD
Text Label 4950 3850 0    60   ~ 0
RES
Text Label 4950 3950 0    60   ~ 0
WR
Text Label 7950 3650 0    60   ~ 0
HW
Text Label 7950 3750 0    60   ~ 0
CONVD
Text Label 7950 3850 0    60   ~ 0
CONVC
Text Label 7950 3950 0    60   ~ 0
CONVB
Text Label 7950 4050 0    60   ~ 0
CONVA
Text Label 7950 4150 0    60   ~ 0
SLEEP
Text Label 7950 4350 0    60   ~ 0
RANGE
Text Label 8950 4550 0    60   ~ 0
RANGE
Text Label 8850 4450 0    60   ~ 0
BUSY/INT
Text Label 8850 4350 0    60   ~ 0
SLEEP
Text Label 8900 6500 0    60   ~ 0
CONVA
Text Label 8900 6400 0    60   ~ 0
CONVB
Text Label 8900 6300 0    60   ~ 0
CONVC
Text Label 8900 6200 0    60   ~ 0
CONVD
Text Label 8950 6100 0    60   ~ 0
HW
Text Label 9100 5600 0    60   ~ 0
WR
Text Label 9100 5500 0    60   ~ 0
RES
Text Label 9100 5400 0    60   ~ 0
STD
Text Label 9100 5300 0    60   ~ 0
PAR
$Comp
L GND #PWR04
U 1 1 51539576
P 7950 6200
F 0 "#PWR04" H 7950 6200 30  0001 C CNN
F 1 "GND" H 7950 6130 30  0001 C CNN
F 2 "" H 7950 6200 60  0001 C CNN
F 3 "" H 7950 6200 60  0001 C CNN
	1    7950 6200
	1    0    0    -1  
$EndComp
Text GLabel 6650 5150 2    60   Input ~ 0
DVDD
$Comp
L GND #PWR05
U 1 1 5153A199
P 6500 5125
F 0 "#PWR05" H 6500 5125 30  0001 C CNN
F 1 "GND" H 6500 5055 30  0001 C CNN
F 2 "" H 6500 5125 60  0001 C CNN
F 3 "" H 6500 5125 60  0001 C CNN
	1    6500 5125
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P101
U 1 1 51536579
P 3000 5325
F 0 "P101" V 2950 5325 40  0000 C CNN
F 1 "CONN_2" V 3050 5325 40  0000 C CNN
F 2 "" H 3000 5325 60  0001 C CNN
F 3 "" H 3000 5325 60  0001 C CNN
	1    3000 5325
	1    0    0    -1  
$EndComp
$Comp
L AVR-ISP-6 CON101
U 1 1 5153701D
P 8750 6950
F 0 "CON101" H 8670 7190 50  0000 C CNN
F 1 "AVR-ISP-6" H 8510 6720 50  0000 L BNN
F 2 "AVR-ISP-6" V 8230 6990 50  0001 C CNN
F 3 "" H 8750 6950 60  0001 C CNN
	1    8750 6950
	1    0    0    -1  
$EndComp
Text Label 8200 7100 0    60   ~ 0
RST
Text Label 9000 5000 0    60   ~ 0
RST
$Comp
L R R103
U 1 1 51539A36
P 8400 4750
F 0 "R103" V 8480 4750 50  0000 C CNN
F 1 "R" V 8400 4750 50  0000 C CNN
F 2 "" H 8400 4750 60  0001 C CNN
F 3 "" H 8400 4750 60  0001 C CNN
	1    8400 4750
	1    0    0    -1  
$EndComp
Text GLabel 8400 4500 1    60   Input ~ 0
DVDD
Text GLabel 11125 6725 0    60   Input ~ 0
DVDD
$Comp
L GND #PWR06
U 1 1 5153A6F5
P 11150 4475
F 0 "#PWR06" H 11150 4475 30  0001 C CNN
F 1 "GND" H 11150 4405 30  0001 C CNN
F 2 "" H 11150 4475 60  0001 C CNN
F 3 "" H 11150 4475 60  0001 C CNN
	1    11150 4475
	1    0    0    -1  
$EndComp
$Comp
L CONN_4X2 P102
U 1 1 5153B250
P 8775 3900
F 0 "P102" H 8775 4150 50  0000 C CNN
F 1 "CONN_4X2" V 8775 3900 40  0000 C CNN
F 2 "" H 8775 3900 60  0001 C CNN
F 3 "" H 8775 3900 60  0001 C CNN
	1    8775 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5153C3CF
P 3300 4200
F 0 "#PWR07" H 3300 4200 30  0001 C CNN
F 1 "GND" H 3300 4130 30  0001 C CNN
F 2 "" H 3300 4200 60  0001 C CNN
F 3 "" H 3300 4200 60  0001 C CNN
	1    3300 4200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5153C605
P 1850 3800
F 0 "#PWR08" H 1850 3800 30  0001 C CNN
F 1 "GND" H 1850 3730 30  0001 C CNN
F 2 "" H 1850 3800 60  0001 C CNN
F 3 "" H 1850 3800 60  0001 C CNN
	1    1850 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5153C662
P 3300 3800
F 0 "#PWR09" H 3300 3800 30  0001 C CNN
F 1 "GND" H 3300 3730 30  0001 C CNN
F 2 "" H 3300 3800 60  0001 C CNN
F 3 "" H 3300 3800 60  0001 C CNN
	1    3300 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5153DB5A
P 3650 2525
F 0 "#PWR010" H 3650 2525 30  0001 C CNN
F 1 "GND" H 3650 2455 30  0001 C CNN
F 2 "" H 3650 2525 60  0001 C CNN
F 3 "" H 3650 2525 60  0001 C CNN
	1    3650 2525
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5153DBB7
P 3675 2850
F 0 "#PWR011" H 3675 2850 30  0001 C CNN
F 1 "GND" H 3675 2780 30  0001 C CNN
F 2 "" H 3675 2850 60  0001 C CNN
F 3 "" H 3675 2850 60  0001 C CNN
	1    3675 2850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5153DEDE
P 1850 3150
F 0 "#PWR012" H 1850 3150 30  0001 C CNN
F 1 "GND" H 1850 3080 30  0001 C CNN
F 2 "" H 1850 3150 60  0001 C CNN
F 3 "" H 1850 3150 60  0001 C CNN
	1    1850 3150
	1    0    0    -1  
$EndComp
$Comp
L R R101
U 1 1 5153ED42
P 1600 3300
F 0 "R101" V 1680 3300 50  0000 C CNN
F 1 "R" V 1600 3300 50  0000 C CNN
F 2 "" H 1600 3300 60  0001 C CNN
F 3 "" H 1600 3300 60  0001 C CNN
	1    1600 3300
	0    -1   -1   0   
$EndComp
$Comp
L R R102
U 1 1 5153F20A
P 3550 3300
F 0 "R102" V 3630 3300 50  0000 C CNN
F 1 "4k7" V 3550 3300 50  0000 C CNN
F 2 "" H 3550 3300 60  0001 C CNN
F 3 "" H 3550 3300 60  0001 C CNN
	1    3550 3300
	0    -1   -1   0   
$EndComp
Text GLabel 1250 3300 0    60   Input ~ 0
DVDD
Text GLabel 3875 3300 2    60   Input ~ 0
DVDD
$Comp
L INDUCTOR L102
U 1 1 5153FB9C
P 3200 1500
F 0 "L102" V 3150 1500 40  0000 C CNN
F 1 "INDUCTOR" V 3300 1500 40  0000 C CNN
F 2 "" H 3200 1500 60  0001 C CNN
F 3 "" H 3200 1500 60  0001 C CNN
	1    3200 1500
	-1   0    0    1   
$EndComp
$Comp
L INDUCTOR L105
U 1 1 5153FF1C
P 3750 1500
F 0 "L105" V 3700 1500 40  0000 C CNN
F 1 "INDUCTOR" V 3850 1500 40  0000 C CNN
F 2 "" H 3750 1500 60  0001 C CNN
F 3 "" H 3750 1500 60  0001 C CNN
	1    3750 1500
	-1   0    0    1   
$EndComp
$Comp
L INDUCTOR L104
U 1 1 5153FF22
P 3600 1500
F 0 "L104" V 3550 1500 40  0000 C CNN
F 1 "INDUCTOR" V 3700 1500 40  0000 C CNN
F 2 "" H 3600 1500 60  0001 C CNN
F 3 "" H 3600 1500 60  0001 C CNN
	1    3600 1500
	-1   0    0    1   
$EndComp
$Comp
L BQ32000 U102
U 1 1 5154110C
P 3850 6875
F 0 "U102" H 3825 6925 60  0000 C CNN
F 1 "BQ32000" H 3850 6800 60  0000 C CNN
F 2 "" H 3850 6875 60  0001 C CNN
F 3 "" H 3850 6875 60  0001 C CNN
	1    3850 6875
	1    0    0    -1  
$EndComp
$Comp
L C C116
U 1 1 51541846
P 3150 6250
F 0 "C116" H 3200 6350 50  0000 L CNN
F 1 "1µF" H 3200 6150 50  0000 L CNN
F 2 "" H 3150 6250 60  0001 C CNN
F 3 "" H 3150 6250 60  0001 C CNN
	1    3150 6250
	1    0    0    -1  
$EndComp
Text GLabel 3050 6050 0    60   Input ~ 0
DVDD
$Comp
L GND #PWR013
U 1 1 5154260A
P 3150 6500
F 0 "#PWR013" H 3150 6500 30  0001 C CNN
F 1 "GND" H 3150 6430 30  0001 C CNN
F 2 "" H 3150 6500 60  0001 C CNN
F 3 "" H 3150 6500 60  0001 C CNN
	1    3150 6500
	1    0    0    -1  
$EndComp
Text Label 2775 6750 0    60   ~ 0
SDA
Text Label 2775 6875 0    60   ~ 0
SCL
$Comp
L GND #PWR014
U 1 1 51543461
P 3850 7650
F 0 "#PWR014" H 3850 7650 30  0001 C CNN
F 1 "GND" H 3850 7580 30  0001 C CNN
F 2 "" H 3850 7650 60  0001 C CNN
F 3 "" H 3850 7650 60  0001 C CNN
	1    3850 7650
	1    0    0    -1  
$EndComp
$Comp
L BATTERY BT101
U 1 1 515441EF
P 4250 6050
F 0 "BT101" H 4250 6250 50  0000 C CNN
F 1 "BATTERY" H 4250 5860 50  0000 C CNN
F 2 "" H 4250 6050 60  0001 C CNN
F 3 "" H 4250 6050 60  0001 C CNN
	1    4250 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 515445A2
P 4550 6175
F 0 "#PWR015" H 4550 6175 30  0001 C CNN
F 1 "GND" H 4550 6105 30  0001 C CNN
F 2 "" H 4550 6175 60  0001 C CNN
F 3 "" H 4550 6175 60  0001 C CNN
	1    4550 6175
	1    0    0    -1  
$EndComp
$Comp
L LED D101
U 1 1 5154BDA4
P 9025 4650
F 0 "D101" H 9025 4750 50  0000 C CNN
F 1 "LED" H 9025 4550 50  0000 C CNN
F 2 "" H 9025 4650 60  0001 C CNN
F 3 "" H 9025 4650 60  0001 C CNN
	1    9025 4650
	-1   0    0    1   
$EndComp
$Comp
L LED D102
U 1 1 5154BE0A
P 9025 4750
F 0 "D102" H 9025 4850 50  0000 C CNN
F 1 "LED" H 9025 4650 50  0000 C CNN
F 2 "" H 9025 4750 60  0001 C CNN
F 3 "" H 9025 4750 60  0001 C CNN
	1    9025 4750
	-1   0    0    1   
$EndComp
$Comp
L R R105
U 1 1 5154C021
P 8750 4900
F 0 "R105" V 8830 4900 50  0000 C CNN
F 1 "R" V 8750 4900 50  0000 C CNN
F 2 "" H 8750 4900 60  0001 C CNN
F 3 "" H 8750 4900 60  0001 C CNN
	1    8750 4900
	-1   0    0    1   
$EndComp
$Comp
L R R104
U 1 1 5154C07E
P 8600 5000
F 0 "R104" V 8680 5000 50  0000 C CNN
F 1 "R" V 8600 5000 50  0000 C CNN
F 2 "" H 8600 5000 60  0001 C CNN
F 3 "" H 8600 5000 60  0001 C CNN
	1    8600 5000
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR016
U 1 1 5154C56D
P 8675 5275
F 0 "#PWR016" H 8675 5275 30  0001 C CNN
F 1 "GND" H 8675 5205 30  0001 C CNN
F 2 "" H 8675 5275 60  0001 C CNN
F 3 "" H 8675 5275 60  0001 C CNN
	1    8675 5275
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5154CC74
P 9250 7125
F 0 "#PWR017" H 9250 7125 30  0001 C CNN
F 1 "GND" H 9250 7055 30  0001 C CNN
F 2 "" H 9250 7125 60  0001 C CNN
F 3 "" H 9250 7125 60  0001 C CNN
	1    9250 7125
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 5154D766
P 5625 7275
F 0 "#PWR018" H 5625 7275 30  0001 C CNN
F 1 "GND" H 5625 7205 30  0001 C CNN
F 2 "" H 5625 7275 60  0001 C CNN
F 3 "" H 5625 7275 60  0001 C CNN
	1    5625 7275
	1    0    0    -1  
$EndComp
$Comp
L SPST SW102
U 1 1 5154DB4F
P 6125 6875
F 0 "SW102" H 6125 6975 70  0000 C CNN
F 1 "SPST" H 6125 6775 70  0000 C CNN
F 2 "" H 6125 6875 60  0001 C CNN
F 3 "" H 6125 6875 60  0001 C CNN
	1    6125 6875
	1    0    0    -1  
$EndComp
$Comp
L SPST SW101
U 1 1 5154DBB5
P 6125 7200
F 0 "SW101" H 6125 7300 70  0000 C CNN
F 1 "SPST" H 6125 7100 70  0000 C CNN
F 2 "" H 6125 7200 60  0001 C CNN
F 3 "" H 6125 7200 60  0001 C CNN
	1    6125 7200
	1    0    0    -1  
$EndComp
Text GLabel 7800 4450 2    47   Input ~ 0
DB0
Text GLabel 7300 4850 3    47   Input ~ 0
DB1
Text GLabel 7200 4850 3    47   Input ~ 0
DB2
Text GLabel 7100 4850 3    47   Input ~ 0
DB3
Text GLabel 7000 4850 3    47   Input ~ 0
DB4
Text GLabel 6900 4850 3    47   Input ~ 0
DB5
Text GLabel 6800 4850 3    47   Input ~ 0
DB6
Text GLabel 6700 4850 3    47   Input ~ 0
DB7
Text GLabel 6400 4850 3    47   Input ~ 0
DB8
Text GLabel 6300 4850 3    47   Input ~ 0
DB9
Text GLabel 6200 4850 3    47   Input ~ 0
DB10
Text GLabel 6100 4850 3    47   Input ~ 0
DB11
Text GLabel 6000 4850 3    47   Input ~ 0
DB12
Text GLabel 5900 5050 3    47   Input ~ 0
DB13
Text GLabel 5800 5050 3    47   Input ~ 0
DB14
Text GLabel 5100 4450 0    47   Input ~ 0
DB15
Text GLabel 1750 2100 0    47   Input ~ 0
DB0
Text GLabel 1750 2200 0    47   Input ~ 0
DB1
Text GLabel 3200 2200 2    47   Input ~ 0
DB2
Text GLabel 1950 2300 0    47   Input ~ 0
DB3
Text GLabel 3200 2300 2    47   Input ~ 0
DB4
Text GLabel 1950 2400 0    47   Input ~ 0
DB5
Text GLabel 3200 2400 2    47   Input ~ 0
DB6
Text GLabel 1950 2500 0    47   Input ~ 0
DB7
Text GLabel 1950 2600 0    47   Input ~ 0
DB8
Text GLabel 3200 2600 2    47   Input ~ 0
DB9
Text GLabel 3200 2700 2    47   Input ~ 0
DB10
Text GLabel 1950 2800 0    47   Input ~ 0
DB11
Text GLabel 1850 2900 0    47   Input ~ 0
DB12
Text GLabel 3250 2900 2    47   Input ~ 0
DB13
Text GLabel 1850 3000 0    47   Input ~ 0
DB14
Text GLabel 3250 3000 2    47   Input ~ 0
DB15
Text GLabel 2450 5525 2    47   Input ~ 0
AGND
$Comp
L R R1
U 1 1 51585EB3
P 2400 6450
F 0 "R1" H 2275 6400 50  0000 C CNN
F 1 "0R?" H 2250 6500 50  0000 C CNN
F 2 "" H 2400 6450 60  0001 C CNN
F 3 "" H 2400 6450 60  0001 C CNN
	1    2400 6450
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR019
U 1 1 51586021
P 2400 6750
F 0 "#PWR019" H 2400 6750 30  0001 C CNN
F 1 "GND" H 2400 6680 30  0001 C CNN
F 2 "" H 2400 6750 60  0001 C CNN
F 3 "" H 2400 6750 60  0001 C CNN
	1    2400 6750
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR020
U 1 1 5158CA84
P 3300 1950
F 0 "#PWR020" H 3300 2040 20  0001 C CNN
F 1 "+5V" H 3300 2040 30  0000 C CNN
F 2 "" H 3300 1950 60  0000 C CNN
F 3 "" H 3300 1950 60  0000 C CNN
	1    3300 1950
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR021
U 1 1 5158CF7D
P 2425 5325
F 0 "#PWR021" H 2425 5415 20  0001 C CNN
F 1 "+5V" H 2425 5415 30  0000 C CNN
F 2 "" H 2425 5325 60  0000 C CNN
F 3 "" H 2425 5325 60  0000 C CNN
	1    2425 5325
	1    0    0    -1  
$EndComp
$Comp
L CRYSTAL X101
U 1 1 5154493A
P 4900 6850
F 0 "X101" V 5075 6700 60  0000 C CNN
F 1 "32K768" V 4725 6650 60  0000 C CNN
F 2 "" H 4900 6850 60  0001 C CNN
F 3 "" H 4900 6850 60  0001 C CNN
	1    4900 6850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7800 3450 8350 3450
Wire Wire Line
	7800 3150 8250 3150
Wire Wire Line
	8250 3150 8250 3050
Wire Wire Line
	8250 3050 8350 3050
Wire Wire Line
	5200 3150 4850 3150
Wire Wire Line
	4850 3150 4850 3050
Wire Wire Line
	4850 3050 4750 3050
Wire Wire Line
	5200 3450 4750 3450
Wire Wire Line
	5900 2000 5900 2300
Wire Wire Line
	5900 2000 5850 2000
Wire Wire Line
	5850 2000 5850 1850
Wire Wire Line
	6200 2300 6200 2000
Wire Wire Line
	6200 2000 6250 2000
Wire Wire Line
	6250 2000 6250 1850
Wire Wire Line
	6900 2300 6900 1800
Wire Wire Line
	7200 2300 7200 2000
Wire Wire Line
	7200 2000 7300 2000
Wire Wire Line
	7300 2000 7300 1800
Wire Wire Line
	3200 1950 3300 1950
Wire Wire Line
	3300 1950 3350 1950
Wire Wire Line
	3200 1900 3200 1950
Wire Wire Line
	3200 1950 3200 2000
Connection ~ 3200 1950
Wire Wire Line
	3675 1950 3900 1950
Wire Wire Line
	3900 1950 4100 1950
Wire Wire Line
	4100 1950 4175 1950
Wire Wire Line
	1100 1900 1800 1900
Wire Wire Line
	1800 1900 1950 1900
Wire Wire Line
	800  1125 1050 1125
Wire Wire Line
	1050 1125 1150 1125
Wire Wire Line
	3200 2100 3400 2100
Connection ~ 1050 1125
Wire Wire Line
	800  1525 1050 1525
Wire Wire Line
	3900 2350 4100 2350
Wire Wire Line
	1000 7000 1100 7000
Wire Wire Line
	1100 7000 1350 7000
Wire Wire Line
	1350 7000 1600 7000
Wire Wire Line
	1600 7000 1850 7000
Wire Wire Line
	1850 7000 2100 7000
Connection ~ 1850 7000
Connection ~ 1600 7000
Connection ~ 1350 7000
Wire Wire Line
	1000 7400 1100 7400
Wire Wire Line
	1100 7400 1350 7400
Wire Wire Line
	1350 7400 1600 7400
Wire Wire Line
	1600 7400 1850 7400
Wire Wire Line
	1850 7400 2100 7400
Connection ~ 1600 7400
Wire Wire Line
	7800 3050 8175 3050
Wire Wire Line
	8175 3050 8175 2975
Wire Wire Line
	8175 2975 8650 2975
Wire Wire Line
	7800 3550 8650 3550
Wire Wire Line
	1000 5800 1100 5800
Wire Wire Line
	1100 5800 1400 5800
Wire Wire Line
	1400 5800 1700 5800
Wire Wire Line
	1700 5800 2000 5800
Wire Wire Line
	1000 6200 1100 6200
Wire Wire Line
	1100 6200 1400 6200
Wire Wire Line
	1400 6200 1700 6200
Wire Wire Line
	1700 6200 2000 6200
Wire Wire Line
	2000 6200 2400 6200
Wire Wire Line
	1700 6200 1700 6300
Wire Wire Line
	1400 6200 1400 6300
Wire Wire Line
	1100 6300 1100 6200
Wire Wire Line
	1000 6700 1100 6700
Wire Wire Line
	1100 6700 1400 6700
Wire Wire Line
	1400 6700 1700 6700
Wire Wire Line
	1700 6700 2000 6700
Wire Wire Line
	8650 2975 8650 3150
Wire Wire Line
	8650 3150 8800 3150
Wire Wire Line
	8650 3550 8650 3350
Wire Wire Line
	8650 3350 8800 3350
Wire Wire Line
	6800 1200 8000 1200
Wire Wire Line
	7350 1400 8000 1400
Wire Wire Line
	7300 2300 7350 2300
Wire Wire Line
	7350 2300 7350 1400
Wire Wire Line
	6800 1200 6800 2300
Wire Wire Line
	5800 800  6900 800 
Wire Wire Line
	5800 800  5800 2300
Wire Wire Line
	6300 2300 6300 600 
Connection ~ 6300 600 
Wire Wire Line
	6300 600  6900 600 
Wire Wire Line
	5200 3550 4625 3550
Wire Wire Line
	4625 3550 4625 1600
Wire Wire Line
	4625 1600 4150 1600
Wire Wire Line
	4150 1600 4150 1050
Wire Wire Line
	4150 1050 4550 1050
Wire Wire Line
	4900 3050 5200 3050
Wire Wire Line
	4900 1500 4900 3050
Wire Wire Line
	4900 1500 4250 1500
Wire Wire Line
	4250 1500 4250 850 
Wire Wire Line
	4250 850  4550 850 
Wire Wire Line
	6600 2300 6600 2150
Wire Wire Line
	6700 2300 6700 1750
Wire Wire Line
	6700 1750 6600 1750
Wire Wire Line
	1750 2000 1950 2000
Wire Wire Line
	1100 2700 1950 2700
Wire Wire Line
	1950 3100 1850 3100
Wire Wire Line
	3200 3100 3450 3100
Wire Wire Line
	7800 4250 7950 4250
Wire Wire Line
	5200 4050 5000 4050
Wire Wire Line
	9300 5750 8700 5750
Wire Wire Line
	8700 5750 8700 5500
Wire Wire Line
	8700 5500 8450 5500
Wire Wire Line
	8450 5500 8350 5500
Wire Wire Line
	9300 5850 8700 5850
Wire Wire Line
	8700 5850 8700 6100
Wire Wire Line
	8700 6100 8450 6100
Wire Wire Line
	8450 6100 8350 6100
Connection ~ 8450 6100
Connection ~ 8450 5500
Wire Wire Line
	7950 5500 7950 6100
Wire Wire Line
	7950 6100 7950 6200
Wire Wire Line
	5200 4150 5000 4150
Wire Wire Line
	1950 3600 1800 3600
Wire Wire Line
	9300 5100 9150 5100
Wire Wire Line
	9300 5200 9150 5200
Wire Wire Line
	3200 3500 3300 3500
Wire Wire Line
	3300 3500 3375 3500
Wire Wire Line
	1625 3500 1850 3500
Wire Wire Line
	1850 3500 1950 3500
Wire Wire Line
	5200 3650 4950 3650
Wire Wire Line
	5200 3750 4950 3750
Wire Wire Line
	5200 3850 4950 3850
Wire Wire Line
	5200 3950 4950 3950
Wire Wire Line
	7800 3650 7950 3650
Wire Wire Line
	7800 3750 8375 3750
Wire Wire Line
	7800 3850 8375 3850
Wire Wire Line
	7800 3950 8375 3950
Wire Wire Line
	7800 4050 8375 4050
Wire Wire Line
	7800 4150 7950 4150
Wire Wire Line
	7800 4350 7950 4350
Wire Wire Line
	8825 6200 9300 6200
Wire Wire Line
	9300 6200 9300 6150
Wire Wire Line
	8900 6300 9300 6300
Wire Wire Line
	9300 6300 9300 6250
Wire Wire Line
	8900 6400 9300 6400
Wire Wire Line
	9300 6400 9300 6350
Wire Wire Line
	8900 6500 9300 6500
Wire Wire Line
	9300 6500 9300 6450
Wire Wire Line
	8800 6100 9300 6100
Wire Wire Line
	9300 6100 9300 6050
Wire Wire Line
	9100 5300 9300 5300
Wire Wire Line
	9100 5400 9300 5400
Wire Wire Line
	9100 5500 9300 5500
Wire Wire Line
	9100 5600 9300 5600
Connection ~ 7950 6100
Wire Wire Line
	6600 4850 6600 5150
Wire Wire Line
	6600 5150 6650 5150
Wire Wire Line
	9300 4350 8850 4350
Wire Wire Line
	9300 4450 8850 4450
Wire Wire Line
	9300 4550 8950 4550
Wire Wire Line
	2650 5325 2650 5225
Wire Wire Line
	2350 5325 2425 5325
Wire Wire Line
	2425 5325 2650 5325
Wire Wire Line
	2350 5425 2650 5425
Connection ~ 11200 6450
Wire Wire Line
	8325 6150 8750 6150
Wire Wire Line
	8750 6150 8750 5950
Wire Wire Line
	8750 5950 9300 5950
Wire Wire Line
	8800 6100 8800 6200
Wire Wire Line
	8800 6200 8375 6200
Wire Wire Line
	8375 6200 8375 6800
Wire Wire Line
	9250 7000 9250 6675
Wire Wire Line
	9250 6675 8825 6675
Wire Wire Line
	8825 6675 8825 6200
Wire Wire Line
	8400 5000 9300 5000
Wire Wire Line
	11200 4425 11150 4425
Wire Wire Line
	11150 4425 11150 4475
Wire Wire Line
	9175 4150 9300 4150
Wire Wire Line
	1100 2700 1100 1900
Wire Wire Line
	1800 1900 1800 1125
Wire Wire Line
	1800 1125 1750 1125
Connection ~ 1800 1900
Wire Wire Line
	3200 2500 3650 2500
Wire Wire Line
	3650 2500 3650 2525
Wire Wire Line
	3200 2800 3675 2800
Wire Wire Line
	3675 2800 3675 2850
Wire Wire Line
	1850 3100 1850 3150
Wire Wire Line
	3350 1950 3350 1800
Wire Wire Line
	3350 1800 3200 1800
Wire Wire Line
	3200 1200 3350 1200
Wire Wire Line
	3350 1200 3600 1200
Wire Wire Line
	3600 1200 3750 1200
Wire Wire Line
	3600 1800 3675 1800
Wire Wire Line
	3675 1800 3750 1800
Wire Wire Line
	3675 1800 3675 1950
Connection ~ 3900 1950
Connection ~ 3675 1800
Wire Wire Line
	3050 6050 3150 6050
Wire Wire Line
	3150 6050 3750 6050
Wire Wire Line
	3750 6050 3750 6175
Wire Wire Line
	3150 6450 3150 6500
Wire Wire Line
	3050 6775 2775 6775
Wire Wire Line
	2775 6775 2775 6750
Wire Wire Line
	3050 6875 2775 6875
Wire Wire Line
	3850 7575 3850 7650
Wire Wire Line
	3950 6175 3950 6050
Wire Wire Line
	4550 6050 4550 6175
Wire Wire Line
	4650 6775 4650 6550
Wire Wire Line
	4650 6550 4900 6550
Wire Wire Line
	4650 6875 4650 7150
Wire Wire Line
	4650 7150 4900 7150
Wire Wire Line
	8825 4650 8750 4650
Wire Wire Line
	8825 4750 8600 4750
Wire Wire Line
	8600 5250 8675 5250
Wire Wire Line
	8675 5250 8750 5250
Wire Wire Line
	8750 5250 8750 5150
Wire Wire Line
	8675 5250 8675 5275
Connection ~ 8675 5250
Wire Wire Line
	9300 4650 9225 4650
Wire Wire Line
	9300 4750 9225 4750
Wire Wire Line
	6625 6750 8225 6750
Wire Wire Line
	8225 6750 8325 6750
Wire Wire Line
	6625 6750 6625 6875
Wire Wire Line
	8375 6800 8275 6800
Wire Wire Line
	8275 6800 6850 6800
Wire Wire Line
	6850 6800 6850 7200
Wire Wire Line
	6850 7200 6625 7200
Connection ~ 8275 6800
Connection ~ 11200 6350
Wire Wire Line
	1950 2100 1750 2100
Wire Wire Line
	1950 2200 1750 2200
Wire Wire Line
	5200 4450 5100 4450
Wire Wire Line
	5800 4850 5800 5050
Wire Wire Line
	5900 4850 5900 5050
Wire Wire Line
	1950 3000 1850 3000
Wire Wire Line
	1950 2900 1850 2900
Wire Wire Line
	3250 3000 3200 3000
Wire Wire Line
	3250 2900 3200 2900
Wire Wire Line
	2400 6700 2400 6750
Connection ~ 3300 1950
Connection ~ 2425 5325
Connection ~ 3200 1800
Connection ~ 3350 1800
Connection ~ 3600 1800
Connection ~ 3750 1800
Connection ~ 3750 1200
Connection ~ 3600 1200
Connection ~ 3350 1200
Connection ~ 3200 1200
Connection ~ 1150 1125
Connection ~ 1750 1125
Connection ~ 800  1125
Connection ~ 800  1525
Connection ~ 1050 1525
Wire Wire Line
	1050 1525 1050 1575
Wire Wire Line
	4100 2350 4100 2400
Connection ~ 4100 1950
Connection ~ 3900 2350
Connection ~ 2000 6700
Connection ~ 1700 6700
Connection ~ 1400 6700
Connection ~ 1100 6700
Connection ~ 1100 6200
Connection ~ 1400 6200
Connection ~ 1700 6200
Connection ~ 2000 6200
Connection ~ 1400 5800
Connection ~ 1700 5800
Connection ~ 1100 5800
Connection ~ 1100 7400
Connection ~ 1350 7400
Connection ~ 1850 7400
Connection ~ 1950 2800
Connection ~ 1950 2700
Connection ~ 1950 2600
Connection ~ 8350 3050
Connection ~ 8350 3450
Connection ~ 5850 1850
Connection ~ 6250 1850
Connection ~ 4750 3050
Connection ~ 4750 3450
Connection ~ 8400 5000
Connection ~ 8600 4750
Connection ~ 8750 5150
Connection ~ 8600 5250
Connection ~ 8750 4650
Connection ~ 8825 4650
Connection ~ 8825 4750
Connection ~ 8400 4500
Text GLabel 1000 7400 0    60   Input ~ 0
AGND
Text GLabel 4100 2400 3    60   Input ~ 0
AGND
Wire Wire Line
	9250 7000 9000 7000
Wire Wire Line
	9000 7000 8950 6950
Wire Wire Line
	8950 6950 8875 6950
Wire Wire Line
	8875 7050 8950 7050
Wire Wire Line
	8950 7050 9000 7100
Wire Wire Line
	9000 7100 9250 7100
Wire Wire Line
	9250 7100 9250 7125
Wire Wire Line
	8625 7050 8550 7050
Wire Wire Line
	8550 7050 8500 7100
Wire Wire Line
	8500 7100 8200 7100
Wire Wire Line
	11200 6725 11125 6725
Wire Wire Line
	11200 6150 11200 6350
Wire Wire Line
	11200 6350 11200 6450
Wire Wire Line
	11200 6450 11200 6725
Text GLabel 9325 6900 2    60   Input ~ 0
DVDD
Wire Wire Line
	9325 6900 9000 6900
Wire Wire Line
	9000 6900 8950 6850
Wire Wire Line
	8950 6850 8875 6850
Wire Wire Line
	8625 6950 8550 6950
Wire Wire Line
	8550 6950 8500 7000
Wire Wire Line
	8500 7000 8225 7000
Wire Wire Line
	8275 6800 8275 6900
Wire Wire Line
	8275 6900 8500 6900
Wire Wire Line
	8500 6900 8550 6850
Wire Wire Line
	8550 6850 8625 6850
$Comp
L GND #PWR?
U 1 1 515B1A18
P 5625 6950
F 0 "#PWR?" H 5625 6950 30  0001 C CNN
F 1 "GND" H 5625 6880 30  0001 C CNN
F 2 "" H 5625 6950 60  0001 C CNN
F 3 "" H 5625 6950 60  0001 C CNN
	1    5625 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8325 6750 8325 6150
Wire Wire Line
	8225 7000 8225 6750
Connection ~ 8225 6750
Wire Wire Line
	5625 6950 5625 6875
Wire Wire Line
	5625 7200 5625 7275
Connection ~ 1100 7000
Connection ~ 3150 6050
Wire Wire Line
	2000 6300 2000 6200
Wire Wire Line
	6500 4850 6500 5125
Text Label 9050 4850 0    60   ~ 0
PWEN
Wire Wire Line
	9050 4850 9300 4850
Text Label 650  5325 0    60   ~ 0
PWEN
Wire Wire Line
	650  5325 950  5325
Wire Wire Line
	11200 4150 11200 4250
Wire Wire Line
	11200 4250 11200 4350
Wire Wire Line
	11200 4350 11200 4425
Connection ~ 11200 4350
Connection ~ 11200 4250
Wire Wire Line
	9175 3750 9175 3850
Wire Wire Line
	9175 3850 9175 3950
Wire Wire Line
	9175 3950 9175 4050
Wire Wire Line
	9175 4050 9175 4150
Connection ~ 9175 4050
Connection ~ 9175 3950
Connection ~ 9175 3850
Wire Wire Line
	2350 5525 2450 5525
Wire Wire Line
	850  5425 950  5425
Wire Wire Line
	850  5525 950  5525
Wire Wire Line
	3875 3300 3800 3300
Wire Wire Line
	3300 3300 3300 3500
Connection ~ 3300 3500
Wire Wire Line
	3300 3800 3300 3700
Wire Wire Line
	3300 3700 3200 3700
Wire Wire Line
	1950 3700 1850 3700
Wire Wire Line
	1850 3700 1850 3800
$Comp
L GND #PWR?
U 1 1 515B25BC
P 1850 4200
F 0 "#PWR?" H 1850 4200 30  0001 C CNN
F 1 "GND" H 1850 4130 30  0001 C CNN
F 2 "" H 1850 4200 60  0001 C CNN
F 3 "" H 1850 4200 60  0001 C CNN
	1    1850 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 4000 1850 4100
Wire Wire Line
	1850 4100 1850 4200
Wire Wire Line
	1850 4000 1950 4000
Wire Wire Line
	1950 4100 1850 4100
Connection ~ 1850 4100
Wire Wire Line
	3300 4000 3300 4100
Wire Wire Line
	3300 4100 3300 4200
Wire Wire Line
	3300 4000 3200 4000
Wire Wire Line
	3200 4100 3300 4100
Connection ~ 3300 4100
Wire Wire Line
	1850 3300 1850 3500
Connection ~ 1850 3500
Wire Wire Line
	1250 3300 1350 3300
$EndSCHEMATC
