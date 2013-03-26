EESchema Schematic File Version 2  date Tue 26 Mar 2013 09:30:36 PM CET
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
LIBS:curry2-cache
EELAYER 27 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 6
Title ""
Date "26 mar 2013"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ADS8568 U?
U 1 1 513F8222
P 6450 3650
F 0 "U?" H 6400 3550 60  0000 C CNN
F 1 "ADS8568" H 6400 3800 60  0000 C CNN
	1    6450 3650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5141E2A2
P 8350 3250
F 0 "C?" H 8400 3350 50  0000 L CNN
F 1 "10µF" H 8400 3150 50  0000 L CNN
	1    8350 3250
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5141E2C2
P 7100 1800
F 0 "C?" H 7150 1900 50  0000 L CNN
F 1 "10µF" H 7150 1700 50  0000 L CNN
	1    7100 1800
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5141E2D2
P 6050 1850
F 0 "C?" H 6100 1950 50  0000 L CNN
F 1 "10µF" H 6100 1750 50  0000 L CNN
	1    6050 1850
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 5141E2D8
P 4750 3250
F 0 "C?" H 4800 3350 50  0000 L CNN
F 1 "10µF" H 4800 3150 50  0000 L CNN
	1    4750 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 3450 8350 3450
Wire Wire Line
	7800 3150 8050 3150
Wire Wire Line
	8050 3150 8050 3050
Wire Wire Line
	8050 3050 8350 3050
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
$Sheet
S 750  6000 1400 350 
U 5151B8A3
F0 "power" 60
F1 "power.sch" 60
F2 "Enable" I L 750 6100 60 
F3 "+14" I L 750 6200 60 
F4 "-14" I L 750 6300 60 
F5 "+5" I R 2150 6100 60 
F6 "GND" I R 2150 6200 60 
$EndSheet
$Comp
L INDUCTOR L?
U 1 1 5151FA61
P 3600 1950
F 0 "L?" V 3550 1950 40  0000 C CNN
F 1 "INDUCTOR" V 3700 1950 40  0000 C CNN
	1    3600 1950
	0    -1   -1   0   
$EndComp
$Comp
L INDUCTOR L?
U 1 1 5151FB55
P 1500 1900
F 0 "L?" V 1450 1900 40  0000 C CNN
F 1 "INDUCTOR" V 1600 1900 40  0000 C CNN
	1    1500 1900
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 5151FC21
P 4100 2150
F 0 "C?" H 4150 2250 50  0000 L CNN
F 1 "10µF" H 4150 2050 50  0000 L CNN
	1    4100 2150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5151FC48
P 1100 2100
F 0 "C?" H 1150 2200 50  0000 L CNN
F 1 "10µF" H 1150 2000 50  0000 L CNN
	1    1100 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1950 3300 1950
Wire Wire Line
	3200 1900 3200 2000
Connection ~ 3200 1950
Wire Wire Line
	3900 1950 4100 1950
Wire Wire Line
	1950 1900 1800 1900
Wire Wire Line
	850  1900 1200 1900
$Comp
L GND #PWR?
U 1 1 515200D1
P 4100 2350
F 0 "#PWR?" H 4100 2350 30  0001 C CNN
F 1 "GND" H 4100 2280 30  0001 C CNN
	1    4100 2350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 51520228
P 1100 2300
F 0 "#PWR?" H 1100 2300 30  0001 C CNN
F 1 "GND" H 1100 2230 30  0001 C CNN
	1    1100 2300
	1    0    0    -1  
$EndComp
Text GLabel 4100 1950 2    60   Input ~ 0
AVDD
Text GLabel 850  1900 0    60   Input ~ 0
DVDD
$Comp
L GND #PWR?
U 1 1 515208A0
P 3400 2100
F 0 "#PWR?" H 3400 2100 30  0001 C CNN
F 1 "GND" H 3400 2030 30  0001 C CNN
	1    3400 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 2100 3400 2100
$Comp
L C C?
U 1 1 51520C5B
P 3900 2150
F 0 "C?" H 3950 2250 50  0000 L CNN
F 1 "10µF" H 3950 2050 50  0000 L CNN
	1    3900 2150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 51520C82
P 850 2100
F 0 "C?" H 900 2200 50  0000 L CNN
F 1 "10µF" H 900 2000 50  0000 L CNN
	1    850  2100
	1    0    0    -1  
$EndComp
Connection ~ 1100 1900
Wire Wire Line
	850  2300 1100 2300
Wire Wire Line
	3900 2350 4100 2350
Text GLabel 5150 6050 1    60   Input ~ 0
AVDD
$Comp
L C C?
U 1 1 51520FF3
P 4950 6350
F 0 "C?" H 5000 6450 50  0000 L CNN
F 1 "0.1µF" H 5000 6250 50  0000 L CNN
	1    4950 6350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 51521026
P 5200 6350
F 0 "C?" H 5250 6450 50  0000 L CNN
F 1 "0.1µF" H 5250 6250 50  0000 L CNN
	1    5200 6350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 5152102C
P 5450 6350
F 0 "C?" H 5500 6450 50  0000 L CNN
F 1 "0.1µF" H 5500 6250 50  0000 L CNN
	1    5450 6350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 51521032
P 5700 6350
F 0 "C?" H 5750 6450 50  0000 L CNN
F 1 "0.1µF" H 5750 6250 50  0000 L CNN
	1    5700 6350
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 51521038
P 5950 6350
F 0 "C?" H 6000 6450 50  0000 L CNN
F 1 "0.1µF" H 6000 6250 50  0000 L CNN
	1    5950 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4950 6150 5950 6150
Connection ~ 5700 6150
Connection ~ 5450 6150
Connection ~ 5200 6150
Wire Wire Line
	5150 6050 5150 6150
Connection ~ 5150 6150
Wire Wire Line
	4950 6550 5950 6550
Connection ~ 5450 6550
Text GLabel 5600 6550 3    60   Input ~ 0
AGND
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
Wire Wire Line
	7800 3050 7950 3050
Wire Wire Line
	7950 3050 7950 3000
Wire Wire Line
	7950 3000 8650 3000
Wire Wire Line
	7800 3550 8650 3550
Text GLabel 750  6200 0    47   Input ~ 0
HVDD
Text GLabel 750  6300 0    47   Input ~ 0
HVSS
Text GLabel 7800 2950 2    47   Input ~ 0
HVDD
Text GLabel 5200 2950 0    47   Input ~ 0
HVSS
Text GLabel 3400 5900 1    47   Input ~ 0
HVDD
Text GLabel 3400 6950 3    47   Input ~ 0
HVSS
Text GLabel 3000 6350 0    47   Input ~ 0
AGND
$Comp
L C C?
U 1 1 515232D4
P 3400 6150
F 0 "C?" H 3450 6250 50  0000 L CNN
F 1 "4.7µF" H 3450 6050 50  0000 L CNN
	1    3400 6150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 515234F6
P 3400 6650
F 0 "C?" H 3450 6750 50  0000 L CNN
F 1 "4.7µF" H 3450 6550 50  0000 L CNN
	1    3400 6650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 51523540
P 3700 6150
F 0 "C?" H 3750 6250 50  0000 L CNN
F 1 "4.7µF" H 3750 6050 50  0000 L CNN
	1    3700 6150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 51523546
P 3700 6650
F 0 "C?" H 3750 6750 50  0000 L CNN
F 1 "4.7µF" H 3750 6550 50  0000 L CNN
	1    3700 6650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 515235AF
P 4000 6150
F 0 "C?" H 4050 6250 50  0000 L CNN
F 1 "4.7µF" H 4050 6050 50  0000 L CNN
	1    4000 6150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 515235B5
P 4000 6650
F 0 "C?" H 4050 6750 50  0000 L CNN
F 1 "4.7µF" H 4050 6550 50  0000 L CNN
	1    4000 6650
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 515235BB
P 4300 6150
F 0 "C?" H 4350 6250 50  0000 L CNN
F 1 "4.7µF" H 4350 6050 50  0000 L CNN
	1    4300 6150
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 515235C1
P 4300 6650
F 0 "C?" H 4350 6750 50  0000 L CNN
F 1 "4.7µF" H 4350 6550 50  0000 L CNN
	1    4300 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 5950 3400 5950
Wire Wire Line
	3400 5950 3400 5900
Wire Wire Line
	4300 6450 4300 6350
Wire Wire Line
	4000 6450 4000 6350
Wire Wire Line
	3700 6450 3700 6350
Wire Wire Line
	3400 6450 3400 6350
Wire Wire Line
	4300 6850 3400 6850
Wire Wire Line
	3400 6850 3400 6950
Wire Wire Line
	4300 6350 3000 6350
Text GLabel 9950 3100 2    47   Input ~ 0
HVDD
$Sheet
S 9100 3000 850  500 
U 5151799D
F0 "chAop" 60
F1 "2chop.sch" 60
F2 "OUT2" I L 9100 3150 60 
F3 "+14" I R 9950 3100 60 
F4 "-14" I R 9950 3400 60 
F5 "OUT1" I L 9100 3350 60 
F6 "AGND" I R 9950 3300 60 
F7 "5v" I R 9950 3200 60 
$EndSheet
Text GLabel 9950 3300 2    47   Input ~ 0
AGND
Text GLabel 9950 3400 2    47   Input ~ 0
HVSS
Text GLabel 9950 3200 2    60   Input ~ 0
AVDD
Wire Wire Line
	8650 3000 8650 3150
Wire Wire Line
	8650 3150 9100 3150
Wire Wire Line
	8650 3550 8650 3350
Wire Wire Line
	8650 3350 9100 3350
Text GLabel 8850 1150 2    47   Input ~ 0
HVDD
$Sheet
S 8000 1050 850  500 
U 5152710B
F0 "chAop" 60
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
Text GLabel 8850 1250 2    60   Input ~ 0
AVDD
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
Text GLabel 7750 550  2    47   Input ~ 0
HVDD
$Sheet
S 6900 450  850  500 
U 5152811C
F0 "chAop" 60
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
Text GLabel 7750 650  2    60   Input ~ 0
AVDD
Wire Wire Line
	5800 800  6900 800 
Wire Wire Line
	5800 800  5800 2300
Wire Wire Line
	6300 2300 6300 600 
Connection ~ 6300 600 
Wire Wire Line
	6300 600  6900 600 
Text GLabel 5400 800  2    47   Input ~ 0
HVDD
$Sheet
S 4550 700  850  500 
U 51529554
F0 "chAop" 60
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
Text GLabel 5400 900  2    60   Input ~ 0
AVDD
Wire Wire Line
	5200 3550 4550 3550
Wire Wire Line
	4550 3550 4550 1600
Wire Wire Line
	4550 1600 4150 1600
Wire Wire Line
	4150 1600 4150 1050
Wire Wire Line
	4150 1050 4550 1050
Wire Wire Line
	5200 3050 4900 3050
Wire Wire Line
	4900 3050 4900 2950
Wire Wire Line
	4900 2950 4650 2950
Wire Wire Line
	4650 2950 4650 1500
Wire Wire Line
	4650 1500 4250 1500
Wire Wire Line
	4250 1500 4250 850 
Wire Wire Line
	4250 850  4550 850 
$Comp
L C C?
U 1 1 5152A2A3
P 6600 1950
F 0 "C?" H 6650 2050 50  0000 L CNN
F 1 "0.47µF" H 6650 1850 50  0000 L CNN
	1    6600 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 2300 6600 2150
Wire Wire Line
	6700 2300 6700 1750
Wire Wire Line
	6700 1750 6600 1750
Text GLabel 6700 1750 1    47   Input ~ 0
AGND
Entry Wire Line
	7300 4850 7400 4950
Entry Wire Line
	7200 4850 7300 4950
Entry Wire Line
	7100 4850 7200 4950
Entry Wire Line
	7000 4850 7100 4950
Entry Wire Line
	6900 4850 7000 4950
Entry Wire Line
	6800 4850 6900 4950
Entry Wire Line
	6700 4850 6800 4950
Entry Wire Line
	6400 4850 6500 4950
Entry Wire Line
	6300 4850 6400 4950
Entry Wire Line
	6200 4850 6300 4950
Entry Wire Line
	6100 4850 6200 4950
Entry Wire Line
	6000 4850 6100 4950
Entry Wire Line
	5900 4850 6000 4950
Entry Wire Line
	5800 4850 5900 4950
Entry Wire Line
	5200 4450 5300 4550
Wire Wire Line
	5300 5150 5300 4550
Wire Wire Line
	5900 5150 5900 4950
Wire Wire Line
	6000 5150 6000 4950
Wire Wire Line
	6100 5150 6100 4950
Wire Wire Line
	6200 5150 6200 4950
Wire Wire Line
	6300 5150 6300 4950
Wire Wire Line
	6400 5150 6400 4950
Wire Wire Line
	6500 5150 6500 4950
Wire Wire Line
	6800 5150 6800 4950
Wire Wire Line
	6900 5150 6900 4950
Wire Wire Line
	7000 5150 7000 4950
Wire Wire Line
	7100 5150 7100 4950
Wire Wire Line
	7200 5150 7200 4950
Wire Wire Line
	7300 5150 7300 4950
Wire Wire Line
	7400 5150 7400 4950
Wire Wire Line
	7650 5150 7650 4800
Wire Wire Line
	7650 4800 7800 4800
Wire Wire Line
	7800 4800 7800 4450
Text Label 7650 5150 3    60   ~ 0
DB0
Text Label 7400 5150 3    60   ~ 0
DB1
Text Label 7300 5150 3    60   ~ 0
DB2
Text Label 7200 5150 3    60   ~ 0
DB3
Text Label 7100 5150 3    60   ~ 0
DB4
Text Label 7000 5150 3    60   ~ 0
DB5
Text Label 6900 5150 3    60   ~ 0
DB6
Text Label 6800 5150 3    60   ~ 0
DB7
Text Label 6500 5150 3    60   ~ 0
DB8
Text Label 6400 5150 3    60   ~ 0
DB9
Text Label 6300 5150 3    60   ~ 0
DB10
Text Label 6200 5150 3    60   ~ 0
DB11
Text Label 6100 5150 3    60   ~ 0
DB12
Text Label 6000 5150 3    60   ~ 0
DB13
Text Label 5900 5150 3    60   ~ 0
DB14
Text Label 5300 5150 3    60   ~ 0
DB15
Wire Bus Line
	7650 5150 5300 5150
Wire Wire Line
	1750 2000 1950 2000
Wire Wire Line
	1950 2100 1750 2100
Wire Wire Line
	1950 2200 1750 2200
Wire Wire Line
	1950 2300 1750 2300
Wire Wire Line
	1950 2400 1750 2400
Wire Wire Line
	1950 2500 1750 2500
Wire Wire Line
	1750 2600 1950 2600
Wire Wire Line
	1950 2700 1750 2700
Wire Wire Line
	1950 2800 1750 2800
Wire Wire Line
	1950 2900 1750 2900
Wire Wire Line
	1950 3000 1750 3000
Wire Wire Line
	1950 3100 1750 3100
Wire Wire Line
	3200 2200 3300 2200
Wire Wire Line
	3200 2300 3300 2300
Wire Wire Line
	3200 2400 3300 2400
Wire Wire Line
	3200 2600 3300 2600
Wire Wire Line
	3200 2700 3300 2700
Wire Wire Line
	3200 2900 3300 2900
Wire Wire Line
	3200 3000 3300 3000
Wire Wire Line
	3200 3100 3450 3100
Text Label 1750 2100 2    60   ~ 0
DB0
Text Label 1750 2200 2    60   ~ 0
DB1
Text Label 3300 2200 0    60   ~ 0
DB2
Text Label 1750 2300 2    60   ~ 0
DB3
Text Label 3300 2300 0    60   ~ 0
DB4
Text Label 1750 2400 2    60   ~ 0
DB5
Text Label 3300 2400 0    60   ~ 0
DB6
Text Label 1750 2500 2    60   ~ 0
DB7
Text Label 1750 2600 2    60   ~ 0
DB8
Text Label 3300 2600 0    60   ~ 0
DB9
Text Label 3300 2700 0    60   ~ 0
DB10
Text Label 1750 2800 2    60   ~ 0
DB11
Text Label 1750 2900 2    60   ~ 0
DB12
Text Label 3300 2900 0    60   ~ 0
DB13
Text Label 1750 3000 2    60   ~ 0
DB14
Text Label 3300 3000 0    60   ~ 0
DB15
Wire Wire Line
	7800 4250 7950 4250
Text Label 7950 4250 0    60   ~ 0
BUSY/INT
Text Label 1750 2000 2    60   ~ 0
BUSY/INT
Text Label 5000 4050 0    60   ~ 0
RD
Wire Wire Line
	5200 4050 5000 4050
Text Label 3450 3100 0    60   ~ 0
RD
$Comp
L ATMEGA88-A IC?
U 1 1 51531D4C
P 9900 5800
F 0 "IC?" H 9200 7050 50  0000 L BNN
F 1 "ATMEGA88-A" H 10200 4400 50  0000 L BNN
F 2 "TQFP32" H 9350 4450 50  0001 C CNN
	1    9900 5800
	-1   0    0    1   
$EndComp
$Comp
L CRYSTAL X?
U 1 1 515330C4
P 8050 6250
F 0 "X?" H 8050 6400 60  0000 C CNN
F 1 "CRYSTAL" H 8050 6100 60  0000 C CNN
	1    8050 6250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8900 6200 8300 6200
Wire Wire Line
	8300 6200 8300 5950
Wire Wire Line
	8300 5950 7950 5950
Wire Wire Line
	8900 6300 8300 6300
Wire Wire Line
	8300 6300 8300 6550
Wire Wire Line
	8300 6550 7950 6550
$Comp
L C C?
U 1 1 51533631
P 7750 5950
F 0 "C?" H 7800 6050 50  0000 L CNN
F 1 "22pF" H 7800 5850 50  0000 L CNN
	1    7750 5950
	0    -1   -1   0   
$EndComp
$Comp
L C C?
U 1 1 515336E5
P 7750 6550
F 0 "C?" H 7800 6650 50  0000 L CNN
F 1 "22pF" H 7800 6450 50  0000 L CNN
	1    7750 6550
	0    -1   -1   0   
$EndComp
Connection ~ 8050 6550
Connection ~ 8050 5950
Wire Wire Line
	7550 5950 7550 6650
Text Label 5000 4150 0    60   ~ 0
CS
Wire Wire Line
	5200 4150 5000 4150
Wire Wire Line
	1950 3750 1800 3750
Text Label 1800 3750 0    60   ~ 0
CS
Wire Wire Line
	8900 5550 8750 5550
Wire Wire Line
	8900 5650 8750 5650
Text Label 8750 5550 0    60   ~ 0
SCL
Text Label 8750 5650 0    60   ~ 0
SDA
Text Label 4100 3650 0    60   ~ 0
SCL
Text Label 1750 3650 0    60   ~ 0
SDA
$Comp
L RASPI U?
U 1 1 513F6D96
P 2550 2300
F 0 "U?" H 2750 2950 60  0000 C CNN
F 1 "RASPI" H 2800 2850 60  0000 C CNN
	1    2550 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 3650 4000 3650
Wire Wire Line
	1750 3650 1950 3650
Wire Wire Line
	5200 3650 4950 3650
Wire Wire Line
	5200 3750 4950 3750
Wire Wire Line
	5200 3850 4950 3850
Wire Wire Line
	5200 3950 4950 3950
Text Label 4950 3650 0    60   ~ 0
PAR
Text Label 4950 3750 0    60   ~ 0
STD
Text Label 4950 3850 0    60   ~ 0
RES
Text Label 4950 3950 0    60   ~ 0
WR
Wire Wire Line
	7800 3650 7950 3650
Wire Wire Line
	7800 3750 7950 3750
Wire Wire Line
	7800 3850 7950 3850
Wire Wire Line
	7800 3950 7950 3950
Wire Wire Line
	7800 4050 7950 4050
Wire Wire Line
	7800 4150 7950 4150
Wire Wire Line
	7800 4350 7950 4350
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
Text Label 8550 5000 0    60   ~ 0
RANGE
Text Label 8450 4900 0    60   ~ 0
BUSY/INT
Text Label 8450 4800 0    60   ~ 0
SLEEP
Text Label 8500 6950 0    60   ~ 0
CONVA
Text Label 8500 6850 0    60   ~ 0
CONVB
Text Label 8500 6750 0    60   ~ 0
CONVC
Text Label 8500 6650 0    60   ~ 0
CONVD
Wire Wire Line
	8500 6650 8900 6650
Wire Wire Line
	8900 6650 8900 6600
Wire Wire Line
	8500 6750 8900 6750
Wire Wire Line
	8900 6750 8900 6700
Wire Wire Line
	8500 6850 8900 6850
Wire Wire Line
	8900 6850 8900 6800
Wire Wire Line
	8500 6950 8900 6950
Wire Wire Line
	8900 6950 8900 6900
Text Label 8550 6550 0    60   ~ 0
HW
Wire Wire Line
	8550 6550 8900 6550
Wire Wire Line
	8900 6550 8900 6500
Text Label 8700 6050 0    60   ~ 0
WR
Text Label 8700 5950 0    60   ~ 0
RES
Text Label 8700 5850 0    60   ~ 0
STD
Text Label 8700 5750 0    60   ~ 0
PAR
Wire Wire Line
	8700 5750 8900 5750
Wire Wire Line
	8700 5850 8900 5850
Wire Wire Line
	8700 5950 8900 5950
Wire Wire Line
	8700 6050 8900 6050
$Comp
L GND #PWR?
U 1 1 51539576
P 7550 6650
F 0 "#PWR?" H 7550 6650 30  0001 C CNN
F 1 "GND" H 7550 6580 30  0001 C CNN
	1    7550 6650
	1    0    0    -1  
$EndComp
Connection ~ 7550 6550
Text GLabel 6650 5350 3    60   Input ~ 0
DVDD
Wire Wire Line
	6500 4850 6550 4850
Wire Wire Line
	6550 4850 6550 5350
Wire Wire Line
	6550 5350 6500 5350
Wire Wire Line
	6600 4850 6600 5300
Wire Wire Line
	6600 5300 6650 5300
Wire Wire Line
	6650 5300 6650 5350
$Comp
L GND #PWR?
U 1 1 5153A199
P 6500 5400
F 0 "#PWR?" H 6500 5400 30  0001 C CNN
F 1 "GND" H 6500 5330 30  0001 C CNN
	1    6500 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 5350 6500 5400
Wire Wire Line
	8900 4800 8450 4800
Wire Wire Line
	8900 4900 8450 4900
Wire Wire Line
	8900 5000 8550 5000
$EndSCHEMATC
