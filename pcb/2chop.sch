EESchema Schematic File Version 2  date Tue 02 Apr 2013 02:02:16 PM CEST
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
Sheet 5 6
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
L TL082 U301
U 1 1 5141E5EA
P 4550 3950
AR Path="/5151799D/5141E5EA" Ref="U301"  Part="1" 
AR Path="/5152710B/5141E5EA" Ref="U401"  Part="1" 
AR Path="/5152811C/5141E5EA" Ref="U501"  Part="1" 
AR Path="/51529554/5141E5EA" Ref="U601"  Part="1" 
F 0 "U501" H 4500 4150 60  0000 L CNN
F 1 "TL082" H 4500 3700 60  0000 L CNN
F 2 "" H 4550 3950 60  0001 C CNN
F 3 "" H 4550 3950 60  0001 C CNN
	1    4550 3950
	1    0    0    1   
$EndComp
$Comp
L R R306
U 1 1 5141E658
P 4800 3200
AR Path="/5151799D/5141E658" Ref="R306"  Part="1" 
AR Path="/5152710B/5141E658" Ref="R406"  Part="1" 
AR Path="/5152811C/5141E658" Ref="R506"  Part="1" 
AR Path="/51529554/5141E658" Ref="R606"  Part="1" 
F 0 "R506" V 4880 3200 50  0000 C CNN
F 1 "1K" V 4800 3200 50  0000 C CNN
F 2 "" H 4800 3200 60  0001 C CNN
F 3 "" H 4800 3200 60  0001 C CNN
	1    4800 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R303
U 1 1 5141E665
P 3400 3200
AR Path="/5151799D/5141E665" Ref="R303"  Part="1" 
AR Path="/5152710B/5141E665" Ref="R403"  Part="1" 
AR Path="/5152811C/5141E665" Ref="R503"  Part="1" 
AR Path="/51529554/5141E665" Ref="R603"  Part="1" 
F 0 "R503" V 3480 3200 50  0000 C CNN
F 1 "NI" V 3400 3200 50  0000 C CNN
F 2 "" H 3400 3200 60  0001 C CNN
F 3 "" H 3400 3200 60  0001 C CNN
	1    3400 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R304
U 1 1 5141E66B
P 3400 3850
AR Path="/5151799D/5141E66B" Ref="R304"  Part="1" 
AR Path="/5152710B/5141E66B" Ref="R404"  Part="1" 
AR Path="/5152811C/5141E66B" Ref="R504"  Part="1" 
AR Path="/51529554/5141E66B" Ref="R604"  Part="1" 
F 0 "R504" V 3480 3850 50  0000 C CNN
F 1 "1K" V 3400 3850 50  0000 C CNN
F 2 "" H 3400 3850 60  0001 C CNN
F 3 "" H 3400 3850 60  0001 C CNN
	1    3400 3850
	0    -1   -1   0   
$EndComp
$Comp
L R R305
U 1 1 5141E671
P 3400 4050
AR Path="/5151799D/5141E671" Ref="R305"  Part="1" 
AR Path="/5152710B/5141E671" Ref="R405"  Part="1" 
AR Path="/5152811C/5141E671" Ref="R505"  Part="1" 
AR Path="/51529554/5141E671" Ref="R605"  Part="1" 
F 0 "R505" V 3480 4050 50  0000 C CNN
F 1 "0R" V 3400 4050 50  0000 C CNN
F 2 "" H 3400 4050 60  0001 C CNN
F 3 "" H 3400 4050 60  0001 C CNN
	1    3400 4050
	0    -1   -1   0   
$EndComp
$Comp
L R R302
U 1 1 5141E677
P 3150 4400
AR Path="/5151799D/5141E677" Ref="R302"  Part="1" 
AR Path="/5152710B/5141E677" Ref="R402"  Part="1" 
AR Path="/5152811C/5141E677" Ref="R502"  Part="1" 
AR Path="/51529554/5141E677" Ref="R602"  Part="1" 
F 0 "R502" H 2950 4300 50  0000 C CNN
F 1 "0R" V 3150 4400 50  0000 C CNN
F 2 "" H 3150 4400 60  0001 C CNN
F 3 "" H 3150 4400 60  0001 C CNN
	1    3150 4400
	-1   0    0    1   
$EndComp
$Comp
L R R301
U 1 1 5141E67D
P 2650 4050
AR Path="/5151799D/5141E67D" Ref="R301"  Part="1" 
AR Path="/5152710B/5141E67D" Ref="R401"  Part="1" 
AR Path="/5152811C/5141E67D" Ref="R501"  Part="1" 
AR Path="/51529554/5141E67D" Ref="R601"  Part="1" 
F 0 "R501" V 2750 4050 50  0000 C CNN
F 1 "NI" V 2650 4050 50  0000 C CNN
F 2 "" H 2650 4050 60  0001 C CNN
F 3 "" H 2650 4050 60  0001 C CNN
	1    2650 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	5050 3200 5050 3400
Wire Wire Line
	5050 3400 5050 3950
Wire Wire Line
	3650 3200 3850 3200
Wire Wire Line
	3850 3200 4500 3200
Wire Wire Line
	4500 3200 4550 3200
Wire Wire Line
	3650 3850 3850 3850
Wire Wire Line
	3850 3850 4050 3850
Wire Wire Line
	3850 3200 3850 3850
Connection ~ 3850 3850
Connection ~ 3850 3200
Wire Wire Line
	3650 4050 4050 4050
Wire Wire Line
	2900 4050 3150 4050
Wire Wire Line
	2400 3850 2900 3850
Wire Wire Line
	2900 3850 3150 3850
Wire Wire Line
	2400 3850 2400 4050
$Comp
L C C301
U 1 1 515177F5
P 2900 4400
AR Path="/5151799D/515177F5" Ref="C301"  Part="1" 
AR Path="/5152710B/515177F5" Ref="C401"  Part="1" 
AR Path="/5152811C/515177F5" Ref="C501"  Part="1" 
AR Path="/51529554/515177F5" Ref="C601"  Part="1" 
F 0 "C501" H 2700 4500 50  0000 L CNN
F 1 "NI" H 2800 4300 50  0000 L CNN
F 2 "" H 2900 4400 60  0001 C CNN
F 3 "" H 2900 4400 60  0001 C CNN
	1    2900 4400
	1    0    0    -1  
$EndComp
$Comp
L C C304
U 1 1 51517804
P 4850 3400
AR Path="/5151799D/51517804" Ref="C304"  Part="1" 
AR Path="/5152710B/51517804" Ref="C404"  Part="1" 
AR Path="/5152811C/51517804" Ref="C504"  Part="1" 
AR Path="/51529554/51517804" Ref="C604"  Part="1" 
F 0 "C504" V 4900 3450 50  0000 L CNN
F 1 "39pF" V 4800 3450 50  0000 L CNN
F 2 "" H 4850 3400 60  0001 C CNN
F 3 "" H 4850 3400 60  0001 C CNN
	1    4850 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3150 3300 3150 3200
Wire Wire Line
	2900 4600 2900 4650
Text HLabel 5950 4300 2    60   Input ~ 0
OUT2
Text HLabel 4350 4350 0    60   Input ~ 0
+14
Text HLabel 4350 3550 0    60   Input ~ 0
-14
$Comp
L R R313
U 1 1 5151790E
P 8550 3200
AR Path="/5151799D/5151790E" Ref="R313"  Part="1" 
AR Path="/5152710B/5151790E" Ref="R413"  Part="1" 
AR Path="/5152811C/5151790E" Ref="R513"  Part="1" 
AR Path="/51529554/5151790E" Ref="R613"  Part="1" 
F 0 "R513" V 8630 3200 50  0000 C CNN
F 1 "1K" V 8550 3200 50  0000 C CNN
F 2 "" H 8550 3200 60  0001 C CNN
F 3 "" H 8550 3200 60  0001 C CNN
	1    8550 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R310
U 1 1 51517914
P 7150 3200
AR Path="/5151799D/51517914" Ref="R310"  Part="1" 
AR Path="/5152710B/51517914" Ref="R410"  Part="1" 
AR Path="/5152811C/51517914" Ref="R510"  Part="1" 
AR Path="/51529554/51517914" Ref="R610"  Part="1" 
F 0 "R510" V 7230 3200 50  0000 C CNN
F 1 "NI" V 7150 3200 50  0000 C CNN
F 2 "" H 7150 3200 60  0001 C CNN
F 3 "" H 7150 3200 60  0001 C CNN
	1    7150 3200
	0    -1   -1   0   
$EndComp
$Comp
L R R311
U 1 1 5151791A
P 7150 3850
AR Path="/5151799D/5151791A" Ref="R311"  Part="1" 
AR Path="/5152710B/5151791A" Ref="R411"  Part="1" 
AR Path="/5152811C/5151791A" Ref="R511"  Part="1" 
AR Path="/51529554/5151791A" Ref="R611"  Part="1" 
F 0 "R511" V 7230 3850 50  0000 C CNN
F 1 "1K" V 7150 3850 50  0000 C CNN
F 2 "" H 7150 3850 60  0001 C CNN
F 3 "" H 7150 3850 60  0001 C CNN
	1    7150 3850
	0    -1   -1   0   
$EndComp
$Comp
L R R312
U 1 1 51517920
P 7150 4050
AR Path="/5151799D/51517920" Ref="R312"  Part="1" 
AR Path="/5152710B/51517920" Ref="R412"  Part="1" 
AR Path="/5152811C/51517920" Ref="R512"  Part="1" 
AR Path="/51529554/51517920" Ref="R612"  Part="1" 
F 0 "R512" V 7230 4050 50  0000 C CNN
F 1 "0" V 7150 4050 50  0000 C CNN
F 2 "" H 7150 4050 60  0001 C CNN
F 3 "" H 7150 4050 60  0001 C CNN
	1    7150 4050
	0    -1   -1   0   
$EndComp
$Comp
L R R309
U 1 1 51517926
P 6900 4400
AR Path="/5151799D/51517926" Ref="R309"  Part="1" 
AR Path="/5152710B/51517926" Ref="R409"  Part="1" 
AR Path="/5152811C/51517926" Ref="R509"  Part="1" 
AR Path="/51529554/51517926" Ref="R609"  Part="1" 
F 0 "R509" H 6700 4350 50  0000 C CNN
F 1 "0R" V 6900 4400 50  0000 C CNN
F 2 "" H 6900 4400 60  0001 C CNN
F 3 "" H 6900 4400 60  0001 C CNN
	1    6900 4400
	-1   0    0    1   
$EndComp
$Comp
L R R308
U 1 1 5151792C
P 6400 4050
AR Path="/5151799D/5151792C" Ref="R308"  Part="1" 
AR Path="/5152710B/5151792C" Ref="R408"  Part="1" 
AR Path="/5152811C/5151792C" Ref="R508"  Part="1" 
AR Path="/51529554/5151792C" Ref="R608"  Part="1" 
F 0 "R508" V 6500 4050 50  0000 C CNN
F 1 "NI" V 6400 4050 50  0000 C CNN
F 2 "" H 6400 4050 60  0001 C CNN
F 3 "" H 6400 4050 60  0001 C CNN
	1    6400 4050
	0    1    1    0   
$EndComp
Wire Wire Line
	8800 3200 8800 3400
Wire Wire Line
	8800 3400 8800 3950
Wire Wire Line
	7400 3200 7600 3200
Wire Wire Line
	7600 3200 8300 3200
Wire Wire Line
	7400 3850 7600 3850
Wire Wire Line
	7600 3850 7800 3850
Wire Wire Line
	7600 3200 7600 3850
Connection ~ 7600 3850
Connection ~ 7600 3200
Wire Wire Line
	7400 4050 7800 4050
Wire Wire Line
	6650 4050 6900 4050
Wire Wire Line
	6150 3850 6650 3850
Wire Wire Line
	6650 3850 6900 3850
Wire Wire Line
	6150 3850 6150 4050
$Comp
L C C306
U 1 1 51517941
P 6650 4400
AR Path="/5151799D/51517941" Ref="C306"  Part="1" 
AR Path="/5152710B/51517941" Ref="C406"  Part="1" 
AR Path="/5152811C/51517941" Ref="C506"  Part="1" 
AR Path="/51529554/51517941" Ref="C606"  Part="1" 
F 0 "C506" H 6450 4500 50  0000 L CNN
F 1 "NI" H 6550 4300 50  0000 L CNN
F 2 "" H 6650 4400 60  0001 C CNN
F 3 "" H 6650 4400 60  0001 C CNN
	1    6650 4400
	1    0    0    -1  
$EndComp
$Comp
L C C307
U 1 1 51517947
P 8600 3400
AR Path="/5151799D/51517947" Ref="C307"  Part="1" 
AR Path="/5152710B/51517947" Ref="C407"  Part="1" 
AR Path="/5152811C/51517947" Ref="C507"  Part="1" 
AR Path="/51529554/51517947" Ref="C607"  Part="1" 
F 0 "C507" V 8650 3450 50  0000 L CNN
F 1 "39pF" V 8550 3450 50  0000 L CNN
F 2 "" H 8600 3400 60  0001 C CNN
F 3 "" H 8600 3400 60  0001 C CNN
	1    8600 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6900 3300 6900 3200
Wire Wire Line
	6650 4050 6650 4200
Wire Wire Line
	6650 4600 6650 4650
Wire Wire Line
	6650 4650 6900 4650
Wire Wire Line
	6900 4650 6900 4750
Wire Wire Line
	8300 3200 8300 3400
Text HLabel 9750 4300 2    60   Input ~ 0
OUT1
Text HLabel 8100 4350 0    60   Input ~ 0
+14
Text HLabel 8100 3550 0    60   Input ~ 0
-14
$Comp
L TL082 U301
U 2 1 5141E5BD
P 8300 3950
AR Path="/5151799D/5141E5BD" Ref="U301"  Part="2" 
AR Path="/5152710B/5141E5BD" Ref="U401"  Part="2" 
AR Path="/5152811C/5141E5BD" Ref="U501"  Part="2" 
AR Path="/51529554/5141E5BD" Ref="U601"  Part="2" 
F 0 "U501" H 8250 4150 60  0000 L CNN
F 1 "TL082" H 8250 3700 60  0000 L CNN
F 2 "" H 8300 3950 60  0001 C CNN
F 3 "" H 8300 3950 60  0001 C CNN
	2    8300 3950
	1    0    0    1   
$EndComp
$Comp
L C C302
U 1 1 5151B311
P 2800 1900
AR Path="/5151799D/5151B311" Ref="C302"  Part="1" 
AR Path="/5152710B/5151B311" Ref="C402"  Part="1" 
AR Path="/5152811C/5151B311" Ref="C502"  Part="1" 
AR Path="/51529554/5151B311" Ref="C602"  Part="1" 
F 0 "C502" H 2950 1850 50  0000 L CNN
F 1 "100nF" H 2950 1950 50  0000 L CNN
F 2 "" H 2800 1900 60  0001 C CNN
F 3 "" H 2800 1900 60  0001 C CNN
	1    2800 1900
	-1   0    0    1   
$EndComp
$Comp
L C C303
U 1 1 5151B335
P 3100 1900
AR Path="/5151799D/5151B335" Ref="C303"  Part="1" 
AR Path="/5152710B/5151B335" Ref="C403"  Part="1" 
AR Path="/5152811C/5151B335" Ref="C503"  Part="1" 
AR Path="/51529554/5151B335" Ref="C603"  Part="1" 
F 0 "C503" H 2750 1850 50  0000 L CNN
F 1 "100nF" H 2700 1950 50  0000 L CNN
F 2 "" H 3100 1900 60  0001 C CNN
F 3 "" H 3100 1900 60  0001 C CNN
	1    3100 1900
	-1   0    0    1   
$EndComp
Text HLabel 2700 1700 0    60   Input ~ 0
+14
Text HLabel 3200 1700 2    60   Input ~ 0
-14
Wire Wire Line
	2800 1700 2700 1700
Wire Wire Line
	2800 2100 2950 2100
Wire Wire Line
	2950 2100 3100 2100
Wire Wire Line
	3100 2100 3200 2100
Wire Wire Line
	2950 2100 2950 2250
Connection ~ 2950 2100
Wire Wire Line
	3200 1700 3100 1700
$Comp
L R R307
U 1 1 515217D1
P 5600 4300
AR Path="/5151799D/515217D1" Ref="R307"  Part="1" 
AR Path="/5152710B/515217D1" Ref="R407"  Part="1" 
AR Path="/5152811C/515217D1" Ref="R507"  Part="1" 
AR Path="/51529554/515217D1" Ref="R607"  Part="1" 
F 0 "R507" V 5680 4300 50  0000 C CNN
F 1 "10R" V 5600 4300 50  0000 C CNN
F 2 "" H 5600 4300 60  0001 C CNN
F 3 "" H 5600 4300 60  0001 C CNN
	1    5600 4300
	0    -1   -1   0   
$EndComp
$Comp
L C C305
U 1 1 515217E3
P 5850 4500
AR Path="/5151799D/515217E3" Ref="C305"  Part="1" 
AR Path="/5152710B/515217E3" Ref="C405"  Part="1" 
AR Path="/5152811C/515217E3" Ref="C505"  Part="1" 
AR Path="/51529554/515217E3" Ref="C605"  Part="1" 
F 0 "C505" H 6000 4450 50  0000 L CNN
F 1 "2200pF" H 6000 4550 50  0000 L CNN
F 2 "" H 5850 4500 60  0001 C CNN
F 3 "" H 5850 4500 60  0001 C CNN
	1    5850 4500
	-1   0    0    1   
$EndComp
$Comp
L AGND #PWR035
U 1 1 51521807
P 3150 3300
AR Path="/5151799D/51521807" Ref="#PWR035"  Part="1" 
AR Path="/5152710B/51521807" Ref="#PWR044"  Part="1" 
AR Path="/5152811C/51521807" Ref="#PWR053"  Part="1" 
AR Path="/51529554/51521807" Ref="#PWR062"  Part="1" 
F 0 "#PWR053" H 3150 3300 40  0001 C CNN
F 1 "AGND" H 3150 3230 50  0000 C CNN
F 2 "" H 3150 3300 60  0001 C CNN
F 3 "" H 3150 3300 60  0001 C CNN
	1    3150 3300
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR036
U 1 1 5152181E
P 3150 4750
AR Path="/5151799D/5152181E" Ref="#PWR036"  Part="1" 
AR Path="/5152710B/5152181E" Ref="#PWR045"  Part="1" 
AR Path="/5152811C/5152181E" Ref="#PWR054"  Part="1" 
AR Path="/51529554/5152181E" Ref="#PWR063"  Part="1" 
F 0 "#PWR054" H 3150 4750 40  0001 C CNN
F 1 "AGND" H 3150 4680 50  0000 C CNN
F 2 "" H 3150 4750 60  0001 C CNN
F 3 "" H 3150 4750 60  0001 C CNN
	1    3150 4750
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR037
U 1 1 51521824
P 2950 2250
AR Path="/5151799D/51521824" Ref="#PWR037"  Part="1" 
AR Path="/5152710B/51521824" Ref="#PWR046"  Part="1" 
AR Path="/5152811C/51521824" Ref="#PWR055"  Part="1" 
AR Path="/51529554/51521824" Ref="#PWR064"  Part="1" 
F 0 "#PWR055" H 2950 2250 40  0001 C CNN
F 1 "AGND" H 2950 2180 50  0000 C CNN
F 2 "" H 2950 2250 60  0001 C CNN
F 3 "" H 2950 2250 60  0001 C CNN
	1    2950 2250
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR038
U 1 1 5152182A
P 6900 4750
AR Path="/5151799D/5152182A" Ref="#PWR038"  Part="1" 
AR Path="/5152710B/5152182A" Ref="#PWR047"  Part="1" 
AR Path="/5152811C/5152182A" Ref="#PWR056"  Part="1" 
AR Path="/51529554/5152182A" Ref="#PWR065"  Part="1" 
F 0 "#PWR056" H 6900 4750 40  0001 C CNN
F 1 "AGND" H 6900 4680 50  0000 C CNN
F 2 "" H 6900 4750 60  0001 C CNN
F 3 "" H 6900 4750 60  0001 C CNN
	1    6900 4750
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR039
U 1 1 5152183A
P 5850 4800
AR Path="/5151799D/5152183A" Ref="#PWR039"  Part="1" 
AR Path="/5152710B/5152183A" Ref="#PWR048"  Part="1" 
AR Path="/5152811C/5152183A" Ref="#PWR057"  Part="1" 
AR Path="/51529554/5152183A" Ref="#PWR066"  Part="1" 
F 0 "#PWR057" H 5850 4800 40  0001 C CNN
F 1 "AGND" H 5850 4730 50  0000 C CNN
F 2 "" H 5850 4800 60  0001 C CNN
F 3 "" H 5850 4800 60  0001 C CNN
	1    5850 4800
	1    0    0    -1  
$EndComp
$Comp
L AGND #PWR040
U 1 1 5152184A
P 6900 3300
AR Path="/5151799D/5152184A" Ref="#PWR040"  Part="1" 
AR Path="/5152710B/5152184A" Ref="#PWR049"  Part="1" 
AR Path="/5152811C/5152184A" Ref="#PWR058"  Part="1" 
AR Path="/51529554/5152184A" Ref="#PWR067"  Part="1" 
F 0 "#PWR058" H 6900 3300 40  0001 C CNN
F 1 "AGND" H 6900 3230 50  0000 C CNN
F 2 "" H 6900 3300 60  0001 C CNN
F 3 "" H 6900 3300 60  0001 C CNN
	1    6900 3300
	1    0    0    -1  
$EndComp
$Comp
L R R314
U 1 1 5152185C
P 9400 4300
AR Path="/5151799D/5152185C" Ref="R314"  Part="1" 
AR Path="/5152710B/5152185C" Ref="R414"  Part="1" 
AR Path="/5152811C/5152185C" Ref="R514"  Part="1" 
AR Path="/51529554/5152185C" Ref="R614"  Part="1" 
F 0 "R514" V 9480 4300 50  0000 C CNN
F 1 "10R" V 9400 4300 50  0000 C CNN
F 2 "" H 9400 4300 60  0001 C CNN
F 3 "" H 9400 4300 60  0001 C CNN
	1    9400 4300
	0    -1   -1   0   
$EndComp
$Comp
L C C308
U 1 1 51521862
P 9650 4500
AR Path="/5151799D/51521862" Ref="C308"  Part="1" 
AR Path="/5152710B/51521862" Ref="C408"  Part="1" 
AR Path="/5152811C/51521862" Ref="C508"  Part="1" 
AR Path="/51529554/51521862" Ref="C608"  Part="1" 
F 0 "C508" H 9800 4450 50  0000 L CNN
F 1 "2200pF" H 9800 4550 50  0000 L CNN
F 2 "" H 9650 4500 60  0001 C CNN
F 3 "" H 9650 4500 60  0001 C CNN
	1    9650 4500
	-1   0    0    1   
$EndComp
$Comp
L AGND #PWR041
U 1 1 5152186E
P 9650 4800
AR Path="/5151799D/5152186E" Ref="#PWR041"  Part="1" 
AR Path="/5152710B/5152186E" Ref="#PWR050"  Part="1" 
AR Path="/5152811C/5152186E" Ref="#PWR059"  Part="1" 
AR Path="/51529554/5152186E" Ref="#PWR068"  Part="1" 
F 0 "#PWR059" H 9650 4800 40  0001 C CNN
F 1 "AGND" H 9650 4730 50  0000 C CNN
F 2 "" H 9650 4800 60  0001 C CNN
F 3 "" H 9650 4800 60  0001 C CNN
	1    9650 4800
	1    0    0    -1  
$EndComp
$Comp
L CONN_3 K302
U 1 1 5152197F
P 9150 3600
AR Path="/5151799D/5152197F" Ref="K302"  Part="1" 
AR Path="/5152710B/5152197F" Ref="K402"  Part="1" 
AR Path="/5152811C/5152197F" Ref="K502"  Part="1" 
AR Path="/51529554/5152197F" Ref="K602"  Part="1" 
F 0 "K502" V 9100 3600 50  0000 C CNN
F 1 "CONN_3" V 9200 3600 40  0000 C CNN
F 2 "" H 9150 3600 60  0001 C CNN
F 3 "" H 9150 3600 60  0001 C CNN
	1    9150 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8800 3950 9050 3950
Wire Wire Line
	9150 3950 9150 4300
Wire Wire Line
	9250 3950 9450 3950
Wire Wire Line
	9450 3950 9450 2750
Wire Wire Line
	9450 2750 6650 2750
Wire Wire Line
	6650 2750 4400 2750
Wire Wire Line
	6650 2750 6650 3850
Connection ~ 6650 3850
$Comp
L CONN_3 K301
U 1 1 51521A30
P 5350 3600
AR Path="/5151799D/51521A30" Ref="K301"  Part="1" 
AR Path="/5152710B/51521A30" Ref="K401"  Part="1" 
AR Path="/5152811C/51521A30" Ref="K501"  Part="1" 
AR Path="/51529554/51521A30" Ref="K601"  Part="1" 
F 0 "K501" V 5300 3600 50  0000 C CNN
F 1 "CONN_3" V 5400 3600 40  0000 C CNN
F 2 "" H 5350 3600 60  0001 C CNN
F 3 "" H 5350 3600 60  0001 C CNN
	1    5350 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5050 3950 5250 3950
Wire Wire Line
	5350 3950 5350 4300
Wire Wire Line
	5450 3950 5650 3950
Wire Wire Line
	5650 3950 5650 2850
Wire Wire Line
	5650 2850 4300 2850
Wire Wire Line
	4300 2850 2900 2850
Wire Wire Line
	2900 2850 2900 3850
Connection ~ 2900 3850
Text HLabel 3200 2100 2    60   Input ~ 0
AGND
Wire Wire Line
	4650 3400 4500 3400
Wire Wire Line
	4500 3400 4500 3200
Connection ~ 4500 3200
Wire Wire Line
	8300 3400 8400 3400
$Comp
L CONN_5X2 P301
U 1 1 515254E2
P 4500 1900
AR Path="/5151799D/515254E2" Ref="P301"  Part="1" 
AR Path="/5152710B/515254E2" Ref="P401"  Part="1" 
AR Path="/5152811C/515254E2" Ref="P501"  Part="1" 
AR Path="/51529554/515254E2" Ref="P601"  Part="1" 
F 0 "P501" H 4500 2200 60  0000 C CNN
F 1 "CONN_5X2" V 4500 1900 50  0000 C CNN
F 2 "" H 4500 1900 60  0001 C CNN
F 3 "" H 4500 1900 60  0001 C CNN
	1    4500 1900
	0    -1   -1   0   
$EndComp
$Comp
L AGND #PWR043
U 1 1 515254EF
P 4150 1500
AR Path="/5151799D/515254EF" Ref="#PWR043"  Part="1" 
AR Path="/5152710B/515254EF" Ref="#PWR052"  Part="1" 
AR Path="/5152811C/515254EF" Ref="#PWR061"  Part="1" 
AR Path="/51529554/515254EF" Ref="#PWR070"  Part="1" 
F 0 "#PWR061" H 4150 1500 40  0001 C CNN
F 1 "AGND" H 4150 1430 50  0000 C CNN
F 2 "" H 4150 1500 60  0001 C CNN
F 3 "" H 4150 1500 60  0001 C CNN
	1    4150 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 1400 4150 1500
Wire Wire Line
	4300 2300 4300 2850
Connection ~ 4300 2850
Wire Wire Line
	4400 2750 4400 2300
Connection ~ 6650 2750
Text HLabel 4500 2300 3    60   Input ~ 0
-14
Text HLabel 4600 2300 3    60   Input ~ 0
+14
Text HLabel 4700 2300 3    60   Input ~ 0
5v
Connection ~ 5050 3400
Connection ~ 8800 3400
Wire Wire Line
	2900 4200 2900 4050
Wire Wire Line
	3150 4050 3150 4150
Wire Wire Line
	3150 4650 3150 4750
Wire Wire Line
	2900 4650 3150 4650
Connection ~ 3100 2100
Wire Wire Line
	6900 4050 6900 4150
Wire Wire Line
	4150 1400 4300 1400
Wire Wire Line
	4300 1400 4400 1400
Wire Wire Line
	4400 1400 4500 1400
Wire Wire Line
	4500 1400 4600 1400
Wire Wire Line
	4600 1400 4700 1400
Wire Wire Line
	4700 1400 4700 1500
Wire Wire Line
	4600 1500 4600 1400
Connection ~ 4600 1400
Wire Wire Line
	4500 1400 4500 1500
Connection ~ 4500 1400
Wire Wire Line
	4400 1500 4400 1400
Connection ~ 4400 1400
Wire Wire Line
	4300 1400 4300 1500
Connection ~ 4300 1400
Connection ~ 2900 4050
Connection ~ 3150 4050
Connection ~ 3150 4650
Connection ~ 6900 4050
Connection ~ 6650 4050
Connection ~ 6900 4650
Connection ~ 8800 3950
Connection ~ 5050 3950
Wire Wire Line
	5950 4300 5850 4300
Wire Wire Line
	9750 4300 9650 4300
Connection ~ 9650 4300
Connection ~ 5850 4300
Wire Wire Line
	9650 4800 9650 4700
Wire Wire Line
	5850 4800 5850 4700
Wire Wire Line
	8100 4350 8200 4350
Wire Wire Line
	8200 3550 8100 3550
Wire Wire Line
	4350 3550 4450 3550
Wire Wire Line
	4450 4350 4350 4350
$EndSCHEMATC
