EESchema Schematic File Version 2
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
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L 4051 U1
U 1 1 59AE92EC
P 8750 1800
F 0 "U1" H 8850 1800 50  0000 C CNN
F 1 "4051" H 8850 1600 50  0000 C CNN
F 2 "" H 8750 1800 60  0001 C CNN
F 3 "" H 8750 1800 60  0001 C CNN
	1    8750 1800
	1    0    0    -1  
$EndComp
$Comp
L 4051 U2
U 1 1 59AE9351
P 8750 3500
F 0 "U2" H 8850 3500 50  0000 C CNN
F 1 "4051" H 8850 3300 50  0000 C CNN
F 2 "" H 8750 3500 60  0001 C CNN
F 3 "" H 8750 3500 60  0001 C CNN
	1    8750 3500
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X16 P1
U 1 1 59AE94DE
P 6400 1950
F 0 "P1" H 6400 2800 50  0000 C CNN
F 1 "CONN_01X16" V 6500 1950 50  0000 C CNN
F 2 "" H 6400 1950 50  0000 C CNN
F 3 "" H 6400 1950 50  0000 C CNN
	1    6400 1950
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6600 1200 8050 1200
Wire Wire Line
	7300 2900 8050 2900
Wire Wire Line
	7300 2900 7300 1300
Wire Wire Line
	7300 1300 6600 1300
Wire Wire Line
	7250 3000 8050 3000
Wire Wire Line
	7250 3000 7250 1500
Wire Wire Line
	7250 1500 6600 1500
Wire Wire Line
	7200 3100 8050 3100
Wire Wire Line
	7200 3100 7200 1700
Wire Wire Line
	7200 1700 6600 1700
Wire Wire Line
	7150 3200 8050 3200
Wire Wire Line
	7150 3200 7150 1900
Wire Wire Line
	7150 1900 6600 1900
Wire Wire Line
	7100 3300 8050 3300
Wire Wire Line
	7100 3300 7100 2100
Wire Wire Line
	7100 2100 6600 2100
Wire Wire Line
	7050 3400 8050 3400
Wire Wire Line
	7050 3400 7050 2300
Wire Wire Line
	7050 2300 6600 2300
Wire Wire Line
	7000 3500 8050 3500
Wire Wire Line
	7000 3500 7000 2500
Wire Wire Line
	7000 2500 6600 2500
Wire Wire Line
	6950 3600 8050 3600
Wire Wire Line
	6950 3600 6950 2700
Wire Wire Line
	6950 2700 6600 2700
Wire Wire Line
	6600 1400 7400 1400
Wire Wire Line
	7400 1400 7400 1300
Wire Wire Line
	7400 1300 8050 1300
Wire Wire Line
	7450 1400 8050 1400
Wire Wire Line
	7450 1400 7450 1600
Wire Wire Line
	7450 1600 6600 1600
Wire Wire Line
	7500 1500 8050 1500
Wire Wire Line
	7500 1500 7500 1800
Wire Wire Line
	7500 1800 6600 1800
Wire Wire Line
	7550 1600 8050 1600
Wire Wire Line
	7550 1600 7550 2000
Wire Wire Line
	7550 2000 6600 2000
Wire Wire Line
	7600 1700 8050 1700
Wire Wire Line
	7600 1700 7600 2200
Wire Wire Line
	7600 2200 6600 2200
Wire Wire Line
	8050 1800 7650 1800
Wire Wire Line
	7650 1800 7650 2400
Wire Wire Line
	7650 2400 6600 2400
Wire Wire Line
	8050 1900 7700 1900
Wire Wire Line
	7700 1900 7700 2600
Wire Wire Line
	7700 2600 6600 2600
Wire Wire Line
	7950 2400 8050 2400
Wire Wire Line
	7950 2400 7950 4100
Wire Wire Line
	7950 4100 8050 4100
Wire Wire Line
	8050 2300 7900 2300
Wire Wire Line
	7900 2300 7900 4000
Wire Wire Line
	7900 4000 8050 4000
Wire Wire Line
	8050 3900 7850 3900
Wire Wire Line
	7850 3900 7850 2200
Wire Wire Line
	7850 2200 8050 2200
Wire Wire Line
	7950 2600 9950 2600
Connection ~ 7950 2600
Wire Wire Line
	7900 2650 9950 2650
Connection ~ 7900 2650
Connection ~ 7850 2700
Wire Wire Line
	7850 2700 9950 2700
Wire Wire Line
	9450 1200 9900 1200
Wire Wire Line
	9450 2900 9950 2900
$Comp
L GND #PWR6
U 1 1 59AEA220
P 9450 4100
F 0 "#PWR6" H 9450 3850 50  0001 C CNN
F 1 "GND" H 9450 3950 50  0000 C CNN
F 2 "" H 9450 4100 50  0000 C CNN
F 3 "" H 9450 4100 50  0000 C CNN
	1    9450 4100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR5
U 1 1 59AEA23A
P 9450 2400
F 0 "#PWR5" H 9450 2150 50  0001 C CNN
F 1 "GND" H 9450 2250 50  0000 C CNN
F 2 "" H 9450 2400 50  0000 C CNN
F 3 "" H 9450 2400 50  0000 C CNN
	1    9450 2400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR4
U 1 1 59AEA257
P 7950 2100
F 0 "#PWR4" H 7950 1850 50  0001 C CNN
F 1 "GND" H 7950 1950 50  0000 C CNN
F 2 "" H 7950 2100 50  0000 C CNN
F 3 "" H 7950 2100 50  0000 C CNN
	1    7950 2100
	0    1    1    0   
$EndComp
$Comp
L GND #PWR3
U 1 1 59AEA277
P 7700 3800
F 0 "#PWR3" H 7700 3550 50  0001 C CNN
F 1 "GND" H 7700 3650 50  0000 C CNN
F 2 "" H 7700 3800 50  0000 C CNN
F 3 "" H 7700 3800 50  0000 C CNN
	1    7700 3800
	0    1    1    0   
$EndComp
Wire Wire Line
	7700 3800 8050 3800
Wire Wire Line
	7950 2100 8050 2100
Wire Wire Line
	9450 2300 9450 2400
Wire Wire Line
	9450 4000 9450 4100
Text Notes 9700 2550 0    60   ~ 0
to arduino pins 2,3 and 4
Text Notes 9600 1150 0    60   ~ 0
to Arduino pin A0\n
Text Notes 9750 3050 0    60   ~ 0
to Arduino pin A1
Text Notes 5550 2000 0    60   ~ 0
16 mutiplexed \nanalog inputs
$Comp
L LED D1
U 1 1 59AEA84E
P 1750 2550
F 0 "D1" H 1750 2650 50  0000 C CNN
F 1 "LED" H 1750 2450 50  0000 C CNN
F 2 "" H 1750 2550 50  0000 C CNN
F 3 "" H 1750 2550 50  0000 C CNN
	1    1750 2550
	0    -1   -1   0   
$EndComp
$Comp
L Photores R2
U 1 1 59AEA92B
P 2550 2550
F 0 "R2" V 2630 2550 50  0000 C CNN
F 1 "Photores" V 2760 2550 50  0000 C TNN
F 2 "" V 2480 2550 50  0000 C CNN
F 3 "" H 2550 2550 50  0000 C CNN
	1    2550 2550
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 59AEA972
P 1750 1850
F 0 "R1" V 1830 1850 50  0000 C CNN
F 1 "R" V 1750 1850 50  0000 C CNN
F 2 "" V 1680 1850 50  0000 C CNN
F 3 "" H 1750 1850 50  0000 C CNN
	1    1750 1850
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 59AEA9CD
P 2550 3350
F 0 "R3" V 2630 3350 50  0000 C CNN
F 1 "4.7k" V 2550 3350 50  0000 C CNN
F 2 "" V 2480 3350 50  0000 C CNN
F 3 "" H 2550 3350 50  0000 C CNN
	1    2550 3350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR2
U 1 1 59AEAA10
P 2100 3850
F 0 "#PWR2" H 2100 3600 50  0001 C CNN
F 1 "GND" H 2100 3700 50  0000 C CNN
F 2 "" H 2100 3850 50  0000 C CNN
F 3 "" H 2100 3850 50  0000 C CNN
	1    2100 3850
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR1
U 1 1 59AEAA32
P 2100 1300
F 0 "#PWR1" H 2100 1150 50  0001 C CNN
F 1 "+5V" H 2100 1440 50  0000 C CNN
F 2 "" H 2100 1300 50  0000 C CNN
F 3 "" H 2100 1300 50  0000 C CNN
	1    2100 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	2100 1300 2100 1500
Wire Wire Line
	1750 1500 2550 1500
Wire Wire Line
	1750 1500 1750 1700
Wire Wire Line
	2550 1500 2550 2300
Connection ~ 2100 1500
Wire Wire Line
	2550 2800 2550 3200
Wire Wire Line
	2550 3750 2550 3500
Wire Wire Line
	1750 3750 2550 3750
Wire Wire Line
	2100 3750 2100 3850
Wire Wire Line
	1750 2750 1750 3750
Connection ~ 2100 3750
Wire Wire Line
	1750 2350 1750 2000
Wire Wire Line
	1750 2200 3700 2200
Connection ~ 1750 2200
Wire Wire Line
	2550 3000 3700 3000
Connection ~ 2550 3000
Text Notes 3050 2100 0    60   ~ 0
to ADC to measure voltage drop \nacross the LED as well as the current \ngoing through the LED (given\nthe value of R)
Text Notes 3100 2900 0    60   ~ 0
to ADC to measure the light\noutput of the LED
Wire Notes Line
	1150 950  1150 4400
Wire Notes Line
	1150 4400 5000 4400
Wire Notes Line
	5000 4400 5000 950 
Wire Notes Line
	5000 950  1150 950 
Wire Notes Line
	5500 950  5500 4400
Wire Notes Line
	5500 4400 11050 4400
Wire Notes Line
	11050 4400 11050 950 
Wire Notes Line
	11050 950  5500 950 
Text Notes 1350 4600 0    60   ~ 0
LED Testing Circuit. Built 6 times with different LEDs and values for R.
Text Notes 6400 4650 0    60   ~ 0
Multiplexer for the Arduino ADC. Allows 16 analog inputs using 2 analog pins\nfrom the arduino and 3 digital pins.
$EndSCHEMATC
