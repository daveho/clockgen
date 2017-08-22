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
LIBS:Symbols_DCDC-ACDC-Converter_RevC_20Jul2012
LIBS:ttl_ieee
LIBS:ya68k
LIBS:clockgen-cache
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
L 74LS590 U?
U 1 1 599C3081
P 4750 2050
F 0 "U?" H 5050 2850 50  0000 C CNN
F 1 "74LS590" H 5000 1300 50  0000 C CNN
F 2 "" H 4750 2050 60  0000 C CNN
F 3 "" H 4750 2050 60  0000 C CNN
	1    4750 2050
	1    0    0    -1  
$EndComp
$Comp
L 74LS151 U?
U 1 1 599C30EA
P 6750 2650
F 0 "U?" H 6750 2650 50  0000 C CNN
F 1 "74LS151" H 6750 2500 50  0000 C CNN
F 2 "" H 6750 2650 50  0000 C CNN
F 3 "" H 6750 2650 50  0000 C CNN
	1    6750 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 599C31CA
P 6450 3500
F 0 "#PWR?" H 6450 3250 50  0001 C CNN
F 1 "GND" H 6450 3350 50  0000 C CNN
F 2 "" H 6450 3500 50  0000 C CNN
F 3 "" H 6450 3500 50  0000 C CNN
	1    6450 3500
	1    0    0    -1  
$EndComp
$Comp
L XTAL_OSC_HALF_CAN U?
U 1 1 599C32CC
P 2800 1900
F 0 "U?" H 2800 2200 60  0000 C CNN
F 1 "XTAL_OSC_HALF_CAN" H 2800 1600 60  0000 C CNN
F 2 "" H 2800 1900 60  0000 C CNN
F 3 "" H 2800 1900 60  0000 C CNN
	1    2800 1900
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 599C33B7
P 2000 1450
F 0 "#PWR?" H 2000 1300 50  0001 C CNN
F 1 "VCC" H 2000 1600 50  0000 C CNN
F 2 "" H 2000 1450 50  0000 C CNN
F 3 "" H 2000 1450 50  0000 C CNN
	1    2000 1450
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 P?
U 1 1 599C3D2F
P 8450 5400
F 0 "P?" H 8450 5650 50  0000 C CNN
F 1 "CONN_01X04" V 8550 5400 50  0000 C CNN
F 2 "" H 8450 5400 50  0000 C CNN
F 3 "" H 8450 5400 50  0000 C CNN
	1    8450 5400
	1    0    0    -1  
$EndComp
Text Notes 7950 5050 0    60   ~ 0
OLED display
Text Label 8050 5450 0    60   ~ 0
SCL
Text Label 8050 5550 0    60   ~ 0
SDA
$Comp
L VCC #PWR?
U 1 1 599C3EA4
P 7750 5100
F 0 "#PWR?" H 7750 4950 50  0001 C CNN
F 1 "VCC" H 7750 5250 50  0000 C CNN
F 2 "" H 7750 5100 50  0000 C CNN
F 3 "" H 7750 5100 50  0000 C CNN
	1    7750 5100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 599C3EC2
P 7750 5450
F 0 "#PWR?" H 7750 5200 50  0001 C CNN
F 1 "GND" H 7750 5300 50  0000 C CNN
F 2 "" H 7750 5450 50  0000 C CNN
F 3 "" H 7750 5450 50  0000 C CNN
	1    7750 5450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 599C40DB
P 3900 1950
F 0 "#PWR?" H 3900 1700 50  0001 C CNN
F 1 "GND" H 3900 1800 50  0000 C CNN
F 2 "" H 3900 1950 50  0000 C CNN
F 3 "" H 3900 1950 50  0000 C CNN
	1    3900 1950
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 599C413A
P 4250 5650
F 0 "SW?" H 4400 5760 50  0000 C CNN
F 1 "SW_PUSH" H 4250 5570 50  0000 C CNN
F 2 "" H 4250 5650 50  0000 C CNN
F 3 "" H 4250 5650 50  0000 C CNN
	1    4250 5650
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 599C41F1
P 4550 5650
F 0 "SW?" H 4700 5760 50  0000 C CNN
F 1 "SW_PUSH" H 4550 5570 50  0000 C CNN
F 2 "" H 4550 5650 50  0000 C CNN
F 3 "" H 4550 5650 50  0000 C CNN
	1    4550 5650
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 599C426D
P 4850 5650
F 0 "SW?" H 5000 5760 50  0000 C CNN
F 1 "SW_PUSH" H 4850 5570 50  0000 C CNN
F 2 "" H 4850 5650 50  0000 C CNN
F 3 "" H 4850 5650 50  0000 C CNN
	1    4850 5650
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 599C4273
P 5150 5650
F 0 "SW?" H 5300 5760 50  0000 C CNN
F 1 "SW_PUSH" H 5150 5570 50  0000 C CNN
F 2 "" H 5150 5650 50  0000 C CNN
F 3 "" H 5150 5650 50  0000 C CNN
	1    5150 5650
	0    1    1    0   
$EndComp
$Comp
L SW_PUSH SW?
U 1 1 599C465A
P 5400 5650
F 0 "SW?" H 5550 5760 50  0000 C CNN
F 1 "SW_PUSH" H 5400 5570 50  0000 C CNN
F 2 "" H 5400 5650 50  0000 C CNN
F 3 "" H 5400 5650 50  0000 C CNN
	1    5400 5650
	0    1    1    0   
$EndComp
$Comp
L 74LS175 U?
U 1 1 599C4EB5
P 9800 2700
F 0 "U?" H 9800 2700 50  0000 C CNN
F 1 "74LS175" H 9900 2300 50  0000 C CNN
F 2 "" H 9800 2700 50  0000 C CNN
F 3 "" H 9800 2700 50  0000 C CNN
	1    9800 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 599C5033
P 4850 6350
F 0 "#PWR?" H 4850 6100 50  0001 C CNN
F 1 "GND" H 4850 6200 50  0000 C CNN
F 2 "" H 4850 6350 50  0000 C CNN
F 3 "" H 4850 6350 50  0000 C CNN
	1    4850 6350
	1    0    0    -1  
$EndComp
Text Label 3500 1750 0    60   ~ 0
CLK
Text Label 8600 3050 0    60   ~ 0
CLK
$Comp
L GND #PWR?
U 1 1 599C52D5
P 2100 2300
F 0 "#PWR?" H 2100 2050 50  0001 C CNN
F 1 "GND" H 2100 2150 50  0000 C CNN
F 2 "" H 2100 2300 50  0000 C CNN
F 3 "" H 2100 2300 50  0000 C CNN
	1    2100 2300
	1    0    0    -1  
$EndComp
Text Label 5100 6650 0    60   ~ 0
CDIV0
Text Label 5100 6750 0    60   ~ 0
CDIV1
Text Label 5100 6850 0    60   ~ 0
CDIV2
Text Label 5800 2900 0    60   ~ 0
CDIV0
Text Label 5800 3000 0    60   ~ 0
CDIV1
Text Label 5800 3100 0    60   ~ 0
CDIV2
Entry Wire Line
	5550 3000 5650 2900
Entry Wire Line
	5550 3100 5650 3000
Entry Wire Line
	5550 3200 5650 3100
Entry Wire Line
	5450 6650 5550 6550
Entry Wire Line
	5450 6750 5550 6650
Entry Wire Line
	5450 6850 5550 6750
Text Label 10650 2250 0    60   ~ 0
FASTCLK
$Comp
L 74LS02 U?
U 1 1 599C5A4B
P 8350 2250
F 0 "U?" H 8350 2300 50  0000 C CNN
F 1 "74LS02" H 8400 2200 50  0000 C CNN
F 2 "" H 8350 2250 50  0000 C CNN
F 3 "" H 8350 2250 50  0000 C CNN
	1    8350 2250
	1    0    0    -1  
$EndComp
Text Label 3600 3900 0    60   ~ 0
-FASTEN
Text Label 3000 5700 0    60   ~ 0
RST
Text Label 3000 5800 0    60   ~ 0
-RST
$Comp
L 74LS02 U?
U 4 1 599C659F
P 8750 1000
F 0 "U?" H 8750 1050 50  0000 C CNN
F 1 "74LS02" H 8800 950 50  0000 C CNN
F 2 "" H 8750 1000 50  0000 C CNN
F 3 "" H 8750 1000 50  0000 C CNN
	4    8750 1000
	1    0    0    -1  
$EndComp
$Comp
L 74LS02 U?
U 3 1 599C66D6
P 10050 1400
F 0 "U?" H 10050 1450 50  0000 C CNN
F 1 "74LS02" H 10100 1350 50  0000 C CNN
F 2 "" H 10050 1400 50  0000 C CNN
F 3 "" H 10050 1400 50  0000 C CNN
	3    10050 1400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 599C6AB2
P 9400 1700
F 0 "#PWR?" H 9400 1450 50  0001 C CNN
F 1 "GND" H 9400 1550 50  0000 C CNN
F 2 "" H 9400 1700 50  0000 C CNN
F 3 "" H 9400 1700 50  0000 C CNN
	1    9400 1700
	1    0    0    -1  
$EndComp
Text Label 10700 1000 0    60   ~ 0
CLKOUT
Text Label 10700 1400 0    60   ~ 0
-CLKOUT
Text Label 7650 900  0    60   ~ 0
FASTCLK
Text Label 7650 1100 0    60   ~ 0
SLOWCLK
Text Label 5050 6950 0    60   ~ 0
-FASTEN
Text Label 5050 7050 0    60   ~ 0
-CTCLR
Text Label 3600 3750 0    60   ~ 0
-CTCLR
Text Label 5100 7150 0    60   ~ 0
SLOWCLK
$Comp
L GND #PWR?
U 1 1 599C87C9
P 8950 3300
F 0 "#PWR?" H 8950 3050 50  0001 C CNN
F 1 "GND" H 8950 3150 50  0000 C CNN
F 2 "" H 8950 3300 50  0000 C CNN
F 3 "" H 8950 3300 50  0000 C CNN
	1    8950 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 2000 6050 2000
Wire Wire Line
	5300 2100 6050 2100
Wire Wire Line
	5300 2200 6050 2200
Wire Wire Line
	5300 2300 6050 2300
Wire Wire Line
	5300 2400 6050 2400
Wire Wire Line
	5300 2500 6050 2500
Wire Wire Line
	5300 2600 6050 2600
Wire Wire Line
	5300 2700 6050 2700
Wire Wire Line
	6450 3350 6450 3500
Wire Wire Line
	2000 2050 2150 2050
Wire Wire Line
	2000 1450 2000 2050
Wire Wire Line
	8250 5450 7950 5450
Wire Wire Line
	7950 5550 8250 5550
Wire Wire Line
	8250 5250 7750 5250
Wire Wire Line
	7750 5250 7750 5100
Wire Wire Line
	8250 5350 7750 5350
Wire Wire Line
	7750 5350 7750 5450
Wire Wire Line
	3450 1750 4200 1750
Wire Wire Line
	4200 1500 3800 1500
Wire Wire Line
	3800 1500 3800 1750
Connection ~ 3800 1750
Wire Wire Line
	4200 1650 3900 1650
Wire Wire Line
	3900 1400 3900 1950
Wire Wire Line
	4200 1400 3900 1400
Connection ~ 3900 1650
Wire Wire Line
	4250 5950 4250 6200
Wire Wire Line
	4250 6200 5400 6200
Wire Wire Line
	4850 5950 4850 6350
Wire Wire Line
	5400 6200 5400 5950
Connection ~ 4850 6200
Wire Wire Line
	4550 5950 4550 6200
Connection ~ 4550 6200
Wire Wire Line
	5150 5950 5150 6200
Connection ~ 5150 6200
Wire Wire Line
	4250 5350 4250 5250
Wire Wire Line
	4250 5250 2900 5250
Wire Wire Line
	2900 5150 4550 5150
Wire Wire Line
	4550 5150 4550 5350
Wire Wire Line
	2900 5050 4850 5050
Wire Wire Line
	4850 5050 4850 5350
Wire Wire Line
	2900 4950 5150 4950
Wire Wire Line
	5150 4950 5150 5350
Wire Wire Line
	2900 4850 5400 4850
Wire Wire Line
	5400 4850 5400 5350
Wire Wire Line
	10500 2250 11150 2250
Wire Wire Line
	2150 1750 2000 1750
Connection ~ 2000 1750
Wire Wire Line
	2150 1850 2100 1850
Wire Wire Line
	2100 1850 2100 2300
Wire Wire Line
	2900 6650 5450 6650
Wire Wire Line
	2900 6750 5450 6750
Wire Wire Line
	2900 6850 5450 6850
Wire Bus Line
	5550 3000 5550 6750
Wire Wire Line
	6050 2900 5650 2900
Wire Wire Line
	6050 3000 5650 3000
Wire Wire Line
	6050 3100 5650 3100
Wire Wire Line
	7450 2150 7750 2150
Wire Wire Line
	8950 2250 9100 2250
Wire Wire Line
	7750 2350 7600 2350
Wire Wire Line
	7600 2350 7600 3900
Wire Wire Line
	7600 3900 3550 3900
Wire Wire Line
	6050 3300 5900 3300
Wire Wire Line
	5900 3300 5900 3900
Connection ~ 5900 3900
Wire Wire Line
	3400 5700 2900 5700
Wire Wire Line
	3400 5800 2900 5800
Wire Wire Line
	8150 900  7600 900 
Wire Wire Line
	8150 1100 7600 1100
Wire Wire Line
	9350 1000 10950 1000
Wire Wire Line
	9450 1300 9400 1300
Wire Wire Line
	9400 1300 9400 1000
Connection ~ 9400 1000
Wire Wire Line
	9450 1500 9400 1500
Wire Wire Line
	9400 1500 9400 1700
Wire Wire Line
	10650 1400 10950 1400
Wire Wire Line
	2900 6950 5500 6950
Wire Wire Line
	2900 7050 5500 7050
Wire Wire Line
	2900 7150 5500 7150
Wire Wire Line
	4200 1850 4200 3750
Wire Wire Line
	3550 3750 8600 3750
Wire Wire Line
	8600 3750 8600 3150
Connection ~ 4200 3750
Wire Wire Line
	9100 2450 8950 2450
Wire Wire Line
	8950 2450 8950 3300
Wire Wire Line
	9100 2650 8950 2650
Connection ~ 8950 2650
Wire Wire Line
	9100 2850 8950 2850
Connection ~ 8950 2850
Wire Wire Line
	9100 3050 8550 3050
Wire Wire Line
	8600 3150 9100 3150
Wire Wire Line
	2900 5450 3400 5450
Wire Wire Line
	2900 5550 3400 5550
Text Label 3050 5450 0    60   ~ 0
XTAL1
Text Label 3050 5550 0    60   ~ 0
XTAL2
Wire Wire Line
	2900 6300 3400 6300
Text Label 3050 6300 0    60   ~ 0
AVRRST
Wire Wire Line
	2900 6450 3400 6450
Wire Wire Line
	2900 6550 3400 6550
Text Label 3200 6450 0    60   ~ 0
RX
Text Label 3200 6550 0    60   ~ 0
TX
$Comp
L LED D?
U 1 1 599D23F7
P 3150 4400
F 0 "D?" H 3150 4500 50  0000 C CNN
F 1 "LED" H 3150 4300 50  0000 C CNN
F 2 "" H 3150 4400 50  0001 C CNN
F 3 "" H 3150 4400 50  0001 C CNN
	1    3150 4400
	0    -1   -1   0   
$EndComp
$Comp
L R R?
U 1 1 599D24C7
P 3150 4000
F 0 "R?" V 3230 4000 50  0000 C CNN
F 1 "R" V 3150 4000 50  0000 C CNN
F 2 "" V 3080 4000 50  0001 C CNN
F 3 "" H 3150 4000 50  0001 C CNN
	1    3150 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 4250 3150 4150
Wire Wire Line
	3150 4550 3150 5350
Wire Wire Line
	3150 3850 3150 3750
$Comp
L VCC #PWR?
U 1 1 599D26C5
P 3150 3750
F 0 "#PWR?" H 3150 3600 50  0001 C CNN
F 1 "VCC" H 3150 3900 50  0000 C CNN
F 2 "" H 3150 3750 50  0001 C CNN
F 3 "" H 3150 3750 50  0001 C CNN
	1    3150 3750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 6200 3900 6200
Wire Wire Line
	2900 6100 3700 6100
Text Label 3150 6100 0    60   ~ 0
SDA
Text Notes 9200 4600 0    60   ~ 0
Reset button
Text Notes 6550 5050 0    60   ~ 0
Crystal osc
Text Label 8950 5250 0    60   ~ 0
AVRRST
Text Label 6000 5350 0    60   ~ 0
XTAL2
Text Label 6000 5250 0    60   ~ 0
XTAL1
$Comp
L VCC #PWR?
U 1 1 599D0A6B
P 9450 4850
F 0 "#PWR?" H 9450 4700 50  0001 C CNN
F 1 "VCC" H 9450 5000 50  0000 C CNN
F 2 "" H 9450 4850 50  0001 C CNN
F 3 "" H 9450 4850 50  0001 C CNN
	1    9450 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 4900 9450 4850
Wire Wire Line
	9450 5250 9450 5200
$Comp
L R R?
U 1 1 599D0902
P 9450 5050
F 0 "R?" V 9530 5050 50  0000 C CNN
F 1 "R" V 9450 5050 50  0000 C CNN
F 2 "" V 9380 5050 50  0001 C CNN
F 3 "" H 9450 5050 50  0001 C CNN
	1    9450 5050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 599D0741
P 9450 5950
F 0 "#PWR?" H 9450 5700 50  0001 C CNN
F 1 "GND" H 9450 5800 50  0000 C CNN
F 2 "" H 9450 5950 50  0001 C CNN
F 3 "" H 9450 5950 50  0001 C CNN
	1    9450 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9450 5850 9450 5950
Wire Wire Line
	8900 5250 9450 5250
$Comp
L SW_PUSH SW?
U 1 1 599D0583
P 9450 5550
F 0 "SW?" H 9600 5660 50  0000 C CNN
F 1 "SW_PUSH" H 9450 5470 50  0000 C CNN
F 2 "" H 9450 5550 50  0000 C CNN
F 3 "" H 9450 5550 50  0000 C CNN
	1    9450 5550
	0    1    1    0   
$EndComp
Connection ~ 7450 5550
Wire Wire Line
	7300 5550 7450 5550
Wire Wire Line
	7450 5250 7450 5700
Wire Wire Line
	7300 5250 7450 5250
Connection ~ 6750 5550
Connection ~ 6750 5250
Wire Wire Line
	6500 5550 7000 5550
Wire Wire Line
	6500 5350 6500 5550
Wire Wire Line
	5950 5350 6500 5350
Wire Wire Line
	5950 5250 7000 5250
$Comp
L GND #PWR?
U 1 1 599CCD88
P 7450 5700
F 0 "#PWR?" H 7450 5450 50  0001 C CNN
F 1 "GND" H 7450 5550 50  0000 C CNN
F 2 "" H 7450 5700 50  0001 C CNN
F 3 "" H 7450 5700 50  0001 C CNN
	1    7450 5700
	1    0    0    -1  
$EndComp
$Comp
L C C?
U 1 1 599CCB90
P 7150 5550
F 0 "C?" H 7175 5650 50  0000 L CNN
F 1 "C" H 7175 5450 50  0000 L CNN
F 2 "" H 7188 5400 50  0001 C CNN
F 3 "" H 7150 5550 50  0001 C CNN
	1    7150 5550
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 599CCAB2
P 7150 5250
F 0 "C?" H 7175 5350 50  0000 L CNN
F 1 "C" H 7175 5150 50  0000 L CNN
F 2 "" H 7188 5100 50  0001 C CNN
F 3 "" H 7150 5250 50  0001 C CNN
	1    7150 5250
	0    1    1    0   
$EndComp
$Comp
L Crystal Y?
U 1 1 599CC479
P 6750 5400
F 0 "Y?" H 6750 5550 50  0000 C CNN
F 1 "Crystal" H 6750 5250 50  0000 C CNN
F 2 "" H 6750 5400 50  0001 C CNN
F 3 "" H 6750 5400 50  0001 C CNN
	1    6750 5400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 599D0367
P 800 7300
F 0 "#PWR?" H 800 7050 50  0001 C CNN
F 1 "GND" H 800 7150 50  0000 C CNN
F 2 "" H 800 7300 50  0001 C CNN
F 3 "" H 800 7300 50  0001 C CNN
	1    800  7300
	1    0    0    -1  
$EndComp
Connection ~ 800  7150
Wire Wire Line
	1000 7150 800  7150
Wire Wire Line
	800  7050 800  7300
Wire Wire Line
	1000 7050 800  7050
$Comp
L VCC #PWR?
U 1 1 599CFF9E
P 800 4700
F 0 "#PWR?" H 800 4550 50  0001 C CNN
F 1 "VCC" H 800 4850 50  0000 C CNN
F 2 "" H 800 4700 50  0001 C CNN
F 3 "" H 800 4700 50  0001 C CNN
	1    800  4700
	1    0    0    -1  
$EndComp
Connection ~ 800  5150
Wire Wire Line
	800  5150 1000 5150
Connection ~ 800  4850
Wire Wire Line
	1000 4850 800  4850
Wire Wire Line
	800  4700 800  5450
Wire Wire Line
	800  5450 1000 5450
Text Label 3150 6200 0    60   ~ 0
SCL
$Comp
L ATMEGA328-PU U?
U 1 1 599CC266
P 1900 5950
F 0 "U?" H 1150 7200 50  0000 L BNN
F 1 "ATMEGA328-PU" H 2300 4550 50  0000 L BNN
F 2 "DIL28" H 1900 5950 50  0001 C CIN
F 3 "" H 1900 5950 50  0001 C CNN
	1    1900 5950
	1    0    0    -1  
$EndComp
Text Notes 3450 6550 0    60   ~ 0
RX/TX go to FT232\nboard (USB serial)
Wire Wire Line
	3150 5350 2900 5350
Text Notes 7100 6800 0    60   ~ 0
flexible clock generator
$Comp
L R R?
U 1 1 599D04B4
P 3700 5850
F 0 "R?" V 3780 5850 50  0000 C CNN
F 1 "R" V 3700 5850 50  0000 C CNN
F 2 "" V 3630 5850 50  0001 C CNN
F 3 "" H 3700 5850 50  0001 C CNN
	1    3700 5850
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 599D0605
P 3900 5850
F 0 "R?" V 3980 5850 50  0000 C CNN
F 1 "R" V 3900 5850 50  0000 C CNN
F 2 "" V 3830 5850 50  0001 C CNN
F 3 "" H 3900 5850 50  0001 C CNN
	1    3900 5850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5700 3700 5600
Wire Wire Line
	3900 5700 3900 5600
Wire Wire Line
	3700 6100 3700 6000
Wire Wire Line
	3900 6200 3900 6000
$Comp
L VCC #PWR?
U 1 1 599D08A0
P 3700 5600
F 0 "#PWR?" H 3700 5450 50  0001 C CNN
F 1 "VCC" H 3700 5750 50  0000 C CNN
F 2 "" H 3700 5600 50  0001 C CNN
F 3 "" H 3700 5600 50  0001 C CNN
	1    3700 5600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 599D08E2
P 3900 5600
F 0 "#PWR?" H 3900 5450 50  0001 C CNN
F 1 "VCC" H 3900 5750 50  0000 C CNN
F 2 "" H 3900 5600 50  0001 C CNN
F 3 "" H 3900 5600 50  0001 C CNN
	1    3900 5600
	1    0    0    -1  
$EndComp
$Comp
L 74LS02 U?
U 2 1 599D0A9C
P 10550 5300
F 0 "U?" H 10550 5350 50  0000 C CNN
F 1 "74LS02" H 10600 5250 50  0000 C CNN
F 2 "" H 10550 5300 50  0001 C CNN
F 3 "" H 10550 5300 50  0001 C CNN
	2    10550 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 5200 9800 5200
Wire Wire Line
	9800 5200 9800 5550
Wire Wire Line
	9950 5400 9800 5400
Connection ~ 9800 5400
$Comp
L GND #PWR?
U 1 1 599D0E91
P 9800 5550
F 0 "#PWR?" H 9800 5300 50  0001 C CNN
F 1 "GND" H 9800 5400 50  0000 C CNN
F 2 "" H 9800 5550 50  0001 C CNN
F 3 "" H 9800 5550 50  0001 C CNN
	1    9800 5550
	1    0    0    -1  
$EndComp
Text Notes 10150 4950 0    60   ~ 0
unused gate
$EndSCHEMATC
