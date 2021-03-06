EESchema Schematic File Version 2
LIBS:uber-rescue
LIBS:boosterpack
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
LIBS:stellaris
LIBS:uber-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date "19 oct 2012"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L +5V #PWR01
U 1 1 5080DC8B
P 9500 4600
F 0 "#PWR01" H 9500 4690 20  0001 C CNN
F 1 "+5V" H 9500 4690 30  0000 C CNN
F 2 "" H 9500 4600 60  0001 C CNN
F 3 "" H 9500 4600 60  0001 C CNN
	1    9500 4600
	0    -1   -1   0   
$EndComp
$Comp
L CONN_02X04 NRF24_CONN1
U 1 1 570E2A08
P 6300 2150
F 0 "NRF24_CONN1" H 6300 2400 50  0000 C CNN
F 1 "CONN_02X04" H 6300 1900 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x04" H 6300 950 50  0001 C CNN
F 3 "" H 6300 950 50  0000 C CNN
	1    6300 2150
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X12 BUT_CONN1
U 1 1 570E3921
P 4600 3450
F 0 "BUT_CONN1" H 4600 4100 50  0000 C CNN
F 1 "CONN_02X12" V 4600 3450 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x12" H 4600 2250 50  0001 C CNN
F 3 "" H 4600 2250 50  0000 C CNN
	1    4600 3450
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR02
U 1 1 570E3F16
P 4250 2900
F 0 "#PWR02" H 4250 2750 50  0001 C CNN
F 1 "+3.3V" H 4250 3040 50  0000 C CNN
F 2 "" H 4250 2900 50  0000 C CNN
F 3 "" H 4250 2900 50  0000 C CNN
	1    4250 2900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR03
U 1 1 570E40C8
P 6750 2000
F 0 "#PWR03" H 6750 1850 50  0001 C CNN
F 1 "+3.3V" H 6750 2140 50  0000 C CNN
F 2 "" H 6750 2000 50  0000 C CNN
F 3 "" H 6750 2000 50  0000 C CNN
	1    6750 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2000 6750 2000
Wire Wire Line
	4250 2900 4350 2900
$Comp
L +3.3V #PWR04
U 1 1 570E4231
P 3400 3200
F 0 "#PWR04" H 3400 3050 50  0001 C CNN
F 1 "+3.3V" H 3400 3340 50  0000 C CNN
F 2 "" H 3400 3200 50  0000 C CNN
F 3 "" H 3400 3200 50  0000 C CNN
	1    3400 3200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR05
U 1 1 570E4253
P 3400 3500
F 0 "#PWR05" H 3400 3350 50  0001 C CNN
F 1 "+3.3V" H 3400 3640 50  0000 C CNN
F 2 "" H 3400 3500 50  0000 C CNN
F 3 "" H 3400 3500 50  0000 C CNN
	1    3400 3500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR06
U 1 1 570E4293
P 3400 3800
F 0 "#PWR06" H 3400 3650 50  0001 C CNN
F 1 "+3.3V" H 3400 3940 50  0000 C CNN
F 2 "" H 3400 3800 50  0000 C CNN
F 3 "" H 3400 3800 50  0000 C CNN
	1    3400 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 3200 3400 3200
Wire Wire Line
	4350 3800 3400 3800
$Comp
L +3.3V #PWR07
U 1 1 570E4337
P 4950 2900
F 0 "#PWR07" H 4950 2750 50  0001 C CNN
F 1 "+3.3V" H 4950 3040 50  0000 C CNN
F 2 "" H 4950 2900 50  0000 C CNN
F 3 "" H 4950 2900 50  0000 C CNN
	1    4950 2900
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR08
U 1 1 570E4359
P 5650 3200
F 0 "#PWR08" H 5650 3050 50  0001 C CNN
F 1 "+3.3V" H 5650 3340 50  0000 C CNN
F 2 "" H 5650 3200 50  0000 C CNN
F 3 "" H 5650 3200 50  0000 C CNN
	1    5650 3200
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR09
U 1 1 570E437B
P 6000 3500
F 0 "#PWR09" H 6000 3350 50  0001 C CNN
F 1 "+3.3V" H 6000 3640 50  0000 C CNN
F 2 "" H 6000 3500 50  0000 C CNN
F 3 "" H 6000 3500 50  0000 C CNN
	1    6000 3500
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR010
U 1 1 570E439D
P 6000 3800
F 0 "#PWR010" H 6000 3650 50  0001 C CNN
F 1 "+3.3V" H 6000 3940 50  0000 C CNN
F 2 "" H 6000 3800 50  0000 C CNN
F 3 "" H 6000 3800 50  0000 C CNN
	1    6000 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 2900 4950 2900
Wire Wire Line
	4850 3200 5650 3200
Wire Wire Line
	4850 3500 6000 3500
Wire Wire Line
	4850 3800 6000 3800
$Comp
L GND #PWR011
U 1 1 570E4763
P 10850 3450
F 0 "#PWR011" H 10850 3200 50  0001 C CNN
F 1 "GND" H 10850 3300 50  0000 C CNN
F 2 "" H 10850 3450 50  0000 C CNN
F 3 "" H 10850 3450 50  0000 C CNN
	1    10850 3450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR012
U 1 1 570E4803
P 5800 1700
F 0 "#PWR012" H 5800 1450 50  0001 C CNN
F 1 "GND" H 5800 1550 50  0000 C CNN
F 2 "" H 5800 1700 50  0000 C CNN
F 3 "" H 5800 1700 50  0000 C CNN
	1    5800 1700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR013
U 1 1 570E495F
P 5800 3000
F 0 "#PWR013" H 5800 2750 50  0001 C CNN
F 1 "GND" H 5800 2850 50  0000 C CNN
F 2 "" H 5800 3000 50  0000 C CNN
F 3 "" H 5800 3000 50  0000 C CNN
	1    5800 3000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR014
U 1 1 570E4981
P 5800 3300
F 0 "#PWR014" H 5800 3050 50  0001 C CNN
F 1 "GND" H 5800 3150 50  0000 C CNN
F 2 "" H 5800 3300 50  0000 C CNN
F 3 "" H 5800 3300 50  0000 C CNN
	1    5800 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 570E49A3
P 5800 3600
F 0 "#PWR015" H 5800 3350 50  0001 C CNN
F 1 "GND" H 5800 3450 50  0000 C CNN
F 2 "" H 5800 3600 50  0000 C CNN
F 3 "" H 5800 3600 50  0000 C CNN
	1    5800 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR016
U 1 1 570E49C5
P 5800 3900
F 0 "#PWR016" H 5800 3650 50  0001 C CNN
F 1 "GND" H 5800 3750 50  0000 C CNN
F 2 "" H 5800 3900 50  0000 C CNN
F 3 "" H 5800 3900 50  0000 C CNN
	1    5800 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 570E49E7
P 3550 3900
F 0 "#PWR017" H 3550 3650 50  0001 C CNN
F 1 "GND" H 3550 3750 50  0000 C CNN
F 2 "" H 3550 3900 50  0000 C CNN
F 3 "" H 3550 3900 50  0000 C CNN
	1    3550 3900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 570E4A09
P 3600 3600
F 0 "#PWR018" H 3600 3350 50  0001 C CNN
F 1 "GND" H 3600 3450 50  0000 C CNN
F 2 "" H 3600 3600 50  0000 C CNN
F 3 "" H 3600 3600 50  0000 C CNN
	1    3600 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 570E4A2B
P 3600 3300
F 0 "#PWR019" H 3600 3050 50  0001 C CNN
F 1 "GND" H 3600 3150 50  0000 C CNN
F 2 "" H 3600 3300 50  0000 C CNN
F 3 "" H 3600 3300 50  0000 C CNN
	1    3600 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 3600 4350 3600
Wire Wire Line
	3550 3900 4350 3900
Wire Wire Line
	4850 3900 5800 3900
Wire Wire Line
	4850 3600 5800 3600
Wire Wire Line
	4850 3300 5800 3300
Wire Wire Line
	5800 3000 4850 3000
Text GLabel 9500 5400 0    60   Input ~ 0
NRF24_CSN
Text GLabel 6550 2100 2    60   Input ~ 0
NRF24_CSN
Text GLabel 9500 4350 0    60   Input ~ 0
NRF24_IRQ
Text GLabel 6550 2300 2    60   Input ~ 0
NRF24_IRQ
Text GLabel 9500 4150 0    60   Input ~ 0
NRF24_MOSI
Text GLabel 7200 2200 2    60   Input ~ 0
NRF24_MOSI
Wire Wire Line
	6550 2200 7200 2200
Text GLabel 10850 4150 2    60   Input ~ 0
NRF24_MISO
Text GLabel 6050 2300 0    60   Input ~ 0
NRF24_MISO
Text GLabel 10850 4350 2    60   Input ~ 0
NRF24_CLK
Text GLabel 5350 2200 0    60   Input ~ 0
NRF24_CLK
Wire Wire Line
	5350 2200 6050 2200
Text GLabel 8800 4250 0    60   Input ~ 0
NRF24_CE
Wire Wire Line
	8800 4250 9500 4250
Text GLabel 6050 2100 0    60   Input ~ 0
NRF24_CE
Wire Wire Line
	6050 2000 6050 1700
Wire Wire Line
	6050 1700 5800 1700
$Comp
L R R10K14
U 1 1 5710E4C7
P 1950 3050
F 0 "R10K14" V 2030 3050 50  0000 C CNN
F 1 "R" V 1950 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1880 3050 50  0001 C CNN
F 3 "" H 1950 3050 50  0000 C CNN
	1    1950 3050
	1    0    0    -1  
$EndComp
$Comp
L R R10K15
U 1 1 5710E534
P 1950 3350
F 0 "R10K15" V 2030 3350 50  0000 C CNN
F 1 "R" V 1950 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1880 3350 50  0001 C CNN
F 3 "" H 1950 3350 50  0000 C CNN
	1    1950 3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K18
U 1 1 5710E570
P 2100 3600
F 0 "R10K18" V 2180 3600 50  0000 C CNN
F 1 "R" V 2100 3600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2030 3600 50  0001 C CNN
F 3 "" H 2100 3600 50  0000 C CNN
	1    2100 3600
	0    1    1    0   
$EndComp
$Comp
L R R10K21
U 1 1 5710E5FA
P 2400 3600
F 0 "R10K21" V 2480 3600 50  0000 C CNN
F 1 "R" V 2400 3600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2330 3600 50  0001 C CNN
F 3 "" H 2400 3600 50  0000 C CNN
	1    2400 3600
	0    1    1    0   
$EndComp
$Comp
L R R10K24
U 1 1 5710E648
P 2700 3600
F 0 "R10K24" V 2780 3600 50  0000 C CNN
F 1 "R" V 2700 3600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2630 3600 50  0001 C CNN
F 3 "" H 2700 3600 50  0000 C CNN
	1    2700 3600
	0    1    1    0   
$EndComp
$Comp
L R R10K20
U 1 1 5710E684
P 2250 3350
F 0 "R10K20" V 2330 3350 50  0000 C CNN
F 1 "R" V 2250 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2180 3350 50  0001 C CNN
F 3 "" H 2250 3350 50  0000 C CNN
	1    2250 3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K19
U 1 1 5710E6CD
P 2250 3050
F 0 "R10K19" V 2330 3050 50  0000 C CNN
F 1 "R" V 2250 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2180 3050 50  0001 C CNN
F 3 "" H 2250 3050 50  0000 C CNN
	1    2250 3050
	1    0    0    -1  
$EndComp
$Comp
L R R10K23
U 1 1 5710E715
P 2550 3350
F 0 "R10K23" V 2630 3350 50  0000 C CNN
F 1 "R" V 2550 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2480 3350 50  0001 C CNN
F 3 "" H 2550 3350 50  0000 C CNN
	1    2550 3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K22
U 1 1 5710E75A
P 2550 3050
F 0 "R10K22" V 2630 3050 50  0000 C CNN
F 1 "R" V 2550 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2480 3050 50  0001 C CNN
F 3 "" H 2550 3050 50  0000 C CNN
	1    2550 3050
	1    0    0    -1  
$EndComp
$Comp
L R R10K16
U 1 1 5710E99E
P 1950 3900
F 0 "R10K16" V 2030 3900 50  0000 C CNN
F 1 "R" V 1950 3900 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1880 3900 50  0001 C CNN
F 3 "" H 1950 3900 50  0000 C CNN
	1    1950 3900
	1    0    0    -1  
$EndComp
$Comp
L R R10K17
U 1 1 5710E9F3
P 1950 4200
F 0 "R10K17" V 2030 4200 50  0000 C CNN
F 1 "R" V 1950 4200 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1880 4200 50  0001 C CNN
F 3 "" H 1950 4200 50  0000 C CNN
	1    1950 4200
	1    0    0    -1  
$EndComp
$Comp
L R R10K26
U 1 1 5710EA6D
P 2850 3350
F 0 "R10K26" V 2930 3350 50  0000 C CNN
F 1 "R" V 2850 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2780 3350 50  0001 C CNN
F 3 "" H 2850 3350 50  0000 C CNN
	1    2850 3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K25
U 1 1 5710EAD1
P 2850 3050
F 0 "R10K25" V 2930 3050 50  0000 C CNN
F 1 "R" V 2850 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 2780 3050 50  0001 C CNN
F 3 "" H 2850 3050 50  0000 C CNN
	1    2850 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3500 1950 3750
Connection ~ 1950 3600
Wire Wire Line
	2250 3600 2250 3500
Wire Wire Line
	2550 3600 2550 3500
Wire Wire Line
	2850 3500 2850 3950
$Comp
L GND #PWR020
U 1 1 57111AE6
P 1950 4350
F 0 "#PWR020" H 1950 4100 50  0001 C CNN
F 1 "GND" H 1950 4200 50  0000 C CNN
F 2 "" H 1950 4350 50  0000 C CNN
F 3 "" H 1950 4350 50  0000 C CNN
	1    1950 4350
	1    0    0    -1  
$EndComp
Text GLabel 1950 2900 1    60   Input ~ 0
R2R_BUT5
Text GLabel 2250 2900 1    60   Input ~ 0
R2R_BUT6
Text GLabel 2550 2900 1    60   Input ~ 0
R2R_BUT7
Text GLabel 2850 2900 1    60   Input ~ 0
R2R_BUT8
Text GLabel 4200 3100 0    60   Input ~ 0
R2R_BUT1
$Comp
L GND #PWR021
U 1 1 571128FC
P 4100 2800
F 0 "#PWR021" H 4100 2550 50  0001 C CNN
F 1 "GND" H 4100 2650 50  0000 C CNN
F 2 "" H 4100 2800 50  0000 C CNN
F 3 "" H 4100 2800 50  0000 C CNN
	1    4100 2800
	-1   0    0    1   
$EndComp
Wire Wire Line
	4350 3000 4100 3000
Wire Wire Line
	4100 3000 4100 2800
Wire Wire Line
	4350 3100 4200 3100
Wire Wire Line
	3600 3300 4350 3300
Text GLabel 4200 3400 0    60   Input ~ 0
R2R_BUT2
Wire Wire Line
	4350 3400 4200 3400
Text GLabel 4200 3700 0    60   Input ~ 0
R2R_BUT3
Wire Wire Line
	4350 3700 4200 3700
Text GLabel 4200 4000 0    60   Input ~ 0
R2R_BUT4
Wire Wire Line
	4200 4000 4350 4000
Wire Wire Line
	4350 3500 3400 3500
Text GLabel 5050 3400 2    60   Input ~ 0
R2R_BUT6
Text GLabel 5050 3700 2    60   Input ~ 0
R2R_BUT7
Text GLabel 5050 4000 2    60   Input ~ 0
R2R_BUT8
Text GLabel 5050 3100 2    60   Input ~ 0
R2R_BUT5
Wire Wire Line
	4850 3100 5050 3100
Wire Wire Line
	4850 3400 5050 3400
Wire Wire Line
	4850 3700 5050 3700
Wire Wire Line
	4850 4000 5050 4000
$Comp
L R R10K1
U 1 1 57113C83
P 650 3050
F 0 "R10K1" V 730 3050 50  0000 C CNN
F 1 "R" V 650 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 580 3050 50  0001 C CNN
F 3 "" H 650 3050 50  0000 C CNN
	1    650  3050
	1    0    0    -1  
$EndComp
$Comp
L R R10K2
U 1 1 57113C89
P 650 3350
F 0 "R10K2" V 730 3350 50  0000 C CNN
F 1 "R" V 650 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 580 3350 50  0001 C CNN
F 3 "" H 650 3350 50  0000 C CNN
	1    650  3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K5
U 1 1 57113C8F
P 800 3600
F 0 "R10K5" V 880 3600 50  0000 C CNN
F 1 "R" V 800 3600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 730 3600 50  0001 C CNN
F 3 "" H 800 3600 50  0000 C CNN
	1    800  3600
	0    1    1    0   
$EndComp
$Comp
L R R10K8
U 1 1 57113C95
P 1100 3600
F 0 "R10K8" V 1180 3600 50  0000 C CNN
F 1 "R" V 1100 3600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1030 3600 50  0001 C CNN
F 3 "" H 1100 3600 50  0000 C CNN
	1    1100 3600
	0    1    1    0   
$EndComp
$Comp
L R R10K11
U 1 1 57113C9B
P 1400 3600
F 0 "R10K11" V 1480 3600 50  0000 C CNN
F 1 "R" V 1400 3600 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1330 3600 50  0001 C CNN
F 3 "" H 1400 3600 50  0000 C CNN
	1    1400 3600
	0    1    1    0   
$EndComp
$Comp
L R R10K7
U 1 1 57113CA1
P 950 3350
F 0 "R10K7" V 1030 3350 50  0000 C CNN
F 1 "R" V 950 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 880 3350 50  0001 C CNN
F 3 "" H 950 3350 50  0000 C CNN
	1    950  3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K6
U 1 1 57113CA7
P 950 3050
F 0 "R10K6" V 1030 3050 50  0000 C CNN
F 1 "R" V 950 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 880 3050 50  0001 C CNN
F 3 "" H 950 3050 50  0000 C CNN
	1    950  3050
	1    0    0    -1  
$EndComp
$Comp
L R R10K10
U 1 1 57113CAD
P 1250 3350
F 0 "R10K10" V 1330 3350 50  0000 C CNN
F 1 "R" V 1250 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1180 3350 50  0001 C CNN
F 3 "" H 1250 3350 50  0000 C CNN
	1    1250 3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K9
U 1 1 57113CB3
P 1250 3050
F 0 "R10K9" V 1330 3050 50  0000 C CNN
F 1 "R" V 1250 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1180 3050 50  0001 C CNN
F 3 "" H 1250 3050 50  0000 C CNN
	1    1250 3050
	1    0    0    -1  
$EndComp
$Comp
L R R10K3
U 1 1 57113CB9
P 650 3900
F 0 "R10K3" V 730 3900 50  0000 C CNN
F 1 "R" V 650 3900 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 580 3900 50  0001 C CNN
F 3 "" H 650 3900 50  0000 C CNN
	1    650  3900
	1    0    0    -1  
$EndComp
$Comp
L R R10K4
U 1 1 57113CBF
P 650 4200
F 0 "R10K4" V 730 4200 50  0000 C CNN
F 1 "R" V 650 4200 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 580 4200 50  0001 C CNN
F 3 "" H 650 4200 50  0000 C CNN
	1    650  4200
	1    0    0    -1  
$EndComp
$Comp
L R R10K13
U 1 1 57113CC5
P 1550 3350
F 0 "R10K13" V 1630 3350 50  0000 C CNN
F 1 "R" V 1550 3350 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1480 3350 50  0001 C CNN
F 3 "" H 1550 3350 50  0000 C CNN
	1    1550 3350
	1    0    0    -1  
$EndComp
$Comp
L R R10K12
U 1 1 57113CCB
P 1550 3050
F 0 "R10K12" V 1630 3050 50  0000 C CNN
F 1 "R" V 1550 3050 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM10mm" V 1480 3050 50  0001 C CNN
F 3 "" H 1550 3050 50  0000 C CNN
	1    1550 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	650  3500 650  3750
Connection ~ 650  3600
Wire Wire Line
	950  3600 950  3500
Wire Wire Line
	1250 3600 1250 3500
Wire Wire Line
	1550 3500 1550 3950
$Comp
L GND #PWR022
U 1 1 57113CD6
P 650 4350
F 0 "#PWR022" H 650 4100 50  0001 C CNN
F 1 "GND" H 650 4200 50  0000 C CNN
F 2 "" H 650 4350 50  0000 C CNN
F 3 "" H 650 4350 50  0000 C CNN
	1    650  4350
	1    0    0    -1  
$EndComp
Text GLabel 650  2900 1    60   Input ~ 0
R2R_BUT1
Text GLabel 950  2900 1    60   Input ~ 0
R2R_BUT2
Text GLabel 1250 2900 1    60   Input ~ 0
R2R_BUT3
Text GLabel 1550 2900 1    60   Input ~ 0
R2R_BUT4
$Comp
L CONN_01X05 ROT_CONN1
U 1 1 5714BA94
P 7150 5400
F 0 "ROT_CONN1" H 7150 5700 50  0000 C CNN
F 1 "CONN_01X05" V 7250 5400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 7150 5400 50  0001 C CNN
F 3 "" H 7150 5400 50  0000 C CNN
	1    7150 5400
	1    0    0    -1  
$EndComp
Text GLabel 6500 5600 0    60   Input ~ 0
ROT_BUT
Text GLabel 6500 5400 0    60   Input ~ 0
ROT_PIN_A
Text GLabel 6500 5200 0    60   Input ~ 0
ROT_PIN_B
$Comp
L GND #PWR023
U 1 1 5714C86B
P 6950 5600
F 0 "#PWR023" H 6950 5350 50  0001 C CNN
F 1 "GND" H 6950 5450 50  0000 C CNN
F 2 "" H 6950 5600 50  0000 C CNN
F 3 "" H 6950 5600 50  0000 C CNN
	1    6950 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 5714C8C3
P 6700 5300
F 0 "#PWR024" H 6700 5050 50  0001 C CNN
F 1 "GND" H 6700 5150 50  0000 C CNN
F 2 "" H 6700 5300 50  0000 C CNN
F 3 "" H 6700 5300 50  0000 C CNN
	1    6700 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 5500 6850 5500
Wire Wire Line
	6850 5500 6850 5600
Wire Wire Line
	6850 5600 6500 5600
Wire Wire Line
	6950 5200 6500 5200
Wire Wire Line
	6950 5300 6700 5300
Wire Wire Line
	6500 5400 6600 5400
Wire Wire Line
	6600 5400 6600 5500
Wire Wire Line
	6600 5500 6800 5500
Wire Wire Line
	6800 5500 6800 5400
Wire Wire Line
	6800 5400 6950 5400
Text GLabel 10850 5200 2    60   Input ~ 0
ROT_PIN_B
Text GLabel 11400 5300 2    60   Input ~ 0
ROT_PIN_A
Text GLabel 10850 5400 2    60   Input ~ 0
ROT_BUT
Wire Wire Line
	10850 5300 11400 5300
$Comp
L CONN_01X03 SERVO_CONN1
U 1 1 5714EA4B
P 6800 4550
F 0 "SERVO_CONN1" H 6800 4750 50  0000 C CNN
F 1 "CONN_01X03" V 6900 4550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 6800 4550 50  0001 C CNN
F 3 "" H 6800 4550 50  0000 C CNN
	1    6800 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR025
U 1 1 5714F4B3
P 6600 4650
F 0 "#PWR025" H 6600 4400 50  0001 C CNN
F 1 "GND" H 6600 4500 50  0000 C CNN
F 2 "" H 6600 4650 50  0000 C CNN
F 3 "" H 6600 4650 50  0000 C CNN
	1    6600 4650
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR026
U 1 1 5714F50D
P 6450 4550
F 0 "#PWR026" H 6450 4400 50  0001 C CNN
F 1 "+3.3V" H 6450 4690 50  0000 C CNN
F 2 "" H 6450 4550 50  0000 C CNN
F 3 "" H 6450 4550 50  0000 C CNN
	1    6450 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 4550 6600 4550
Text GLabel 6600 4250 0    60   Input ~ 0
SERVO_PWN
Wire Wire Line
	6600 4450 6600 4250
Text GLabel 8800 4050 0    60   Input ~ 0
SERVO_PWN
Wire Wire Line
	8800 4050 9500 4050
$Comp
L CONN_01X10 LCD_CONN1
U 1 1 57150BC5
P 5050 5800
F 0 "LCD_CONN1" H 5050 6350 50  0000 C CNN
F 1 "CONN_01X10" V 5150 5800 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x10" H 5050 5800 50  0001 C CNN
F 3 "" H 5050 5800 50  0000 C CNN
	1    5050 5800
	1    0    0    -1  
$EndComp
Text GLabel 4850 6250 0    60   Input ~ 0
LCD_PD3
Text GLabel 4350 6150 0    60   Input ~ 0
LCD_PD1
Wire Wire Line
	4350 6150 4850 6150
Text GLabel 4850 6050 0    60   Input ~ 0
LCD_PD2
Text GLabel 4350 5950 0    60   Input ~ 0
LCD_PD0
Wire Wire Line
	4350 5950 4850 5950
Text GLabel 4850 5850 0    60   Input ~ 0
LCD_PE4
$Comp
L GND #PWR027
U 1 1 57152280
P 4300 5700
F 0 "#PWR027" H 4300 5450 50  0001 C CNN
F 1 "GND" H 4300 5550 50  0000 C CNN
F 2 "" H 4300 5700 50  0000 C CNN
F 3 "" H 4300 5700 50  0000 C CNN
	1    4300 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 5700 4650 5700
Wire Wire Line
	4650 5700 4650 5750
Wire Wire Line
	4650 5750 4850 5750
Text GLabel 4250 5650 0    60   Input ~ 0
LCD_PE5
Wire Wire Line
	4250 5650 4850 5650
$Comp
L GND #PWR028
U 1 1 571524D0
P 3700 5550
F 0 "#PWR028" H 3700 5300 50  0001 C CNN
F 1 "GND" H 3700 5400 50  0000 C CNN
F 2 "" H 3700 5550 50  0000 C CNN
F 3 "" H 3700 5550 50  0000 C CNN
	1    3700 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 5550 4850 5550
$Comp
L GND #PWR029
U 1 1 57152603
P 3550 5450
F 0 "#PWR029" H 3550 5200 50  0001 C CNN
F 1 "GND" H 3550 5300 50  0000 C CNN
F 2 "" H 3550 5450 50  0000 C CNN
F 3 "" H 3550 5450 50  0000 C CNN
	1    3550 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 5450 4850 5450
$Comp
L +3.3V #PWR030
U 1 1 571526CA
P 4850 5350
F 0 "#PWR030" H 4850 5200 50  0001 C CNN
F 1 "+3.3V" H 4850 5490 50  0000 C CNN
F 2 "" H 4850 5350 50  0000 C CNN
F 3 "" H 4850 5350 50  0000 C CNN
	1    4850 5350
	1    0    0    -1  
$EndComp
Text GLabel 9500 3950 0    60   Input ~ 0
LCD_PE5
Text GLabel 8800 3850 0    60   Input ~ 0
LCD_PE4
Wire Wire Line
	8800 3850 9500 3850
Text GLabel 9500 4800 0    60   Input ~ 0
LCD_PD0
Text GLabel 9500 5000 0    60   Input ~ 0
LCD_PD2
Text GLabel 8900 4900 0    60   Input ~ 0
LCD_PD1
Wire Wire Line
	8900 4900 9500 4900
Text GLabel 8900 5100 0    60   Input ~ 0
LCD_PD3
Wire Wire Line
	8900 5100 9500 5100
$Comp
L CONN_01X03 WS2812_CONN1
U 1 1 57160E8B
P 6200 6550
F 0 "WS2812_CONN1" H 6200 6750 50  0000 C CNN
F 1 "CONN_01X03" V 6300 6550 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 6200 6550 50  0001 C CNN
F 3 "" H 6200 6550 50  0000 C CNN
	1    6200 6550
	1    0    0    -1  
$EndComp
Text GLabel 6000 6650 0    60   Input ~ 0
WS2812_DIN
$Comp
L +5V #PWR031
U 1 1 57162C29
P 5300 6550
F 0 "#PWR031" H 5300 6400 50  0001 C CNN
F 1 "+5V" H 5300 6690 50  0000 C CNN
F 2 "" H 5300 6550 50  0000 C CNN
F 3 "" H 5300 6550 50  0000 C CNN
	1    5300 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 6550 5300 6550
$Comp
L GND #PWR032
U 1 1 57162CF7
P 5750 6350
F 0 "#PWR032" H 5750 6100 50  0001 C CNN
F 1 "GND" H 5750 6200 50  0000 C CNN
F 2 "" H 5750 6350 50  0000 C CNN
F 3 "" H 5750 6350 50  0000 C CNN
	1    5750 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 6350 5900 6350
Wire Wire Line
	5900 6350 5900 6450
Wire Wire Line
	5900 6450 6000 6450
Text GLabel 8850 5500 0    60   Input ~ 0
WS2812_DIN
Wire Wire Line
	9500 5500 8850 5500
Text GLabel 1550 3950 3    60   Input ~ 0
BUT_1_4_ADC
Connection ~ 1550 3600
Text GLabel 8150 5200 0    60   Input ~ 0
BUT_1_4_ADC
Wire Wire Line
	8150 5200 9500 5200
Text GLabel 2850 3950 3    60   Input ~ 0
BUT_5_8_ADC
Connection ~ 2850 3600
Text GLabel 8850 5300 0    60   Input ~ 0
BUT_5_8_ADC
Wire Wire Line
	8850 5300 9500 5300
Text GLabel 7100 3050 0    60   Input ~ 0
MM5450_CLK
Text GLabel 7600 3050 2    60   Input ~ 0
MM5450_DIN
$Comp
L CONN_02X03 MM5450_CONN1
U 1 1 57176A80
P 7350 3150
F 0 "MM5450_CONN1" H 7350 3350 50  0000 C CNN
F 1 "CONN_02X03" H 7350 2950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 7350 1950 50  0001 C CNN
F 3 "" H 7350 1950 50  0000 C CNN
	1    7350 3150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR033
U 1 1 57176B33
P 7100 3250
F 0 "#PWR033" H 7100 3000 50  0001 C CNN
F 1 "GND" H 7100 3100 50  0000 C CNN
F 2 "" H 7100 3250 50  0000 C CNN
F 3 "" H 7100 3250 50  0000 C CNN
	1    7100 3250
	1    0    0    -1  
$EndComp
$Comp
L +3.3V #PWR034
U 1 1 57176BDF
P 7900 3350
F 0 "#PWR034" H 7900 3200 50  0001 C CNN
F 1 "+3.3V" H 7900 3490 50  0000 C CNN
F 2 "" H 7900 3350 50  0000 C CNN
F 3 "" H 7900 3350 50  0000 C CNN
	1    7900 3350
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR035
U 1 1 57176C3F
P 7800 3500
F 0 "#PWR035" H 7800 3350 50  0001 C CNN
F 1 "+5V" H 7800 3640 50  0000 C CNN
F 2 "" H 7800 3500 50  0000 C CNN
F 3 "" H 7800 3500 50  0000 C CNN
	1    7800 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 3150 7750 3150
Wire Wire Line
	7750 3150 7750 3350
Wire Wire Line
	7750 3350 7900 3350
Wire Wire Line
	7600 3250 7600 3500
Wire Wire Line
	7600 3500 7800 3500
Text GLabel 10850 5000 2    60   Input ~ 0
MM5450_CLK
Text GLabel 11600 4900 2    60   Input ~ 0
MM5450_DIN
$Comp
L +3.3V #PWR036
U 1 1 57505544
P 9500 3450
F 0 "#PWR036" H 9500 3300 50  0001 C CNN
F 1 "+3.3V" H 9500 3590 50  0000 C CNN
F 2 "" H 9500 3450 50  0000 C CNN
F 3 "" H 9500 3450 50  0000 C CNN
	1    9500 3450
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR037
U 1 1 575F1466
P 9500 4700
F 0 "#PWR037" H 9500 4450 50  0001 C CNN
F 1 "GND" H 9500 4550 50  0000 C CNN
F 2 "" H 9500 4700 50  0000 C CNN
F 3 "" H 9500 4700 50  0000 C CNN
	1    9500 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	11600 4900 10850 4900
$Comp
L GND #PWR038
U 1 1 575F493E
P 10850 5750
F 0 "#PWR038" H 10850 5500 50  0001 C CNN
F 1 "GND" H 10850 5600 50  0000 C CNN
F 2 "" H 10850 5750 50  0000 C CNN
F 3 "" H 10850 5750 50  0000 C CNN
	1    10850 5750
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR039
U 1 1 575F499E
P 11150 5850
F 0 "#PWR039" H 11150 5600 50  0001 C CNN
F 1 "GND" H 11150 5700 50  0000 C CNN
F 2 "" H 11150 5850 50  0000 C CNN
F 3 "" H 11150 5850 50  0000 C CNN
	1    11150 5850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10850 5850 11150 5850
$Comp
L +3.3V #PWR040
U 1 1 575F4AE9
P 10850 5950
F 0 "#PWR040" H 10850 5800 50  0001 C CNN
F 1 "+3.3V" H 10850 6090 50  0000 C CNN
F 2 "" H 10850 5950 50  0000 C CNN
F 3 "" H 10850 5950 50  0000 C CNN
	1    10850 5950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR041
U 1 1 575F548C
P 9500 5750
F 0 "#PWR041" H 9500 5500 50  0001 C CNN
F 1 "GND" H 9500 5600 50  0000 C CNN
F 2 "" H 9500 5750 50  0000 C CNN
F 3 "" H 9500 5750 50  0000 C CNN
	1    9500 5750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR042
U 1 1 575F54EC
P 9500 5850
F 0 "#PWR042" H 9500 5600 50  0001 C CNN
F 1 "GND" H 9500 5700 50  0000 C CNN
F 2 "" H 9500 5850 50  0000 C CNN
F 3 "" H 9500 5850 50  0000 C CNN
	1    9500 5850
	0    1    1    0   
$EndComp
$Comp
L +5V #PWR043
U 1 1 575F554C
P 9500 5950
F 0 "#PWR043" H 9500 5800 50  0001 C CNN
F 1 "+5V" H 9500 6090 50  0000 C CNN
F 2 "" H 9500 5950 50  0000 C CNN
F 3 "" H 9500 5950 50  0000 C CNN
	1    9500 5950
	-1   0    0    1   
$EndComp
$Comp
L stellaris U1
U 1 1 576007F3
P 9700 3350
F 0 "U1" H 9700 3350 60  0000 C CNN
F 1 "stellaris" H 9700 3350 60  0000 C CNN
F 2 "lib:stellaris_launchpad" H 9700 3350 60  0001 C CNN
F 3 "" H 9700 3350 60  0000 C CNN
	1    9700 3350
	1    0    0    -1  
$EndComp
$EndSCHEMATC
