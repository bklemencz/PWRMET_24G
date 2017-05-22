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
LIBS:stm32
LIBS:BK_Common
LIBS:BK_LED_Drivers
LIBS:BK_STM8
LIBS:PWRMET_24G-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 4 4
Title "Isolated RS485 3.3V"
Date "2017-05-21"
Rev "V0.1"
Comp "BK"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text HLabel 2300 5075 0    60   Input ~ 0
GND
Text HLabel 2325 4600 0    60   Input ~ 0
+3.3V
Text HLabel 2325 4800 0    60   Input ~ 0
RX
Text HLabel 2050 3525 0    60   Input ~ 0
TX
$Comp
L MAX485 U?
U 1 1 5921AB9E
P 5900 2725
F 0 "U?" H 5900 3025 60  0000 C CNN
F 1 "MAX485" H 5925 2425 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 5600 2375 60  0001 C CNN
F 3 "" H 5900 2725 60  0000 C CNN
	1    5900 2725
	1    0    0    -1  
$EndComp
$Comp
L PC817 U?
U 1 1 5921AD12
P 3000 4700
F 0 "U?" H 2800 4900 50  0000 L CNN
F 1 "PC817" H 3000 4900 50  0000 L CNN
F 2 "DIP-4" H 2800 4500 50  0000 L CIN
F 3 "" H 3000 4700 50  0000 L CNN
	1    3000 4700
	-1   0    0    -1  
$EndComp
$Comp
L PC817 U?
U 1 1 5921AD65
P 2950 3625
F 0 "U?" H 2750 3825 50  0000 L CNN
F 1 "PC817" H 2950 3825 50  0000 L CNN
F 2 "DIP-4" H 2750 3425 50  0000 L CIN
F 3 "" H 2950 3625 50  0000 L CNN
	1    2950 3625
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R?
U 1 1 5921AF47
P 2300 3525
F 0 "R?" V 2375 3475 50  0000 L CNN
F 1 "150" V 2225 3450 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2350 3375 50  0001 C CNN
F 3 "" H 2300 3525 50  0000 C CNN
	1    2300 3525
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5921B052
P 2450 3775
F 0 "#PWR?" H 2450 3525 50  0001 C CNN
F 1 "GND" H 2450 3625 50  0000 C CNN
F 2 "" H 2450 3775 50  0000 C CNN
F 3 "" H 2450 3775 50  0000 C CNN
	1    2450 3775
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R?
U 1 1 5921B140
P 3400 3875
F 0 "R?" V 3475 3825 50  0000 L CNN
F 1 "150" V 3325 3800 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3450 3725 50  0001 C CNN
F 3 "" H 3400 3875 50  0000 C CNN
	1    3400 3875
	-1   0    0    1   
$EndComp
Text Label 3525 3725 0    60   ~ 0
DI
Text Label 3425 4075 0    60   ~ 0
GISO
Text Label 3425 3525 0    60   ~ 0
PISO
Text HLabel 2050 2525 0    60   Input ~ 0
DE
$Comp
L PC817 U?
U 1 1 5921B4A7
P 2950 2625
F 0 "U?" H 2750 2825 50  0000 L CNN
F 1 "PC817" H 2950 2825 50  0000 L CNN
F 2 "DIP-4" H 2750 2425 50  0000 L CIN
F 3 "" H 2950 2625 50  0000 L CNN
	1    2950 2625
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R?
U 1 1 5921B4AD
P 2300 2525
F 0 "R?" V 2375 2475 50  0000 L CNN
F 1 "150" V 2225 2450 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2350 2375 50  0001 C CNN
F 3 "" H 2300 2525 50  0000 C CNN
	1    2300 2525
	0    1    1    0   
$EndComp
$Comp
L GND #PWR?
U 1 1 5921B4B3
P 2450 2775
F 0 "#PWR?" H 2450 2525 50  0001 C CNN
F 1 "GND" H 2450 2625 50  0000 C CNN
F 2 "" H 2450 2775 50  0000 C CNN
F 3 "" H 2450 2775 50  0000 C CNN
	1    2450 2775
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R?
U 1 1 5921B4B9
P 3400 2875
F 0 "R?" V 3475 2825 50  0000 L CNN
F 1 "150" V 3325 2800 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3450 2725 50  0001 C CNN
F 3 "" H 3400 2875 50  0000 C CNN
	1    3400 2875
	-1   0    0    1   
$EndComp
Text Label 3400 2725 0    60   ~ 0
REDE
Text Label 3425 3075 0    60   ~ 0
GISO
Text Label 3425 2525 0    60   ~ 0
PISO
$Comp
L R_SMD_0805 R?
U 1 1 5921B6AE
P 3525 4600
F 0 "R?" V 3600 4550 50  0000 L CNN
F 1 "200" V 3450 4525 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3575 4450 50  0001 C CNN
F 3 "" H 3525 4600 50  0000 C CNN
	1    3525 4600
	0    1    1    0   
$EndComp
Text Label 3750 4600 0    60   ~ 0
RO
Text Label 3700 4800 0    60   ~ 0
GISO
$Comp
L R_SMD_0805 R?
U 1 1 5921BB54
P 2500 4950
F 0 "R?" V 2575 4900 50  0000 L CNN
F 1 "150" V 2425 4875 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 2550 4800 50  0001 C CNN
F 3 "" H 2500 4950 50  0000 C CNN
	1    2500 4950
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR?
U 1 1 5921BC22
P 2500 5100
F 0 "#PWR?" H 2500 4850 50  0001 C CNN
F 1 "GND" H 2500 4950 50  0000 C CNN
F 2 "" H 2500 5100 50  0000 C CNN
F 3 "" H 2500 5100 50  0000 C CNN
	1    2500 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2400 3525 2650 3525
Wire Wire Line
	2650 3725 2450 3725
Wire Wire Line
	2450 3725 2450 3775
Wire Wire Line
	3250 3525 3650 3525
Wire Wire Line
	3250 3725 3650 3725
Wire Wire Line
	3400 3725 3400 3775
Wire Wire Line
	3400 3975 3400 4075
Wire Wire Line
	3400 4075 3650 4075
Connection ~ 3400 3725
Wire Wire Line
	2200 3525 2050 3525
Wire Wire Line
	2400 2525 2650 2525
Wire Wire Line
	2650 2725 2450 2725
Wire Wire Line
	2450 2725 2450 2775
Wire Wire Line
	3250 2525 3650 2525
Wire Wire Line
	3250 2725 3650 2725
Wire Wire Line
	3400 2725 3400 2775
Wire Wire Line
	3400 2975 3400 3075
Wire Wire Line
	3400 3075 3650 3075
Connection ~ 3400 2725
Wire Wire Line
	2200 2525 2050 2525
Wire Wire Line
	3300 4600 3425 4600
Wire Wire Line
	3625 4600 3900 4600
Wire Wire Line
	3300 4800 3900 4800
Wire Wire Line
	2325 4800 2700 4800
Wire Wire Line
	2500 4800 2500 4850
Wire Wire Line
	2500 5050 2500 5100
Wire Wire Line
	2325 4600 2700 4600
Connection ~ 2500 4800
Wire Wire Line
	5500 2575 5100 2575
Wire Wire Line
	5500 2775 5400 2775
Wire Wire Line
	5400 2775 5400 2675
Wire Wire Line
	5500 2675 5100 2675
Wire Wire Line
	5500 2875 5100 2875
Text Label 5100 2575 0    60   ~ 0
RO
Text Label 5100 2675 0    60   ~ 0
REDE
Text Label 5100 2875 0    60   ~ 0
DI
Wire Wire Line
	6300 2575 6700 2575
Wire Wire Line
	6300 2675 7050 2675
Wire Wire Line
	6300 2775 7050 2775
Wire Wire Line
	6300 2875 6700 2875
Text Label 6500 2575 0    60   ~ 0
PISO
Text Label 7000 2675 0    60   ~ 0
B
Text Label 7000 2775 0    60   ~ 0
A
Text Label 6500 2875 0    60   ~ 0
GISO
Wire Wire Line
	2300 5075 2500 5075
Connection ~ 2500 5075
$Comp
L CONN_01X04 P?
U 1 1 5921DD29
P 10350 2675
F 0 "P?" H 10350 2925 50  0000 C CNN
F 1 "CONN_01X04" V 10450 2675 50  0000 C CNN
F 2 "" H 10350 2675 50  0000 C CNN
F 3 "" H 10350 2675 50  0000 C CNN
	1    10350 2675
	1    0    0    -1  
$EndComp
Wire Wire Line
	10150 2525 9800 2525
Wire Wire Line
	10150 2625 9800 2625
Wire Wire Line
	10150 2725 9800 2725
Wire Wire Line
	10150 2825 9800 2825
Text Label 9800 2525 0    60   ~ 0
12V_ISO
Text Label 9800 2625 0    60   ~ 0
B
Text Label 9800 2725 0    60   ~ 0
A
Text Label 9800 2825 0    60   ~ 0
GISO
$Comp
L R_SMD_0805 R?
U 1 1 5921EE6D
P 6500 2275
F 0 "R?" V 6575 2225 50  0000 L CNN
F 1 "20K" V 6425 2200 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 6550 2125 50  0001 C CNN
F 3 "" H 6500 2275 50  0000 C CNN
	1    6500 2275
	-1   0    0    1   
$EndComp
Wire Wire Line
	6500 2175 6500 2075
Wire Wire Line
	6500 2075 6700 2075
Wire Wire Line
	6500 2375 6500 2775
Connection ~ 6500 2775
Text Label 6500 2075 0    60   ~ 0
PISO
$Comp
L R_SMD_0805 R?
U 1 1 5921EF8C
P 6450 3175
F 0 "R?" V 6525 3125 50  0000 L CNN
F 1 "20K" V 6375 3100 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 6500 3025 50  0001 C CNN
F 3 "" H 6450 3175 50  0000 C CNN
	1    6450 3175
	-1   0    0    1   
$EndComp
Wire Wire Line
	6450 3075 6450 2675
Connection ~ 6450 2675
Wire Wire Line
	6450 3275 6450 3425
Wire Wire Line
	6450 3425 6700 3425
Text Label 6500 3425 0    60   ~ 0
GISO
$Comp
L R_SMD_0805 R?
U 1 1 5921F3BE
P 6850 3025
F 0 "R?" V 6925 2975 50  0000 L CNN
F 1 "120" V 6775 2950 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 6900 2875 50  0001 C CNN
F 3 "" H 6850 3025 50  0000 C CNN
	1    6850 3025
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6750 3025 6750 2675
Connection ~ 6750 2675
Wire Wire Line
	6950 3025 6950 2775
Connection ~ 6950 2775
$Comp
L MC78L05ACH U?
U 1 1 5922066B
P 5850 4100
F 0 "U?" H 5650 4300 50  0000 C CNN
F 1 "MC78L05ACH" H 5850 4300 50  0000 L CNN
F 2 "SOT-89" H 5850 4200 50  0000 C CIN
F 3 "" H 5850 4100 50  0000 C CNN
	1    5850 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 4050 7000 4050
Wire Wire Line
	5850 4350 5850 4525
Wire Wire Line
	5450 4050 4925 4050
Connection ~ 5850 4525
Wire Wire Line
	4925 4525 7000 4525
Text Label 6825 4525 0    60   ~ 0
GISO
Text Label 6800 4050 0    60   ~ 0
PISO
Text Label 4925 4050 0    60   ~ 0
12V_ISO
$Comp
L C_Cer_SMD_0805 C?
U 1 1 59220DDC
P 5075 4275
F 0 "C?" H 5085 4345 50  0000 L CNN
F 1 "1u" H 5085 4195 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5075 4125 50  0001 C CNN
F 3 "" H 5075 4275 50  0000 C CNN
	1    5075 4275
	1    0    0    -1  
$EndComp
$Comp
L C_Cer_SMD_0805 C?
U 1 1 59220F22
P 5250 4275
F 0 "C?" H 5260 4345 50  0000 L CNN
F 1 "100n" H 5260 4195 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5250 4125 50  0001 C CNN
F 3 "" H 5250 4275 50  0000 C CNN
	1    5250 4275
	1    0    0    -1  
$EndComp
$Comp
L C_Cer_SMD_0805 C?
U 1 1 59220FC1
P 6750 4275
F 0 "C?" H 6760 4345 50  0000 L CNN
F 1 "100n" H 6760 4195 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6750 4125 50  0001 C CNN
F 3 "" H 6750 4275 50  0000 C CNN
	1    6750 4275
	1    0    0    -1  
$EndComp
$Comp
L C_Cer_SMD_0805 C?
U 1 1 59221040
P 6550 4275
F 0 "C?" H 6560 4345 50  0000 L CNN
F 1 "1u" H 6560 4195 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6550 4125 50  0001 C CNN
F 3 "" H 6550 4275 50  0000 C CNN
	1    6550 4275
	1    0    0    -1  
$EndComp
Wire Wire Line
	5075 4175 5075 4050
Connection ~ 5075 4050
Wire Wire Line
	5250 4175 5250 4050
Connection ~ 5250 4050
Wire Wire Line
	5075 4375 5075 4525
Connection ~ 5075 4525
Wire Wire Line
	5250 4375 5250 4525
Connection ~ 5250 4525
Wire Wire Line
	6550 4175 6550 4050
Connection ~ 6550 4050
Wire Wire Line
	6750 4175 6750 4050
Connection ~ 6750 4050
Wire Wire Line
	6550 4375 6550 4525
Connection ~ 6550 4525
Wire Wire Line
	6750 4375 6750 4525
Connection ~ 6750 4525
$EndSCHEMATC
