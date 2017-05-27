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
Text HLabel 3075 4300 0    60   Input ~ 0
GND
Text HLabel 3100 3825 0    60   Input ~ 0
+3.3V
Text HLabel 3100 4025 0    60   Input ~ 0
RX
Text HLabel 2825 2750 0    60   Input ~ 0
TX
$Comp
L MAX485 U5
U 1 1 5921AB9E
P 6675 1950
F 0 "U5" H 6675 2250 60  0000 C CNN
F 1 "MAX485" H 6700 1650 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-8_3.9x4.9mm_Pitch1.27mm" H 6375 1600 60  0001 C CNN
F 3 "" H 6675 1950 60  0000 C CNN
	1    6675 1950
	1    0    0    -1  
$EndComp
$Comp
L PC817 U8
U 1 1 5921AD12
P 3775 3925
F 0 "U8" H 3575 4125 50  0000 L CNN
F 1 "PC817" H 3775 4125 50  0000 L CNN
F 2 "Housings_DIP:DIP-4_W9.53mm_SMD" H 3575 3725 50  0001 L CIN
F 3 "" H 3775 3925 50  0000 L CNN
	1    3775 3925
	-1   0    0    -1  
$EndComp
$Comp
L PC817 U6
U 1 1 5921AD65
P 3725 2850
F 0 "U6" H 3525 3050 50  0000 L CNN
F 1 "PC817" H 3725 3050 50  0000 L CNN
F 2 "Housings_DIP:DIP-4_W9.53mm_SMD" H 3525 2650 50  0001 L CIN
F 3 "" H 3725 2850 50  0000 L CNN
	1    3725 2850
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R25
U 1 1 5921AF47
P 3075 2750
F 0 "R25" V 3150 2700 50  0000 L CNN
F 1 "150" V 3000 2675 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3125 2600 50  0001 C CNN
F 3 "" H 3075 2750 50  0000 C CNN
	1    3075 2750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR027
U 1 1 5921B052
P 3225 3000
F 0 "#PWR027" H 3225 2750 50  0001 C CNN
F 1 "GND" H 3225 2850 50  0000 C CNN
F 2 "" H 3225 3000 50  0000 C CNN
F 3 "" H 3225 3000 50  0000 C CNN
	1    3225 3000
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R26
U 1 1 5921B140
P 4175 3100
F 0 "R26" V 4250 3050 50  0000 L CNN
F 1 "150" V 4100 3025 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4225 2950 50  0001 C CNN
F 3 "" H 4175 3100 50  0000 C CNN
	1    4175 3100
	-1   0    0    1   
$EndComp
Text Label 4300 2950 0    60   ~ 0
DI
Text Label 4200 3300 0    60   ~ 0
GISO
Text Label 4200 2750 0    60   ~ 0
PISO
Text HLabel 2825 1750 0    60   Input ~ 0
DE
$Comp
L PC817 U4
U 1 1 5921B4A7
P 3725 1850
F 0 "U4" H 3525 2050 50  0000 L CNN
F 1 "PC817" H 3725 2050 50  0000 L CNN
F 2 "Housings_DIP:DIP-4_W9.53mm_SMD" H 3525 1650 50  0001 L CIN
F 3 "" H 3725 1850 50  0000 L CNN
	1    3725 1850
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R21
U 1 1 5921B4AD
P 3075 1750
F 0 "R21" V 3150 1700 50  0000 L CNN
F 1 "150" V 3000 1675 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3125 1600 50  0001 C CNN
F 3 "" H 3075 1750 50  0000 C CNN
	1    3075 1750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR028
U 1 1 5921B4B3
P 3225 2000
F 0 "#PWR028" H 3225 1750 50  0001 C CNN
F 1 "GND" H 3225 1850 50  0000 C CNN
F 2 "" H 3225 2000 50  0000 C CNN
F 3 "" H 3225 2000 50  0000 C CNN
	1    3225 2000
	1    0    0    -1  
$EndComp
$Comp
L R_SMD_0805 R22
U 1 1 5921B4B9
P 4175 2100
F 0 "R22" V 4250 2050 50  0000 L CNN
F 1 "150" V 4100 2025 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4225 1950 50  0001 C CNN
F 3 "" H 4175 2100 50  0000 C CNN
	1    4175 2100
	-1   0    0    1   
$EndComp
Text Label 4175 1950 0    60   ~ 0
REDE
Text Label 4200 2300 0    60   ~ 0
GISO
Text Label 4200 1750 0    60   ~ 0
PISO
$Comp
L R_SMD_0805 R27
U 1 1 5921B6AE
P 4300 3825
F 0 "R27" V 4375 3775 50  0000 L CNN
F 1 "200" V 4225 3750 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 4350 3675 50  0001 C CNN
F 3 "" H 4300 3825 50  0000 C CNN
	1    4300 3825
	0    1    1    0   
$EndComp
Text Label 4525 3825 0    60   ~ 0
RO
Text Label 4475 4025 0    60   ~ 0
GISO
$Comp
L R_SMD_0805 R28
U 1 1 5921BB54
P 3275 4175
F 0 "R28" V 3350 4125 50  0000 L CNN
F 1 "150" V 3200 4100 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 3325 4025 50  0001 C CNN
F 3 "" H 3275 4175 50  0000 C CNN
	1    3275 4175
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR029
U 1 1 5921BC22
P 3275 4325
F 0 "#PWR029" H 3275 4075 50  0001 C CNN
F 1 "GND" H 3275 4175 50  0000 C CNN
F 2 "" H 3275 4325 50  0000 C CNN
F 3 "" H 3275 4325 50  0000 C CNN
	1    3275 4325
	1    0    0    -1  
$EndComp
Wire Wire Line
	3175 2750 3425 2750
Wire Wire Line
	3425 2950 3225 2950
Wire Wire Line
	3225 2950 3225 3000
Wire Wire Line
	4025 2750 4425 2750
Wire Wire Line
	4025 2950 4425 2950
Wire Wire Line
	4175 2950 4175 3000
Wire Wire Line
	4175 3200 4175 3300
Wire Wire Line
	4175 3300 4425 3300
Connection ~ 4175 2950
Wire Wire Line
	2975 2750 2825 2750
Wire Wire Line
	3175 1750 3425 1750
Wire Wire Line
	3425 1950 3225 1950
Wire Wire Line
	3225 1950 3225 2000
Wire Wire Line
	4025 1750 4425 1750
Wire Wire Line
	4025 1950 4425 1950
Wire Wire Line
	4175 1950 4175 2000
Wire Wire Line
	4175 2200 4175 2300
Wire Wire Line
	4175 2300 4425 2300
Connection ~ 4175 1950
Wire Wire Line
	2975 1750 2825 1750
Wire Wire Line
	4075 3825 4200 3825
Wire Wire Line
	4400 3825 4675 3825
Wire Wire Line
	4075 4025 4675 4025
Wire Wire Line
	3100 4025 3475 4025
Wire Wire Line
	3275 4025 3275 4075
Wire Wire Line
	3275 4275 3275 4325
Wire Wire Line
	3100 3825 3475 3825
Connection ~ 3275 4025
Wire Wire Line
	6275 1800 5875 1800
Wire Wire Line
	6275 1900 5875 1900
Wire Wire Line
	6275 2100 5875 2100
Text Label 5875 1800 0    60   ~ 0
RO
Text Label 5875 1900 0    60   ~ 0
REDE
Text Label 5875 2100 0    60   ~ 0
DI
Wire Wire Line
	7075 1900 7825 1900
Wire Wire Line
	7075 2000 7825 2000
Text Label 7100 1800 0    60   ~ 0
PISO
Text Label 7775 1900 0    60   ~ 0
B
Text Label 7775 2000 0    60   ~ 0
A
Text Label 7100 2100 0    60   ~ 0
GISO
Wire Wire Line
	3075 4300 3275 4300
Connection ~ 3275 4300
$Comp
L CONN_01X04 P5
U 1 1 5921DD29
P 10350 2675
F 0 "P5" H 10350 2925 50  0000 C CNN
F 1 "CONN_01X04" V 10450 2675 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 10350 2675 50  0001 C CNN
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
L R_SMD_0805 R20
U 1 1 5921EE6D
P 7475 1500
F 0 "R20" V 7550 1450 50  0000 L CNN
F 1 "20K" V 7400 1425 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 7525 1350 50  0001 C CNN
F 3 "" H 7475 1500 50  0000 C CNN
	1    7475 1500
	-1   0    0    1   
$EndComp
Wire Wire Line
	7475 1400 7475 1300
Wire Wire Line
	7475 1300 7675 1300
Text Label 7475 1300 0    60   ~ 0
PISO
$Comp
L R_SMD_0805 R24
U 1 1 5921EF8C
P 7375 2400
F 0 "R24" V 7450 2350 50  0000 L CNN
F 1 "20K" V 7300 2325 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 7425 2250 50  0001 C CNN
F 3 "" H 7375 2400 50  0000 C CNN
	1    7375 2400
	-1   0    0    1   
$EndComp
Wire Wire Line
	7375 2500 7375 2650
Wire Wire Line
	7375 2650 7625 2650
Text Label 7425 2650 0    60   ~ 0
GISO
$Comp
L R_SMD_0805 R23
U 1 1 5921F3BE
P 7625 2250
F 0 "R23" V 7700 2200 50  0000 L CNN
F 1 "120" V 7550 2175 50  0000 L CNN
F 2 "Resistors_SMD:R_0805" H 7675 2100 50  0001 C CNN
F 3 "" H 7625 2250 50  0000 C CNN
	1    7625 2250
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7525 2250 7525 1900
Connection ~ 7525 1900
Wire Wire Line
	7725 2250 7725 2000
Connection ~ 7725 2000
$Comp
L MC78L05ACH U7
U 1 1 5922066B
P 6625 3325
F 0 "U7" H 6425 3525 50  0000 C CNN
F 1 "MC78L05ACH" H 6625 3525 50  0000 L CNN
F 2 "TO_SOT_Packages_SMD:SOT-89-3" H 6625 3425 50  0001 C CIN
F 3 "" H 6625 3325 50  0000 C CNN
	1    6625 3325
	1    0    0    -1  
$EndComp
Wire Wire Line
	7025 3275 7775 3275
Wire Wire Line
	6625 3575 6625 3750
Wire Wire Line
	6225 3275 5700 3275
Connection ~ 6625 3750
Wire Wire Line
	5700 3750 7775 3750
Text Label 7600 3750 0    60   ~ 0
GISO
Text Label 7575 3275 0    60   ~ 0
PISO
Text Label 5700 3275 0    60   ~ 0
12V_ISO
$Comp
L C_Cer_SMD_0805 C9
U 1 1 59220DDC
P 5850 3500
F 0 "C9" H 5860 3570 50  0000 L CNN
F 1 "1u" H 5860 3420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5850 3350 50  0001 C CNN
F 3 "" H 5850 3500 50  0000 C CNN
	1    5850 3500
	1    0    0    -1  
$EndComp
$Comp
L C_Cer_SMD_0805 C10
U 1 1 59220F22
P 6025 3500
F 0 "C10" H 6035 3570 50  0000 L CNN
F 1 "100n" H 6035 3420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 6025 3350 50  0001 C CNN
F 3 "" H 6025 3500 50  0000 C CNN
	1    6025 3500
	1    0    0    -1  
$EndComp
$Comp
L C_Cer_SMD_0805 C12
U 1 1 59220FC1
P 7525 3500
F 0 "C12" H 7535 3570 50  0000 L CNN
F 1 "100n" H 7535 3420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 7525 3350 50  0001 C CNN
F 3 "" H 7525 3500 50  0000 C CNN
	1    7525 3500
	1    0    0    -1  
$EndComp
$Comp
L C_Cer_SMD_0805 C11
U 1 1 59221040
P 7325 3500
F 0 "C11" H 7335 3570 50  0000 L CNN
F 1 "1u" H 7335 3420 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 7325 3350 50  0001 C CNN
F 3 "" H 7325 3500 50  0000 C CNN
	1    7325 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 3400 5850 3275
Connection ~ 5850 3275
Wire Wire Line
	6025 3400 6025 3275
Connection ~ 6025 3275
Wire Wire Line
	5850 3600 5850 3750
Connection ~ 5850 3750
Wire Wire Line
	6025 3600 6025 3750
Connection ~ 6025 3750
Wire Wire Line
	7325 3400 7325 3275
Connection ~ 7325 3275
Wire Wire Line
	7525 3400 7525 3275
Connection ~ 7525 3275
Wire Wire Line
	7325 3600 7325 3750
Connection ~ 7325 3750
Wire Wire Line
	7525 3600 7525 3750
Connection ~ 7525 3750
Wire Wire Line
	6175 2000 6275 2000
Wire Wire Line
	6175 2000 6175 1900
Connection ~ 6175 1900
Wire Wire Line
	7075 1800 7300 1800
Wire Wire Line
	7075 2100 7300 2100
Wire Wire Line
	7375 2300 7375 1900
Connection ~ 7375 1900
Wire Wire Line
	7475 1600 7475 2000
Connection ~ 7475 2000
$EndSCHEMATC
