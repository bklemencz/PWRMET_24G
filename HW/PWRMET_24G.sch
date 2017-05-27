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
Sheet 1 4
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 2125 3250 925  1150
U 591CC0DB
F0 "STM32_NRF24" 60
F1 "STM32_NRF24.sch" 60
F2 "PW_CF" I R 3050 3525 60 
F3 "PW_CF1" I R 3050 3375 60 
F4 "TX" I R 3050 4025 60 
F5 "RX" I R 3050 4175 60 
F6 "PW_SEL" I R 3050 3675 60 
F7 "DE" I R 3050 4325 60 
F8 "+3.3V" I L 2125 3375 60 
F9 "GND" I L 2125 4275 60 
$EndSheet
$Sheet
S 2125 1900 925  1050
U 591C108A
F0 "HLW8012" 60
F1 "HLW8012.sch" 60
F2 "+3.3V" I L 2125 2025 60 
F3 "+5V" I L 2125 2425 60 
F4 "GND" I L 2125 2825 60 
F5 "CF1" I R 3050 2025 60 
F6 "CF" I R 3050 2150 60 
F7 "SEL" I R 3050 2275 60 
$EndSheet
Wire Wire Line
	3050 3375 3250 3375
Wire Wire Line
	3250 3375 3250 2025
Wire Wire Line
	3250 2025 3050 2025
Wire Wire Line
	3050 3525 3350 3525
Wire Wire Line
	3350 3525 3350 2150
Wire Wire Line
	3350 2150 3050 2150
Wire Wire Line
	3050 3675 3450 3675
Wire Wire Line
	3450 3675 3450 2275
Wire Wire Line
	3450 2275 3050 2275
Wire Wire Line
	2125 2825 1800 2825
Wire Wire Line
	1800 2825 1800 5725
Wire Wire Line
	1800 4275 2125 4275
Wire Wire Line
	2125 3375 1900 3375
Wire Wire Line
	1900 2025 1900 4875
Wire Wire Line
	1900 2025 2125 2025
$Sheet
S 2125 4750 950  1100
U 59219A27
F0 "RS485_ISOL" 60
F1 "RS485_ISOL.sch" 60
F2 "GND" I L 2125 5725 60 
F3 "+3.3V" I L 2125 4875 60 
F4 "RX" I R 3075 5025 60 
F5 "TX" I R 3075 4875 60 
F6 "DE" I R 3075 5175 60 
$EndSheet
Wire Wire Line
	1900 4875 2125 4875
Connection ~ 1900 3375
Wire Wire Line
	1800 5725 2125 5725
Connection ~ 1800 4275
Wire Wire Line
	3075 4875 3250 4875
Wire Wire Line
	3250 4875 3250 4025
Wire Wire Line
	3250 4025 3050 4025
Wire Wire Line
	3075 5025 3350 5025
Wire Wire Line
	3350 5025 3350 4175
Wire Wire Line
	3350 4175 3050 4175
Wire Wire Line
	3075 5175 3450 5175
Wire Wire Line
	3450 5175 3450 4325
Wire Wire Line
	3450 4325 3050 4325
$EndSCHEMATC