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
LIBS:custom
LIBS:fuzzer-cache
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
L USB_OTG J1
U 1 1 58FFDDE4
P 2200 2700
F 0 "J1" H 2000 3150 50  0000 L CNN
F 1 "USB_OTG" H 2000 3050 50  0000 L CNN
F 2 "Connect:USB_Micro-B" H 2350 2650 50  0001 C CNN
F 3 "" H 2350 2650 50  0001 C CNN
	1    2200 2700
	1    0    0    -1  
$EndComp
NoConn ~ 2500 2900
Text Label 7600 2950 0    60   ~ 0
SWDAT
Text Label 6800 1950 1    60   ~ 0
SWCLK
Text Label 7450 3150 0    60   ~ 0
USB_DM
Text Label 7450 3050 0    60   ~ 0
USB_DP
$Comp
L STM32F042-48 U2
U 1 1 58FFF40C
P 6250 3300
F 0 "U2" H 6250 3200 50  0000 C CNN
F 1 "STM32F042-48" H 6250 3400 50  0000 C CNN
F 2 "Housings_QFP:LQFP-48_7x7mm_Pitch0.5mm" H 6250 3300 50  0001 C CNN
F 3 "DOCUMENTATION" H 6250 3300 50  0001 C CNN
	1    6250 3300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 58FFF489
P 6700 4450
F 0 "#PWR01" H 6700 4200 50  0001 C CNN
F 1 "GND" H 6700 4300 50  0000 C CNN
F 2 "" H 6700 4450 50  0001 C CNN
F 3 "" H 6700 4450 50  0001 C CNN
	1    6700 4450
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 58FFF4A3
P 7400 2850
F 0 "#PWR02" H 7400 2600 50  0001 C CNN
F 1 "GND" H 7400 2700 50  0000 C CNN
F 2 "" H 7400 2850 50  0001 C CNN
F 3 "" H 7400 2850 50  0001 C CNN
	1    7400 2850
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR03
U 1 1 58FFF4C2
P 5100 3450
F 0 "#PWR03" H 5100 3200 50  0001 C CNN
F 1 "GND" H 5100 3300 50  0000 C CNN
F 2 "" H 5100 3450 50  0001 C CNN
F 3 "" H 5100 3450 50  0001 C CNN
	1    5100 3450
	0    1    1    0   
$EndComp
$Comp
L VDD #PWR04
U 1 1 58FFF4D3
P 5100 3550
F 0 "#PWR04" H 5100 3400 50  0001 C CNN
F 1 "VDD" H 5100 3700 50  0000 C CNN
F 2 "" H 5100 3550 50  0001 C CNN
F 3 "" H 5100 3550 50  0001 C CNN
	1    5100 3550
	0    -1   -1   0   
$EndComp
$Comp
L VDD #PWR05
U 1 1 58FFF4EB
P 6800 4450
F 0 "#PWR05" H 6800 4300 50  0001 C CNN
F 1 "VDD" H 6800 4600 50  0000 C CNN
F 2 "" H 6800 4450 50  0001 C CNN
F 3 "" H 6800 4450 50  0001 C CNN
	1    6800 4450
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR06
U 1 1 58FFF4FC
P 7400 2750
F 0 "#PWR06" H 7400 2600 50  0001 C CNN
F 1 "VDD" H 7400 2900 50  0000 C CNN
F 2 "" H 7400 2750 50  0001 C CNN
F 3 "" H 7400 2750 50  0001 C CNN
	1    7400 2750
	0    1    1    0   
$EndComp
$Comp
L GND #PWR07
U 1 1 58FFF50D
P 5800 2150
F 0 "#PWR07" H 5800 1900 50  0001 C CNN
F 1 "GND" H 5800 2000 50  0000 C CNN
F 2 "" H 5800 2150 50  0001 C CNN
F 3 "" H 5800 2150 50  0001 C CNN
	1    5800 2150
	-1   0    0    1   
$EndComp
$Comp
L VDD #PWR08
U 1 1 58FFF51E
P 5700 2150
F 0 "#PWR08" H 5700 2000 50  0001 C CNN
F 1 "VDD" H 5700 2300 50  0000 C CNN
F 2 "" H 5700 2150 50  0001 C CNN
F 3 "" H 5700 2150 50  0001 C CNN
	1    5700 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 2950 7900 2950
Wire Wire Line
	6800 2150 6800 1650
Wire Wire Line
	7400 3150 7900 3150
Wire Wire Line
	7900 3050 7400 3050
Wire Wire Line
	6200 4450 6200 4750
Text Label 6200 4700 1    60   ~ 0
D0
Wire Wire Line
	6300 4450 6300 4750
Wire Wire Line
	6400 4450 6400 4750
Wire Wire Line
	6500 4450 6500 4750
Wire Wire Line
	6600 4450 6600 4750
Text Label 6300 4700 1    60   ~ 0
D1
Text Label 6400 4700 1    60   ~ 0
D2
Text Label 6500 4700 1    60   ~ 0
D2
Text Label 6600 4700 1    60   ~ 0
D3
Wire Wire Line
	7400 3850 7700 3850
Text Label 7650 3850 2    60   ~ 0
D4
Wire Wire Line
	7400 3750 7700 3750
Wire Wire Line
	7400 3650 7700 3650
Wire Wire Line
	7400 3550 7700 3550
Text Label 7650 3750 2    60   ~ 0
D5
Text Label 7650 3650 2    60   ~ 0
D6
Text Label 7650 3550 2    60   ~ 0
D7
Wire Wire Line
	6600 2150 6600 1850
Text Label 6600 1900 3    60   ~ 0
D3
Wire Wire Line
	6500 2150 6500 1850
Wire Wire Line
	6400 2150 6400 1850
Wire Wire Line
	6300 2150 6300 1850
Text Label 6500 1900 3    60   ~ 0
D4
Text Label 6400 1900 3    60   ~ 0
D5
Text Label 6300 1900 3    60   ~ 0
D6
Wire Wire Line
	6200 2150 6200 1850
Wire Wire Line
	6000 2150 6000 1850
Wire Wire Line
	5900 2150 5900 1850
Text Label 6200 1900 3    60   ~ 0
D7
Text Label 6000 1900 3    60   ~ 0
D0
Text Label 5900 1800 3    60   ~ 0
D1
NoConn ~ 5100 2750
NoConn ~ 5100 2850
NoConn ~ 5100 2950
NoConn ~ 5100 3050
NoConn ~ 5100 3150
NoConn ~ 5100 3250
NoConn ~ 5100 3350
NoConn ~ 5100 3650
NoConn ~ 5100 3750
NoConn ~ 5100 3850
NoConn ~ 7400 3450
NoConn ~ 7400 3350
NoConn ~ 7400 3250
NoConn ~ 6700 2150
Wire Wire Line
	6100 2150 6100 1700
Text Label 6100 2050 1    60   ~ 0
BOOT
Wire Wire Line
	2500 2800 2900 2800
Text Label 2550 2800 0    60   ~ 0
USB_DM
Wire Wire Line
	2500 2700 2900 2700
Text Label 2550 2700 0    60   ~ 0
USB_DP
$Comp
L GND #PWR09
U 1 1 58FFF995
P 2200 3100
F 0 "#PWR09" H 2200 2850 50  0001 C CNN
F 1 "GND" H 2200 2950 50  0000 C CNN
F 2 "" H 2200 3100 50  0001 C CNN
F 3 "" H 2200 3100 50  0001 C CNN
	1    2200 3100
	1    0    0    -1  
$EndComp
NoConn ~ 2100 3100
$Comp
L SOT223-REG U1
U 1 1 58FFFA4B
P 3700 2550
F 0 "U1" H 3850 2354 60  0000 C CNN
F 1 "SOT223-REG" H 3700 2750 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223" H 3700 2550 60  0001 C CNN
F 3 "" H 3700 2550 60  0000 C CNN
	1    3700 2550
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 58FFFC13
P 3200 2650
F 0 "C1" H 3225 2750 50  0000 L CNN
F 1 "2u2" H 3225 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 3238 2500 50  0001 C CNN
F 3 "" H 3200 2650 50  0001 C CNN
	1    3200 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3650 2800 3750 2800
$Comp
L GND #PWR010
U 1 1 58FFFC69
P 3700 2800
F 0 "#PWR010" H 3700 2550 50  0001 C CNN
F 1 "GND" H 3700 2650 50  0000 C CNN
F 2 "" H 3700 2800 50  0001 C CNN
F 3 "" H 3700 2800 50  0001 C CNN
	1    3700 2800
	1    0    0    -1  
$EndComp
Connection ~ 3700 2800
$Comp
L GND #PWR011
U 1 1 58FFFC89
P 3200 2800
F 0 "#PWR011" H 3200 2550 50  0001 C CNN
F 1 "GND" H 3200 2650 50  0000 C CNN
F 2 "" H 3200 2800 50  0001 C CNN
F 3 "" H 3200 2800 50  0001 C CNN
	1    3200 2800
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 58FFFCA0
P 4200 2650
F 0 "C2" H 4225 2750 50  0000 L CNN
F 1 "10u" H 4225 2550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4238 2500 50  0001 C CNN
F 3 "" H 4200 2650 50  0001 C CNN
	1    4200 2650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 58FFFCE9
P 4200 2800
F 0 "#PWR012" H 4200 2550 50  0001 C CNN
F 1 "GND" H 4200 2650 50  0000 C CNN
F 2 "" H 4200 2800 50  0001 C CNN
F 3 "" H 4200 2800 50  0001 C CNN
	1    4200 2800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2500 2500 3300 2500
Connection ~ 3200 2500
Wire Wire Line
	4100 2500 4400 2500
Connection ~ 4200 2500
$Comp
L VDD #PWR013
U 1 1 58FFFD89
P 4400 2500
F 0 "#PWR013" H 4400 2350 50  0001 C CNN
F 1 "VDD" H 4400 2650 50  0000 C CNN
F 2 "" H 4400 2500 50  0001 C CNN
F 3 "" H 4400 2500 50  0001 C CNN
	1    4400 2500
	0    1    1    0   
$EndComp
NoConn ~ 5700 4450
NoConn ~ 5800 4450
NoConn ~ 5900 4450
NoConn ~ 6000 4450
NoConn ~ 6100 4450
$Comp
L +5V #PWR014
U 1 1 58FFFFFD
P 3200 2500
F 0 "#PWR014" H 3200 2350 50  0001 C CNN
F 1 "+5V" H 3200 2640 50  0000 C CNN
F 2 "" H 3200 2500 50  0001 C CNN
F 3 "" H 3200 2500 50  0001 C CNN
	1    3200 2500
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 59000133
P 4650 3500
F 0 "C3" H 4675 3600 50  0000 L CNN
F 1 "100n" H 4675 3400 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 4688 3350 50  0001 C CNN
F 3 "" H 4650 3500 50  0001 C CNN
	1    4650 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 590001EF
P 4650 3650
F 0 "#PWR015" H 4650 3400 50  0001 C CNN
F 1 "GND" H 4650 3500 50  0000 C CNN
F 2 "" H 4650 3650 50  0001 C CNN
F 3 "" H 4650 3650 50  0001 C CNN
	1    4650 3650
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR016
U 1 1 5900020C
P 4650 3350
F 0 "#PWR016" H 4650 3200 50  0001 C CNN
F 1 "VDD" H 4650 3500 50  0000 C CNN
F 2 "" H 4650 3350 50  0001 C CNN
F 3 "" H 4650 3350 50  0001 C CNN
	1    4650 3350
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 59000309
P 5450 2000
F 0 "C4" H 5475 2100 50  0000 L CNN
F 1 "100n" H 5475 1900 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5488 1850 50  0001 C CNN
F 3 "" H 5450 2000 50  0001 C CNN
	1    5450 2000
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5900030F
P 5450 2150
F 0 "#PWR017" H 5450 1900 50  0001 C CNN
F 1 "GND" H 5450 2000 50  0000 C CNN
F 2 "" H 5450 2150 50  0001 C CNN
F 3 "" H 5450 2150 50  0001 C CNN
	1    5450 2150
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR018
U 1 1 59000315
P 5450 1850
F 0 "#PWR018" H 5450 1700 50  0001 C CNN
F 1 "VDD" H 5450 2000 50  0000 C CNN
F 2 "" H 5450 1850 50  0001 C CNN
F 3 "" H 5450 1850 50  0001 C CNN
	1    5450 1850
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 59000420
P 7000 4550
F 0 "C5" H 7025 4650 50  0000 L CNN
F 1 "100n" H 7025 4450 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7038 4400 50  0001 C CNN
F 3 "" H 7000 4550 50  0001 C CNN
	1    7000 4550
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 59000426
P 7000 4700
F 0 "#PWR019" H 7000 4450 50  0001 C CNN
F 1 "GND" H 7000 4550 50  0000 C CNN
F 2 "" H 7000 4700 50  0001 C CNN
F 3 "" H 7000 4700 50  0001 C CNN
	1    7000 4700
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR020
U 1 1 5900042C
P 7000 4400
F 0 "#PWR020" H 7000 4250 50  0001 C CNN
F 1 "VDD" H 7000 4550 50  0000 C CNN
F 2 "" H 7000 4400 50  0001 C CNN
F 3 "" H 7000 4400 50  0001 C CNN
	1    7000 4400
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 590005AC
P 8050 2800
F 0 "C6" H 8075 2900 50  0000 L CNN
F 1 "100n" H 8075 2700 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8088 2650 50  0001 C CNN
F 3 "" H 8050 2800 50  0001 C CNN
	1    8050 2800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR021
U 1 1 590005B2
P 8050 2950
F 0 "#PWR021" H 8050 2700 50  0001 C CNN
F 1 "GND" H 8050 2800 50  0000 C CNN
F 2 "" H 8050 2950 50  0001 C CNN
F 3 "" H 8050 2950 50  0001 C CNN
	1    8050 2950
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR022
U 1 1 590005B8
P 8050 2650
F 0 "#PWR022" H 8050 2500 50  0001 C CNN
F 1 "VDD" H 8050 2800 50  0000 C CNN
F 2 "" H 8050 2650 50  0001 C CNN
F 3 "" H 8050 2650 50  0001 C CNN
	1    8050 2650
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X04 J2
U 1 1 59000C1A
P 8100 2050
F 0 "J2" H 8100 2300 50  0000 C CNN
F 1 "CONN_01X04" V 8200 2050 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04" H 8100 2050 50  0001 C CNN
F 3 "" H 8100 2050 50  0001 C CNN
	1    8100 2050
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR023
U 1 1 59000CAD
P 7900 1900
F 0 "#PWR023" H 7900 1750 50  0001 C CNN
F 1 "VDD" H 7900 2050 50  0000 C CNN
F 2 "" H 7900 1900 50  0001 C CNN
F 3 "" H 7900 1900 50  0001 C CNN
	1    7900 1900
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR024
U 1 1 59000CD5
P 7900 2100
F 0 "#PWR024" H 7900 1850 50  0001 C CNN
F 1 "GND" H 7900 1950 50  0000 C CNN
F 2 "" H 7900 2100 50  0001 C CNN
F 3 "" H 7900 2100 50  0001 C CNN
	1    7900 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	7900 2000 7300 2000
Wire Wire Line
	7300 2200 7900 2200
Text Label 7300 2000 0    60   ~ 0
SWCLK
Text Label 7300 2200 0    60   ~ 0
SWDAT
$Comp
L CONN_02X04 J3
U 1 1 59000F79
P 8600 3750
F 0 "J3" H 8600 4000 50  0000 C CNN
F 1 "CONN_02X04" H 8600 3500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x04" H 8600 2550 50  0001 C CNN
F 3 "" H 8600 2550 50  0001 C CNN
	1    8600 3750
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X04 J4
U 1 1 5900100B
P 8600 4350
F 0 "J4" H 8600 4600 50  0000 C CNN
F 1 "CONN_02X04" H 8600 4100 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x04" H 8600 3150 50  0001 C CNN
F 3 "" H 8600 3150 50  0001 C CNN
	1    8600 4350
	1    0    0    -1  
$EndComp
$Comp
L CONN_02X04 J5
U 1 1 590010D9
P 8600 4950
F 0 "J5" H 8600 5200 50  0000 C CNN
F 1 "CONN_02X04" H 8600 4700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_2x04" H 8600 3750 50  0001 C CNN
F 3 "" H 8600 3750 50  0001 C CNN
	1    8600 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8850 3600 8850 4500
Wire Wire Line
	8850 4050 9050 4050
Connection ~ 8850 4050
$Comp
L GND #PWR025
U 1 1 5900118C
P 9050 4050
F 0 "#PWR025" H 9050 3800 50  0001 C CNN
F 1 "GND" H 9050 3900 50  0000 C CNN
F 2 "" H 9050 4050 50  0001 C CNN
F 3 "" H 9050 4050 50  0001 C CNN
	1    9050 4050
	1    0    0    -1  
$EndComp
Connection ~ 8850 3700
Connection ~ 8850 3800
Connection ~ 8850 3900
Connection ~ 8850 4200
Connection ~ 8850 4300
Connection ~ 8850 4400
Wire Wire Line
	8850 4800 8850 5100
Wire Wire Line
	8850 4950 9050 4950
Connection ~ 8850 4950
$Comp
L GND #PWR026
U 1 1 59001267
P 9050 4950
F 0 "#PWR026" H 9050 4700 50  0001 C CNN
F 1 "GND" H 9050 4800 50  0000 C CNN
F 2 "" H 9050 4950 50  0001 C CNN
F 3 "" H 9050 4950 50  0001 C CNN
	1    9050 4950
	1    0    0    -1  
$EndComp
Connection ~ 8850 4900
Connection ~ 8850 5000
$Comp
L VDD #PWR027
U 1 1 59001299
P 8350 4850
F 0 "#PWR027" H 8350 4700 50  0001 C CNN
F 1 "VDD" H 8350 5000 50  0000 C CNN
F 2 "" H 8350 4850 50  0001 C CNN
F 3 "" H 8350 4850 50  0001 C CNN
	1    8350 4850
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR028
U 1 1 590012C9
P 8350 5050
F 0 "#PWR028" H 8350 4900 50  0001 C CNN
F 1 "+5V" H 8350 5190 50  0000 C CNN
F 2 "" H 8350 5050 50  0001 C CNN
F 3 "" H 8350 5050 50  0001 C CNN
	1    8350 5050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8350 4800 8350 4900
Wire Wire Line
	8350 5000 8350 5100
Connection ~ 8350 5050
Connection ~ 8350 4850
Wire Wire Line
	8350 3600 8100 3600
Text Label 8100 3600 0    60   ~ 0
D0
Wire Wire Line
	8350 3700 8100 3700
Text Label 8100 3700 0    60   ~ 0
D1
Wire Wire Line
	8350 3800 8100 3800
Text Label 8100 3800 0    60   ~ 0
D2
Wire Wire Line
	8350 3900 8100 3900
Text Label 8100 3900 0    60   ~ 0
D3
Wire Wire Line
	8350 4200 8100 4200
Text Label 8100 4200 0    60   ~ 0
D4
Wire Wire Line
	8350 4300 8100 4300
Text Label 8100 4300 0    60   ~ 0
D5
Wire Wire Line
	8350 4400 8100 4400
Text Label 8100 4400 0    60   ~ 0
D6
Wire Wire Line
	8350 4500 8100 4500
Text Label 8100 4500 0    60   ~ 0
D7
$Comp
L R R1
U 1 1 59001871
P 3300 1750
F 0 "R1" V 3380 1750 50  0000 C CNN
F 1 "10K" V 3300 1750 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3230 1750 50  0001 C CNN
F 3 "" H 3300 1750 50  0001 C CNN
	1    3300 1750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR029
U 1 1 590018AC
P 3300 1900
F 0 "#PWR029" H 3300 1650 50  0001 C CNN
F 1 "GND" H 3300 1750 50  0000 C CNN
F 2 "" H 3300 1900 50  0001 C CNN
F 3 "" H 3300 1900 50  0001 C CNN
	1    3300 1900
	1    0    0    -1  
$EndComp
$Comp
L VDD #PWR030
U 1 1 59001928
P 3300 800
F 0 "#PWR030" H 3300 650 50  0001 C CNN
F 1 "VDD" H 3300 950 50  0000 C CNN
F 2 "" H 3300 800 50  0001 C CNN
F 3 "" H 3300 800 50  0001 C CNN
	1    3300 800 
	1    0    0    -1  
$EndComp
Connection ~ 3300 1600
Text Label 3450 1600 0    60   ~ 0
BOOT
$Comp
L SPST SW1
U 1 1 58FFFF87
P 3300 1200
F 0 "SW1" H 3300 900 60  0000 C CNN
F 1 "SPST" H 3300 1000 60  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_TL3342" H 3300 1150 60  0001 C CNN
F 3 "" H 3300 1150 60  0000 C CNN
	1    3300 1200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3300 1600 3750 1600
$EndSCHEMATC
