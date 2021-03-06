EESchema Schematic File Version 4
EELAYER 30 0
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
L Device:R R1
U 1 1 61812F36
P 2725 3375
F 0 "R1" V 2825 3475 50  0000 C CNN
F 1 "10k" V 2825 3275 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 2655 3375 50  0001 C CNN
F 3 "~" H 2725 3375 50  0001 C CNN
	1    2725 3375
	0    1    1    0   
$EndComp
$Comp
L Device:CP C1
U 1 1 61813FCE
P 2175 2475
F 0 "C1" V 2475 2475 50  0000 C CNN
F 1 "100n" V 2375 2475 50  0000 C CNN
F 2 "custom:FakeSMD_C_Disc" H 2213 2325 50  0001 C CNN
F 3 "~" H 2175 2475 50  0001 C CNN
	1    2175 2475
	0    -1   -1   0   
$EndComp
$Comp
L MCU_Microchip_ATtiny:ATtiny45V-10PU U1
U 1 1 61814F7D
P 1925 3175
F 0 "U1" H 1525 3725 50  0000 R CNN
F 1 "ATtiny45V-10PU" V 1525 3475 50  0000 R CNN
F 2 "custom:FakeSMD_DIP-8" H 1925 3175 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/atmel-2586-avr-8-bit-microcontroller-attiny25-attiny45-attiny85_datasheet.pdf" H 1925 3175 50  0001 C CNN
	1    1925 3175
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 618165C0
P 3175 3275
F 0 "SW1" H 3175 3575 50  0000 C CNN
F 1 "WAKE UP" H 3175 3475 50  0000 C CNN
F 2 "custom:FakeSMD_Button" H 3175 3475 50  0001 C CNN
F 3 "~" H 3175 3475 50  0001 C CNN
	1    3175 3275
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U2
U 1 1 618172F7
P 4250 2200
F 0 "U2" H 4000 2750 50  0000 C CNN
F 1 "74HC595" V 4250 2050 50  0000 C CNN
F 2 "custom:FakeSMD_DIP-16" H 4250 2200 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 4250 2200 50  0001 C CNN
	1    4250 2200
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74HC595 U3
U 1 1 61818149
P 4250 4300
F 0 "U3" H 4000 4850 50  0000 C CNN
F 1 "74HC595" V 4250 4150 50  0000 C CNN
F 2 "custom:FakeSMD_DIP-16" H 4250 4300 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 4250 4300 50  0001 C CNN
	1    4250 4300
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J1
U 1 1 6182CDB4
P 1725 1750
F 0 "J1" H 1725 1850 50  0000 C CNN
F 1 "PWR" H 1625 1700 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 1725 1750 50  0001 C CNN
F 3 "~" H 1725 1750 50  0001 C CNN
	1    1725 1750
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG01
U 1 1 6182DCD3
P 1925 1750
F 0 "#FLG01" H 1925 1825 50  0001 C CNN
F 1 "PWR_FLAG" H 2000 1825 50  0000 L CNN
F 2 "" H 1925 1750 50  0001 C CNN
F 3 "~" H 1925 1750 50  0001 C CNN
	1    1925 1750
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 6182E9DF
P 1925 1850
F 0 "#FLG02" H 1925 1925 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 1925 50  0000 R CNN
F 2 "" H 1925 1850 50  0001 C CNN
F 3 "~" H 1925 1850 50  0001 C CNN
	1    1925 1850
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 618348C7
P 1925 1850
F 0 "#PWR02" H 1925 1600 50  0001 C CNN
F 1 "GND" V 1925 1725 50  0000 R CNN
F 2 "" H 1925 1850 50  0001 C CNN
F 3 "" H 1925 1850 50  0001 C CNN
	1    1925 1850
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 61838BF1
P 2325 2475
F 0 "#PWR05" H 2325 2225 50  0001 C CNN
F 1 "GND" H 2325 2325 50  0000 C CNN
F 2 "" H 2325 2475 50  0001 C CNN
F 3 "" H 2325 2475 50  0001 C CNN
	1    2325 2475
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R2
U 1 1 6181D922
P 5050 1800
F 0 "R2" V 5000 1600 50  0000 C CNN
F 1 "270" V 5000 2000 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 1800 50  0001 C CNN
F 3 "~" H 5050 1800 50  0001 C CNN
	1    5050 1800
	0    1    1    0   
$EndComp
Wire Wire Line
	2525 3375 2575 3375
Wire Wire Line
	2025 2475 1925 2475
Wire Wire Line
	1925 2475 1925 2375
Wire Wire Line
	1925 2575 1925 2475
Connection ~ 1925 2475
Connection ~ 1925 1850
$Comp
L power:VCC #PWR01
U 1 1 61833BE3
P 1925 1750
F 0 "#PWR01" H 1925 1600 50  0001 C CNN
F 1 "VCC" V 1925 1875 50  0000 L CNN
F 2 "" H 1925 1750 50  0001 C CNN
F 3 "" H 1925 1750 50  0001 C CNN
	1    1925 1750
	0    1    1    0   
$EndComp
Connection ~ 1925 1750
$Comp
L Device:CP C2
U 1 1 6187B40B
P 4500 1500
F 0 "C2" V 4800 1500 50  0000 C CNN
F 1 "100n" V 4700 1500 50  0000 C CNN
F 2 "custom:FakeSMD_C_Disc" H 4538 1350 50  0001 C CNN
F 3 "~" H 4500 1500 50  0001 C CNN
	1    4500 1500
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR012
U 1 1 6187B411
P 4250 1400
F 0 "#PWR012" H 4250 1250 50  0001 C CNN
F 1 "VCC" H 4250 1550 50  0000 C CNN
F 2 "" H 4250 1400 50  0001 C CNN
F 3 "" H 4250 1400 50  0001 C CNN
	1    4250 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 6187B417
P 4650 1500
F 0 "#PWR016" H 4650 1250 50  0001 C CNN
F 1 "GND" H 4650 1350 50  0000 C CNN
F 2 "" H 4650 1500 50  0001 C CNN
F 3 "" H 4650 1500 50  0001 C CNN
	1    4650 1500
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 1500 4250 1500
Wire Wire Line
	4250 1500 4250 1400
Wire Wire Line
	4250 1600 4250 1500
Connection ~ 4250 1500
$Comp
L Device:CP C3
U 1 1 6187C488
P 4500 3600
F 0 "C3" V 4800 3600 50  0000 C CNN
F 1 "100n" V 4700 3600 50  0000 C CNN
F 2 "custom:FakeSMD_C_Disc" H 4538 3450 50  0001 C CNN
F 3 "~" H 4500 3600 50  0001 C CNN
	1    4500 3600
	0    -1   -1   0   
$EndComp
$Comp
L power:VCC #PWR014
U 1 1 6187C48E
P 4250 3500
F 0 "#PWR014" H 4250 3350 50  0001 C CNN
F 1 "VCC" H 4250 3650 50  0000 C CNN
F 2 "" H 4250 3500 50  0001 C CNN
F 3 "" H 4250 3500 50  0001 C CNN
	1    4250 3500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR017
U 1 1 6187C494
P 4650 3600
F 0 "#PWR017" H 4650 3350 50  0001 C CNN
F 1 "GND" H 4650 3450 50  0000 C CNN
F 2 "" H 4650 3600 50  0001 C CNN
F 3 "" H 4650 3600 50  0001 C CNN
	1    4650 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4350 3600 4250 3600
Wire Wire Line
	4250 3600 4250 3500
Wire Wire Line
	4250 3700 4250 3600
Connection ~ 4250 3600
$Comp
L power:GND #PWR015
U 1 1 6187C567
P 4250 5000
F 0 "#PWR015" H 4250 4750 50  0001 C CNN
F 1 "GND" H 4250 4850 50  0000 C CNN
F 2 "" H 4250 5000 50  0001 C CNN
F 3 "" H 4250 5000 50  0001 C CNN
	1    4250 5000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 6187CFF9
P 4250 2900
F 0 "#PWR013" H 4250 2650 50  0001 C CNN
F 1 "GND" H 4250 2750 50  0000 C CNN
F 2 "" H 4250 2900 50  0001 C CNN
F 3 "" H 4250 2900 50  0001 C CNN
	1    4250 2900
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR03
U 1 1 6183216E
P 1925 2375
F 0 "#PWR03" H 1925 2225 50  0001 C CNN
F 1 "VCC" H 1925 2525 50  0000 C CNN
F 2 "" H 1925 2375 50  0001 C CNN
F 3 "" H 1925 2375 50  0001 C CNN
	1    1925 2375
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 6183058D
P 1925 3775
F 0 "#PWR04" H 1925 3525 50  0001 C CNN
F 1 "GND" H 1925 3625 50  0000 C CNN
F 2 "" H 1925 3775 50  0001 C CNN
F 3 "" H 1925 3775 50  0001 C CNN
	1    1925 3775
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR06
U 1 1 6183A9C7
P 2875 3375
F 0 "#PWR06" H 2875 3225 50  0001 C CNN
F 1 "VCC" H 2875 3525 50  0000 C CNN
F 2 "" H 2875 3375 50  0001 C CNN
F 3 "" H 2875 3375 50  0001 C CNN
	1    2875 3375
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR07
U 1 1 61888D62
P 3375 3275
F 0 "#PWR07" H 3375 3025 50  0001 C CNN
F 1 "GND" H 3375 3125 50  0000 C CNN
F 2 "" H 3375 3275 50  0001 C CNN
F 3 "" H 3375 3275 50  0001 C CNN
	1    3375 3275
	1    0    0    -1  
$EndComp
Wire Wire Line
	2525 3275 2975 3275
Wire Wire Line
	3850 2100 3800 2100
Wire Wire Line
	3800 4200 3850 4200
$Comp
L power:VCC #PWR08
U 1 1 6189012E
P 3800 2100
F 0 "#PWR08" H 3800 1950 50  0001 C CNN
F 1 "VCC" H 3700 2250 50  0000 L CNN
F 2 "" H 3800 2100 50  0001 C CNN
F 3 "" H 3800 2100 50  0001 C CNN
	1    3800 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3850 2400 3800 2400
Wire Wire Line
	3800 4500 3850 4500
$Comp
L power:GND #PWR011
U 1 1 61891787
P 3800 4500
F 0 "#PWR011" H 3800 4250 50  0001 C CNN
F 1 "GND" H 3800 4350 50  0000 C CNN
F 2 "" H 3800 4500 50  0001 C CNN
F 3 "" H 3800 4500 50  0001 C CNN
	1    3800 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 618997F4
P 3800 2400
F 0 "#PWR09" H 3800 2150 50  0001 C CNN
F 1 "GND" H 3800 2250 50  0000 C CNN
F 2 "" H 3800 2400 50  0001 C CNN
F 3 "" H 3800 2400 50  0001 C CNN
	1    3800 2400
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR010
U 1 1 61899DD8
P 3800 4200
F 0 "#PWR010" H 3800 4050 50  0001 C CNN
F 1 "VCC" H 3700 4350 50  0000 L CNN
F 2 "" H 3800 4200 50  0001 C CNN
F 3 "" H 3800 4200 50  0001 C CNN
	1    3800 4200
	0    -1   -1   0   
$EndComp
Text Label 2525 2875 0    50   ~ 0
SRCLK
Text Label 2525 2975 0    50   ~ 0
DATA
Text Label 2525 3075 0    50   ~ 0
RCLK
Text Label 3850 2000 2    50   ~ 0
SRCLK
Text Label 3850 2300 2    50   ~ 0
RCLK
Text Label 3850 1800 2    50   ~ 0
DATA
Wire Wire Line
	4650 2700 4650 3150
Wire Wire Line
	4650 3150 3850 3150
Wire Wire Line
	3850 3150 3850 3900
Text Label 3850 4100 2    50   ~ 0
SRCLK
Text Label 3850 4400 2    50   ~ 0
RCLK
NoConn ~ 4650 4800
Wire Wire Line
	4650 1800 4900 1800
Wire Wire Line
	4650 1900 4900 1900
Wire Wire Line
	4650 2000 4900 2000
Wire Wire Line
	4650 2100 4900 2100
Wire Wire Line
	4650 2200 4900 2200
Wire Wire Line
	4650 2300 4900 2300
Wire Wire Line
	4650 2400 4900 2400
Wire Wire Line
	4650 2500 4900 2500
Wire Wire Line
	4650 3900 4900 3900
Wire Wire Line
	4650 4000 4900 4000
Wire Wire Line
	4900 4100 4650 4100
Wire Wire Line
	4650 4200 4900 4200
Wire Wire Line
	4900 4300 4650 4300
Wire Wire Line
	4650 4400 4900 4400
Wire Wire Line
	4900 4500 4650 4500
Wire Wire Line
	4650 4600 4900 4600
$Comp
L Device:R R3
U 1 1 618C9617
P 5050 1900
F 0 "R3" V 5000 1700 50  0000 C CNN
F 1 "270" V 5000 2100 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 1900 50  0001 C CNN
F 3 "~" H 5050 1900 50  0001 C CNN
	1    5050 1900
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 618CA3F9
P 5050 2000
F 0 "R4" V 5000 1800 50  0000 C CNN
F 1 "270" V 5000 2200 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 2000 50  0001 C CNN
F 3 "~" H 5050 2000 50  0001 C CNN
	1    5050 2000
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 618CA3FF
P 5050 2100
F 0 "R5" V 5000 1900 50  0000 C CNN
F 1 "270" V 5000 2300 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 2100 50  0001 C CNN
F 3 "~" H 5050 2100 50  0001 C CNN
	1    5050 2100
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 618CC89E
P 5050 2200
F 0 "R6" V 5000 2000 50  0000 C CNN
F 1 "270" V 5000 2400 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 2200 50  0001 C CNN
F 3 "~" H 5050 2200 50  0001 C CNN
	1    5050 2200
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 618CC8A4
P 5050 2300
F 0 "R7" V 5000 2100 50  0000 C CNN
F 1 "270" V 5000 2500 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 2300 50  0001 C CNN
F 3 "~" H 5050 2300 50  0001 C CNN
	1    5050 2300
	0    1    1    0   
$EndComp
$Comp
L Device:R R8
U 1 1 618CC8AA
P 5050 2400
F 0 "R8" V 5000 2200 50  0000 C CNN
F 1 "270" V 5000 2600 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 2400 50  0001 C CNN
F 3 "~" H 5050 2400 50  0001 C CNN
	1    5050 2400
	0    1    1    0   
$EndComp
$Comp
L Device:R R10
U 1 1 618D3628
P 5050 3900
F 0 "R10" V 5000 3700 50  0000 C CNN
F 1 "270" V 5000 4100 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 3900 50  0001 C CNN
F 3 "~" H 5050 3900 50  0001 C CNN
	1    5050 3900
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 618D362E
P 5050 4000
F 0 "R11" V 5000 3800 50  0000 C CNN
F 1 "270" V 5000 4200 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4000 50  0001 C CNN
F 3 "~" H 5050 4000 50  0001 C CNN
	1    5050 4000
	0    1    1    0   
$EndComp
$Comp
L Device:R R12
U 1 1 618D3634
P 5050 4100
F 0 "R12" V 5000 3900 50  0000 C CNN
F 1 "270" V 5000 4300 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4100 50  0001 C CNN
F 3 "~" H 5050 4100 50  0001 C CNN
	1    5050 4100
	0    1    1    0   
$EndComp
$Comp
L Device:R R13
U 1 1 618D363A
P 5050 4200
F 0 "R13" V 5000 4000 50  0000 C CNN
F 1 "270" V 5000 4400 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4200 50  0001 C CNN
F 3 "~" H 5050 4200 50  0001 C CNN
	1    5050 4200
	0    1    1    0   
$EndComp
$Comp
L Device:R R14
U 1 1 618D3640
P 5050 4300
F 0 "R14" V 5000 4100 50  0000 C CNN
F 1 "270" V 5000 4500 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4300 50  0001 C CNN
F 3 "~" H 5050 4300 50  0001 C CNN
	1    5050 4300
	0    1    1    0   
$EndComp
$Comp
L Device:R R15
U 1 1 618D3646
P 5050 4400
F 0 "R15" V 5000 4200 50  0000 C CNN
F 1 "270" V 5000 4600 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4400 50  0001 C CNN
F 3 "~" H 5050 4400 50  0001 C CNN
	1    5050 4400
	0    1    1    0   
$EndComp
$Comp
L Device:R R16
U 1 1 618D364C
P 5050 4500
F 0 "R16" V 5000 4300 50  0000 C CNN
F 1 "270" V 5000 4700 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4500 50  0001 C CNN
F 3 "~" H 5050 4500 50  0001 C CNN
	1    5050 4500
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 618D3652
P 5050 4600
F 0 "R17" V 5000 4400 50  0000 C CNN
F 1 "270" V 5000 4800 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 4600 50  0001 C CNN
F 3 "~" H 5050 4600 50  0001 C CNN
	1    5050 4600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 618FC2A2
P 6225 4800
F 0 "#PWR018" H 6225 4550 50  0001 C CNN
F 1 "GND" H 6225 4650 50  0000 C CNN
F 2 "" H 6225 4800 50  0001 C CNN
F 3 "" H 6225 4800 50  0001 C CNN
	1    6225 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1800 5700 1800
Wire Wire Line
	5700 1800 5700 2850
$Comp
L power:GND #PWR021
U 1 1 618FAEA8
P 6225 4100
F 0 "#PWR021" H 6225 3850 50  0001 C CNN
F 1 "GND" H 6225 3950 50  0000 C CNN
F 2 "" H 6225 4100 50  0001 C CNN
F 3 "" H 6225 4100 50  0001 C CNN
	1    6225 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NPN_CBE Q2
U 1 1 6181990C
P 6125 3900
F 0 "Q2" H 6000 4050 50  0000 L CNN
F 1 "Q_NPN_CBE" V 6350 3625 50  0000 L CNN
F 2 "custom:FakeSMD_TO-92-3" H 6325 4000 50  0001 C CNN
F 3 "~" H 6125 3900 50  0001 C CNN
	1    6125 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1900 5675 1900
Wire Wire Line
	5675 1900 5675 2750
Wire Wire Line
	5650 2650 5650 2000
Wire Wire Line
	5650 2000 5200 2000
Wire Wire Line
	5200 2100 5625 2100
Wire Wire Line
	5625 2100 5625 2550
Wire Wire Line
	5200 2200 5600 2200
Wire Wire Line
	5600 2200 5600 2450
Wire Wire Line
	5200 2400 5550 2400
Wire Wire Line
	5550 2400 5550 2250
Wire Wire Line
	5200 3900 5350 3900
Wire Wire Line
	5350 3900 5350 3550
Wire Wire Line
	5200 4000 5375 4000
Wire Wire Line
	5375 4000 5375 3450
Wire Wire Line
	5200 4100 5400 4100
Wire Wire Line
	5400 4100 5400 3350
Wire Wire Line
	5200 4200 5425 4200
Wire Wire Line
	5425 4200 5425 3250
Wire Wire Line
	5200 4300 5450 4300
Wire Wire Line
	5450 4300 5450 3150
Wire Wire Line
	5200 4400 5475 4400
Wire Wire Line
	5475 4400 5475 3050
Wire Wire Line
	5200 4500 5500 4500
Wire Wire Line
	5500 4500 5500 2950
Wire Wire Line
	5875 4600 5925 4600
$Comp
L Device:R R18
U 1 1 6186157B
P 5875 4750
F 0 "R18" H 5950 4700 50  0000 L CNN
F 1 "10k" H 5950 4750 50  0000 L CNN
F 2 "custom:FakeSMD_R" V 5805 4750 50  0001 C CNN
F 3 "~" H 5875 4750 50  0001 C CNN
	1    5875 4750
	-1   0    0    1   
$EndComp
$Comp
L Device:R R19
U 1 1 6186CF66
P 5875 4050
F 0 "R19" H 5975 4000 50  0000 L CNN
F 1 "10k" H 5975 4050 50  0000 L CNN
F 2 "custom:FakeSMD_R" V 5805 4050 50  0001 C CNN
F 3 "~" H 5875 4050 50  0001 C CNN
	1    5875 4050
	-1   0    0    1   
$EndComp
Wire Wire Line
	5875 3900 5925 3900
$Comp
L power:GND #PWR022
U 1 1 6186DE09
P 5875 4900
F 0 "#PWR022" H 5875 4650 50  0001 C CNN
F 1 "GND" H 5875 4750 50  0000 C CNN
F 2 "" H 5875 4900 50  0001 C CNN
F 3 "" H 5875 4900 50  0001 C CNN
	1    5875 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 6186E379
P 5875 4200
F 0 "#PWR023" H 5875 3950 50  0001 C CNN
F 1 "GND" H 5875 4050 50  0000 C CNN
F 2 "" H 5875 4200 50  0001 C CNN
F 3 "" H 5875 4200 50  0001 C CNN
	1    5875 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:Q_NPN_CBE Q1
U 1 1 61818C08
P 6125 4600
F 0 "Q1" H 6050 4775 50  0000 L CNN
F 1 "Q_NPN_CBE" V 6350 4325 50  0000 L CNN
F 2 "custom:FakeSMD_TO-92-3" H 6325 4700 50  0001 C CNN
F 3 "~" H 6125 4600 50  0001 C CNN
	1    6125 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8125 3550 8125 3375
Wire Wire Line
	8950 3375 8950 3550
Wire Wire Line
	8975 3350 8975 3650
Wire Wire Line
	8975 3650 8950 3650
Wire Wire Line
	9000 3325 9000 3750
Wire Wire Line
	9000 3750 8950 3750
Wire Wire Line
	9025 3850 9025 3300
Wire Wire Line
	8050 3850 8125 3850
Wire Wire Line
	8125 3950 8025 3950
Wire Wire Line
	9050 3275 9050 3950
Wire Wire Line
	9050 3950 8950 3950
Wire Wire Line
	8950 3850 9025 3850
Wire Wire Line
	8950 4050 9075 4050
Wire Wire Line
	9075 4050 9075 3250
Wire Wire Line
	8000 4050 8125 4050
Wire Wire Line
	8125 4150 7975 4150
Wire Wire Line
	7975 4150 7975 3225
Wire Wire Line
	9100 3225 9100 4150
Wire Wire Line
	9100 4150 8950 4150
$Comp
L power:GND #PWR0104
U 1 1 618EC53F
P 8125 4250
F 0 "#PWR0104" H 8125 4000 50  0001 C CNN
F 1 "GND" H 8125 4100 50  0000 C CNN
F 2 "" H 8125 4250 50  0001 C CNN
F 3 "" H 8125 4250 50  0001 C CNN
	1    8125 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 618EC545
P 8950 4250
F 0 "#PWR0103" H 8950 4000 50  0001 C CNN
F 1 "GND" H 8950 4100 50  0000 C CNN
F 2 "" H 8950 4250 50  0001 C CNN
F 3 "" H 8950 4250 50  0001 C CNN
	1    8950 4250
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 618CC8B0
P 5050 2500
F 0 "R9" V 5000 2300 50  0000 C CNN
F 1 "270" V 5000 2700 50  0000 C CNN
F 2 "custom:FakeSMD_R" V 4980 2500 50  0001 C CNN
F 3 "~" H 5050 2500 50  0001 C CNN
	1    5050 2500
	0    1    1    0   
$EndComp
Wire Wire Line
	5200 2500 5525 2500
Wire Wire Line
	5525 2500 5525 3900
Wire Wire Line
	6225 3700 6500 3700
Wire Wire Line
	6500 3700 6500 4475
Wire Wire Line
	6500 4475 6675 4475
$Comp
L power:GND #PWR019
U 1 1 61C26DFD
P 6675 4575
F 0 "#PWR019" H 6675 4325 50  0001 C CNN
F 1 "GND" H 6675 4425 50  0000 C CNN
F 2 "" H 6675 4575 50  0001 C CNN
F 3 "" H 6675 4575 50  0001 C CNN
	1    6675 4575
	0    1    1    0   
$EndComp
Wire Wire Line
	5525 3900 5875 3900
Connection ~ 5875 3900
Wire Wire Line
	5875 4600 5200 4600
Connection ~ 5875 4600
Wire Wire Line
	7525 3450 7425 3450
Wire Wire Line
	8125 3375 8950 3375
Wire Wire Line
	8075 3325 9000 3325
Wire Wire Line
	8100 3350 8975 3350
Wire Wire Line
	8050 3300 9025 3300
Wire Wire Line
	8025 3275 9050 3275
Wire Wire Line
	7975 3225 9100 3225
Wire Wire Line
	8000 3250 9075 3250
Wire Wire Line
	9200 4475 8450 4475
Wire Wire Line
	8450 4475 8450 4400
Wire Wire Line
	9225 4450 8625 4450
Wire Wire Line
	8625 4450 8625 4400
$Comp
L power:GND #PWR020
U 1 1 620CA28B
P 7475 4575
F 0 "#PWR020" H 7475 4325 50  0001 C CNN
F 1 "GND" H 7475 4425 50  0000 C CNN
F 2 "" H 7475 4575 50  0001 C CNN
F 3 "" H 7475 4575 50  0001 C CNN
	1    7475 4575
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7475 4475 8450 4475
Connection ~ 8450 4475
Wire Wire Line
	7475 4675 8625 4675
Wire Wire Line
	8625 4450 8625 4675
Connection ~ 8625 4450
Wire Wire Line
	7500 3550 7425 3550
Wire Wire Line
	7550 3350 7425 3350
Wire Wire Line
	7425 3250 7575 3250
Wire Wire Line
	7625 3050 7425 3050
Wire Wire Line
	5200 2300 5575 2300
Wire Wire Line
	5575 2300 5575 2350
Wire Wire Line
	5475 3050 6575 3050
Wire Wire Line
	5500 2950 6575 2950
Wire Wire Line
	5700 2850 6575 2850
Wire Wire Line
	5675 2750 6575 2750
Wire Wire Line
	5650 2650 6575 2650
Wire Wire Line
	5625 2550 6575 2550
Wire Wire Line
	5600 2450 6575 2450
Wire Wire Line
	5575 2350 6575 2350
Wire Wire Line
	5350 3550 6575 3550
Wire Wire Line
	5375 3450 6575 3450
Wire Wire Line
	5400 3350 6575 3350
Wire Wire Line
	5425 3250 6575 3250
Wire Wire Line
	5450 3150 6575 3150
Wire Wire Line
	5550 2250 6575 2250
$Comp
L Connector:Conn_01x14_Female J2
U 1 1 61B7E876
P 6775 2850
F 0 "J2" H 6625 3600 50  0000 L CNN
F 1 "7-seg F" V 6850 2700 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x14_P2.54mm_Vertical" H 6775 2850 50  0001 C CNN
F 3 "~" H 6775 2850 50  0001 C CNN
	1    6775 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7475 2225 7825 2225
Wire Wire Line
	7425 2550 7900 2550
Wire Wire Line
	8125 3750 8075 3750
$Comp
L custom:HDSP-523Y U5
U 1 1 618EC518
P 8425 3850
F 0 "U5" H 8275 4275 50  0000 C CNN
F 1 "HDSP-523Y" H 8625 4275 50  0000 C CNN
F 2 "custom:HDSP-523Y" H 8600 3225 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-2553EN" V 8550 4000 50  0001 C CNN
	1    8425 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7425 2450 7875 2450
Wire Wire Line
	7425 2350 7850 2350
Wire Wire Line
	8125 3650 8100 3650
Wire Wire Line
	8100 3650 8100 3350
Wire Wire Line
	7425 2750 7950 2750
Wire Wire Line
	7950 3650 8100 3650
Connection ~ 8100 3650
Connection ~ 8125 3550
Wire Wire Line
	7825 2225 7825 4150
Wire Wire Line
	7825 4150 7975 4150
Connection ~ 7975 4150
Wire Wire Line
	7850 2350 7850 4050
Wire Wire Line
	7850 4050 8000 4050
Connection ~ 8000 4050
Wire Wire Line
	7875 2450 7875 3950
Wire Wire Line
	7875 3950 8025 3950
Connection ~ 8025 3950
Wire Wire Line
	7900 2550 7900 3850
Wire Wire Line
	7900 3850 8050 3850
Connection ~ 8050 3850
Wire Wire Line
	7925 3750 8075 3750
Connection ~ 8075 3750
Wire Wire Line
	8000 3250 8000 4050
Wire Wire Line
	8025 3275 8025 3950
Wire Wire Line
	8050 3300 8050 3850
Wire Wire Line
	8075 3325 8075 3750
Wire Wire Line
	8625 2550 9225 2550
Wire Wire Line
	8625 2500 8625 2550
Wire Wire Line
	8450 2575 9200 2575
Wire Wire Line
	8450 2500 8450 2575
Wire Wire Line
	7975 1325 9100 1325
Wire Wire Line
	8025 1375 9050 1375
Wire Wire Line
	8000 1350 9075 1350
Wire Wire Line
	8050 1400 9025 1400
Wire Wire Line
	8075 1425 9000 1425
Wire Wire Line
	8100 1450 8975 1450
Wire Wire Line
	8125 1475 8950 1475
Connection ~ 8100 1750
Wire Wire Line
	7525 1750 8100 1750
Connection ~ 8075 1850
Wire Wire Line
	7550 1850 8075 1850
Connection ~ 8050 1950
Wire Wire Line
	7575 1950 8050 1950
Connection ~ 8025 2050
Wire Wire Line
	7600 2050 8025 2050
Connection ~ 8000 2150
Wire Wire Line
	7625 2150 8000 2150
Connection ~ 8125 1650
Connection ~ 7975 2250
$Comp
L power:GND #PWR0101
U 1 1 618E869C
P 8950 2350
F 0 "#PWR0101" H 8950 2100 50  0001 C CNN
F 1 "GND" H 8950 2200 50  0000 C CNN
F 2 "" H 8950 2350 50  0001 C CNN
F 3 "" H 8950 2350 50  0001 C CNN
	1    8950 2350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 618E7AEE
P 8125 2350
F 0 "#PWR0102" H 8125 2100 50  0001 C CNN
F 1 "GND" H 8125 2200 50  0000 C CNN
F 2 "" H 8125 2350 50  0001 C CNN
F 3 "" H 8125 2350 50  0001 C CNN
	1    8125 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2250 8950 2250
Wire Wire Line
	9100 1325 9100 2250
Wire Wire Line
	7975 2250 7975 1325
Wire Wire Line
	8125 2250 7975 2250
Wire Wire Line
	8000 2150 8125 2150
Wire Wire Line
	8000 1350 8000 2150
Wire Wire Line
	9075 2150 9075 1350
Wire Wire Line
	8950 2150 9075 2150
Wire Wire Line
	8950 1950 9025 1950
Wire Wire Line
	9050 2050 8950 2050
Wire Wire Line
	9050 1375 9050 2050
Wire Wire Line
	8025 2050 8025 1375
Wire Wire Line
	8125 2050 8025 2050
Wire Wire Line
	8050 1950 8125 1950
Wire Wire Line
	8050 1400 8050 1950
Wire Wire Line
	9025 1950 9025 1400
Wire Wire Line
	9000 1850 8950 1850
Wire Wire Line
	9000 1425 9000 1850
Wire Wire Line
	8075 1850 8075 1425
Wire Wire Line
	8125 1850 8075 1850
Wire Wire Line
	8975 1750 8950 1750
Wire Wire Line
	8975 1450 8975 1750
Wire Wire Line
	8100 1750 8100 1450
Wire Wire Line
	8125 1750 8100 1750
Wire Wire Line
	8950 1475 8950 1650
Wire Wire Line
	8125 1650 8125 1475
$Comp
L custom:HDSP-523Y U4
U 1 1 618A3FA6
P 8425 1950
F 0 "U4" H 8275 2375 50  0000 C CNN
F 1 "HDSP-523Y" H 8625 2375 50  0000 C CNN
F 2 "custom:HDSP-523Y" H 8600 1325 50  0001 C CNN
F 3 "https://docs.broadcom.com/docs/AV02-2553EN" V 8550 2100 50  0001 C CNN
	1    8425 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7650 2250 7975 2250
Wire Wire Line
	7625 2150 7625 3050
Wire Wire Line
	7600 2050 7600 3150
Wire Wire Line
	7575 1950 7575 3250
Wire Wire Line
	7550 1850 7550 3350
Wire Wire Line
	7525 1750 7525 3450
Wire Wire Line
	7500 1650 7500 3550
Wire Wire Line
	7500 1650 8125 1650
Wire Wire Line
	9200 2575 9200 4475
Wire Wire Line
	9225 2550 9225 4450
Wire Wire Line
	7800 3550 8125 3550
Wire Wire Line
	7950 2750 7950 3650
Wire Wire Line
	6225 4400 6425 4400
Wire Wire Line
	6425 4400 6425 4675
Wire Wire Line
	6425 4675 6675 4675
$Comp
L Connector:Conn_01x03_Male J5
U 1 1 61DA6994
P 7275 4575
F 0 "J5" H 7400 4800 50  0000 C CNN
F 1 "NPNs M" H 7375 4375 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 7275 4575 50  0001 C CNN
F 3 "~" H 7275 4575 50  0001 C CNN
	1    7275 4575
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J3
U 1 1 61B7D607
P 6875 4575
F 0 "J3" H 6725 4800 50  0000 L CNN
F 1 "NPNs F" H 6675 4375 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 6875 4575 50  0001 C CNN
F 3 "~" H 6875 4575 50  0001 C CNN
	1    6875 4575
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x03_Female J6
U 1 1 61874121
P 3300 2525
F 0 "J6" H 3150 2750 50  0000 L CNN
F 1 "DHT11" H 3100 2325 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 3300 2525 50  0001 C CNN
F 3 "~" H 3300 2525 50  0001 C CNN
	1    3300 2525
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR024
U 1 1 61875B9E
P 3025 2525
F 0 "#PWR024" H 3025 2375 50  0001 C CNN
F 1 "VCC" H 3025 2675 50  0000 C CNN
F 2 "" H 3025 2525 50  0001 C CNN
F 3 "" H 3025 2525 50  0001 C CNN
	1    3025 2525
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR025
U 1 1 6187625D
P 3100 2425
F 0 "#PWR025" H 3100 2175 50  0001 C CNN
F 1 "GND" H 3100 2275 50  0000 C CNN
F 2 "" H 3100 2425 50  0001 C CNN
F 3 "" H 3100 2425 50  0001 C CNN
	1    3100 2425
	0    1    1    0   
$EndComp
Wire Wire Line
	2900 2625 3100 2625
Wire Wire Line
	3025 2525 3100 2525
Wire Wire Line
	2900 3175 2525 3175
Wire Wire Line
	2900 2625 2900 3175
$Comp
L Connector:Conn_01x14_Male J4
U 1 1 61DA18C6
P 7225 2850
F 0 "J4" H 7350 3600 50  0000 C CNN
F 1 "7-seg M" V 7150 2875 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x14_P2.54mm_Vertical" H 7225 2850 50  0001 C CNN
F 3 "~" H 7225 2850 50  0001 C CNN
	1    7225 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 3550 7800 2850
Wire Wire Line
	7800 2850 7425 2850
Wire Wire Line
	7925 2650 7925 3750
Wire Wire Line
	7425 2650 7925 2650
Wire Wire Line
	7600 3150 7425 3150
Wire Wire Line
	7650 2250 7650 2950
Wire Wire Line
	7425 2950 7650 2950
Wire Wire Line
	7425 2250 7475 2250
Wire Wire Line
	7475 2250 7475 2225
Wire Notes Line
	1300 1550 1300 3975
Wire Notes Line
	1300 3975 3475 3975
Wire Notes Line
	3475 3975 3475 1550
Wire Notes Line
	3475 1550 1300 1550
Wire Notes Line
	7075 1275 7075 4850
Wire Notes Line
	7075 4850 9275 4850
Wire Notes Line
	9275 4850 9275 1275
Wire Notes Line
	9275 1275 7075 1275
Wire Notes Line
	6975 1125 6975 5225
Wire Notes Line
	6975 5225 3575 5225
Wire Notes Line
	3575 5225 3575 1125
Wire Notes Line
	3575 1125 6975 1125
Text Notes 7900 1225 0    50   ~ 0
7-segments as display
Text Notes 4650 1075 0    50   ~ 0
74HC595s for display shiftout
Text Notes 1900 1500 0    50   ~ 0
??C to do all the magic
$EndSCHEMATC
