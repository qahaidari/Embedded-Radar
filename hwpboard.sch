EESchema Schematic File Version 4
LIBS:hwpboard-cache
EELAYER 26 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "Your Module here"
Date ""
Rev ""
Comp "Your team here"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 5B990E83
P 950 1250
F 0 "J1" H 1005 1567 50  0000 C CNN
F 1 "Barrel_Jack_Switch" H 1005 1476 50  0000 C CNN
F 2 "HWP:BarrelJack_Horizontal_CustomHWP" H 1000 1210 50  0001 C CNN
F 3 "~" H 1000 1210 50  0001 C CNN
	1    950  1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5B991004
P 3350 1500
F 0 "#PWR01" H 3350 1250 50  0001 C CNN
F 1 "GND" H 3355 1327 50  0000 C CNN
F 2 "" H 3350 1500 50  0001 C CNN
F 3 "" H 3350 1500 50  0001 C CNN
	1    3350 1500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 5BAB792F
P 10250 10750
F 0 "J3" H 10330 10792 50  0000 L CNN
F 1 "Conn_01x01" H 10330 10701 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 10250 10750 50  0001 C CNN
F 3 "~" H 10250 10750 50  0001 C CNN
	1    10250 10750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J5
U 1 1 5BAB7A7C
P 11100 10750
F 0 "J5" H 11180 10792 50  0000 L CNN
F 1 "Conn_01x01" H 11180 10701 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 11100 10750 50  0001 C CNN
F 3 "~" H 11100 10750 50  0001 C CNN
	1    11100 10750
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 5BAB7AA2
P 10250 10950
F 0 "J4" H 10330 10992 50  0000 L CNN
F 1 "Conn_01x01" H 10330 10901 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 10250 10950 50  0001 C CNN
F 3 "~" H 10250 10950 50  0001 C CNN
	1    10250 10950
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J6
U 1 1 5BAB7AE2
P 11100 10950
F 0 "J6" H 11180 10992 50  0000 L CNN
F 1 "Conn_01x01" H 11180 10901 50  0000 L CNN
F 2 "HWP:MountingHole_4.1mm_M4_ISO14580_Pad" H 11100 10950 50  0001 C CNN
F 3 "~" H 11100 10950 50  0001 C CNN
	1    11100 10950
	1    0    0    -1  
$EndComp
Text Notes 10500 10550 0    50   ~ 0
Mounting Holes
Wire Notes Line
	9950 10450 9950 11100
Wire Notes Line
	9950 11100 11700 11100
Wire Notes Line
	11700 11100 11700 10450
Wire Notes Line
	11700 10450 9950 10450
Wire Wire Line
	1350 1350 1250 1350
NoConn ~ 10050 10750
NoConn ~ 10050 10950
NoConn ~ 10900 10750
NoConn ~ 10900 10950
$Comp
L power:+5V #PWR02
U 1 1 5BAE8089
P 3350 1150
F 0 "#PWR02" H 3350 1000 50  0001 C CNN
F 1 "+5V" H 3365 1323 50  0000 C CNN
F 2 "" H 3350 1150 50  0001 C CNN
F 3 "" H 3350 1150 50  0001 C CNN
	1    3350 1150
	1    0    0    -1  
$EndComp
NoConn ~ 1250 1250
$Comp
L HWP:NUCLEO64 NUCBRD1
U 1 1 5BD1AA31
P 11450 3500
F 0 "NUCBRD1" H 11400 1364 50  0000 C CNN
F 1 "NUCLEO64" H 11400 1273 50  0000 C CNN
F 2 "HWP:ST_Morpho_Connector_STLink" H 11350 3900 50  0001 C CNN
F 3 "" H 11450 4000 50  0001 C CNN
	1    11450 3500
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:MC33079 U1
U 1 1 5BD1B0AB
P 3500 3850
F 0 "U1" H 3500 3483 50  0000 C CNN
F 1 "MC33079" H 3500 3574 50  0000 C CNN
F 2 "HWP:SOIC-16_3.9x9.9mm_P1.27mm" H 3450 3950 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/MC33078-D.PDF" H 3550 4050 50  0001 C CNN
	1    3500 3850
	1    0    0    1   
$EndComp
$Comp
L Device:R R3
U 1 1 5BD1B18B
P 2450 3750
F 0 "R3" V 2243 3750 50  0000 C CNN
F 1 "4.7k" V 2334 3750 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2380 3750 50  0001 C CNN
F 3 "~" H 2450 3750 50  0001 C CNN
	1    2450 3750
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 5BD1B21F
P 2600 4550
F 0 "R4" V 2807 4550 50  0000 C CNN
F 1 "1k" V 2716 4550 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2530 4550 50  0001 C CNN
F 3 "~" H 2600 4550 50  0001 C CNN
	1    2600 4550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R6
U 1 1 5BD1BE1A
P 4800 3850
F 0 "R6" V 5007 3850 50  0000 C CNN
F 1 "4.7k" V 4916 3850 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 4730 3850 50  0001 C CNN
F 3 "~" H 4800 3850 50  0001 C CNN
	1    4800 3850
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R1
U 1 1 5BD1BE68
P 2050 4300
F 0 "R1" H 2120 4346 50  0000 L CNN
F 1 "10k" H 2120 4255 50  0000 L CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1980 4300 50  0001 C CNN
F 3 "~" H 2050 4300 50  0001 C CNN
	1    2050 4300
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5BD1BEDA
P 2050 4850
F 0 "R2" H 2120 4896 50  0000 L CNN
F 1 "10k" H 2120 4805 50  0000 L CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 1980 4850 50  0001 C CNN
F 3 "~" H 2050 4850 50  0001 C CNN
	1    2050 4850
	1    0    0    -1  
$EndComp
$Comp
L Device:R R7
U 1 1 5BD1BF10
P 5500 2900
F 0 "R7" V 5707 2900 50  0000 C CNN
F 1 "150k" V 5616 2900 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5430 2900 50  0001 C CNN
F 3 "~" H 5500 2900 50  0001 C CNN
	1    5500 2900
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R8
U 1 1 5BD1BF4C
P 6400 4000
F 0 "R8" H 6470 4046 50  0000 L CNN
F 1 "300" H 6470 3955 50  0000 L CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6330 4000 50  0001 C CNN
F 3 "~" H 6400 4000 50  0001 C CNN
	1    6400 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5BD1BFCE
P 3550 2900
F 0 "R5" V 3757 2900 50  0000 C CNN
F 1 "150k" V 3666 2900 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3480 2900 50  0001 C CNN
F 3 "~" H 3550 2900 50  0001 C CNN
	1    3550 2900
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C1
U 1 1 5BD1C159
P 1600 4850
F 0 "C1" H 1485 4804 50  0000 R CNN
F 1 "10u" H 1485 4895 50  0000 R CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1638 4700 50  0001 C CNN
F 3 "~" H 1600 4850 50  0001 C CNN
	1    1600 4850
	-1   0    0    1   
$EndComp
$Comp
L Device:C C2
U 1 1 5BD1C205
P 1850 3750
F 0 "C2" V 1598 3750 50  0000 C CNN
F 1 "10u" V 1689 3750 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1888 3600 50  0001 C CNN
F 3 "~" H 1850 3750 50  0001 C CNN
	1    1850 3750
	0    1    1    0   
$EndComp
$Comp
L Device:C C3
U 1 1 5BD1C883
P 3500 3250
F 0 "C3" V 3248 3250 50  0000 C CNN
F 1 "1n" V 3339 3250 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3538 3100 50  0001 C CNN
F 3 "~" H 3500 3250 50  0001 C CNN
	1    3500 3250
	0    1    1    0   
$EndComp
$Comp
L Device:C C4
U 1 1 5BD1C999
P 4300 3850
F 0 "C4" V 4048 3850 50  0000 C CNN
F 1 "10u" V 4139 3850 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4338 3700 50  0001 C CNN
F 3 "~" H 4300 3850 50  0001 C CNN
	1    4300 3850
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 5BD1CA4E
P 5500 3250
F 0 "C5" V 5248 3250 50  0000 C CNN
F 1 "1n" V 5339 3250 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5538 3100 50  0001 C CNN
F 3 "~" H 5500 3250 50  0001 C CNN
	1    5500 3250
	0    1    1    0   
$EndComp
$Comp
L Amplifier_Operational:MC33079 U1
U 2 1 5BD1CB48
P 5600 3850
F 0 "U1" H 5600 3483 50  0000 C CNN
F 1 "MC33079" H 5600 3574 50  0000 C CNN
F 2 "HWP:SOIC-16_3.9x9.9mm_P1.27mm" H 5550 3950 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/MC33078-D.PDF" H 5650 4050 50  0001 C CNN
	2    5600 3850
	1    0    0    1   
$EndComp
Wire Wire Line
	3200 3750 3050 3750
Wire Wire Line
	2300 3750 2000 3750
Wire Wire Line
	3200 3950 3200 4550
Wire Wire Line
	3200 4550 2750 4550
Wire Wire Line
	2450 4600 2450 4550
Wire Wire Line
	2450 4550 2050 4550
Wire Wire Line
	2050 4550 2050 4450
Connection ~ 2450 4550
Wire Wire Line
	2050 4700 2050 4550
Connection ~ 2050 4550
Wire Wire Line
	1600 4700 1600 4550
Wire Wire Line
	1600 4550 2050 4550
Wire Wire Line
	2050 5000 1800 5000
Wire Wire Line
	3350 3250 3050 3250
Wire Wire Line
	3050 3250 3050 3750
Connection ~ 3050 3750
Wire Wire Line
	3050 3750 2600 3750
Wire Wire Line
	3650 3250 3950 3250
Wire Wire Line
	3950 3250 3950 3850
Wire Wire Line
	3800 3850 3950 3850
Connection ~ 3950 3850
Wire Wire Line
	3950 3850 4150 3850
Wire Wire Line
	3400 2900 3050 2900
Wire Wire Line
	3050 2900 3050 3250
Connection ~ 3050 3250
Wire Wire Line
	3700 2900 3950 2900
Wire Wire Line
	3950 2900 3950 3250
Connection ~ 3950 3250
Wire Wire Line
	4650 3850 4450 3850
Wire Wire Line
	4950 3850 5150 3850
Wire Wire Line
	5150 3850 5150 3750
Wire Wire Line
	5150 3750 5300 3750
Wire Wire Line
	5300 4550 3200 4550
Connection ~ 3200 4550
Wire Wire Line
	5300 3950 5300 4550
Wire Wire Line
	5900 3850 6000 3850
Wire Wire Line
	5350 3250 5150 3250
Wire Wire Line
	5150 3250 5150 3750
Connection ~ 5150 3750
Wire Wire Line
	5650 3250 6000 3250
Wire Wire Line
	5650 2900 6000 2900
Wire Wire Line
	6000 2900 6000 3250
Connection ~ 6000 3250
Wire Wire Line
	6000 3250 6000 3850
Wire Wire Line
	5350 2900 5150 2900
Wire Wire Line
	5150 2900 5150 3250
Connection ~ 5150 3250
$Comp
L power:+5V #PWR03
U 1 1 5BD23F0C
P 15150 3800
F 0 "#PWR03" H 15150 3650 50  0001 C CNN
F 1 "+5V" H 15165 3973 50  0000 C CNN
F 2 "" H 15150 3800 50  0001 C CNN
F 3 "" H 15150 3800 50  0001 C CNN
	1    15150 3800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5BD23F97
P 1800 5200
F 0 "#PWR04" H 1800 4950 50  0001 C CNN
F 1 "GND" H 1805 5027 50  0000 C CNN
F 2 "" H 1800 5200 50  0001 C CNN
F 3 "" H 1800 5200 50  0001 C CNN
	1    1800 5200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 5000 1800 5200
Connection ~ 1800 5000
Wire Wire Line
	1800 5000 1600 5000
$Comp
L Display_Character:WC1602A DS1
U 1 1 5BD283A8
P 14150 3250
F 0 "DS1" H 14150 2272 50  0000 C CNN
F 1 "WC1602A" H 14150 2363 50  0000 C CNN
F 2 "HWP:WC1602A_no_mount" H 14150 2350 50  0001 C CIN
F 3 "http://www.wincomlcd.com/pdf/WC1602A-SFYLYHTC06.pdf" H 14850 3250 50  0001 C CNN
	1    14150 3250
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:MC33079 U1
U 4 1 5BD2B8BC
P 9350 3950
F 0 "U1" H 9350 3583 50  0000 C CNN
F 1 "MC33079" H 9350 3674 50  0000 C CNN
F 2 "HWP:SOIC-16_3.9x9.9mm_P1.27mm" H 9300 4050 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/MC33078-D.PDF" H 9400 4150 50  0001 C CNN
	4    9350 3950
	1    0    0    1   
$EndComp
$Comp
L Amplifier_Operational:MC33079 U1
U 5 1 5BD2B92E
P 3200 5350
F 0 "U1" H 3158 5304 50  0000 L CNN
F 1 "MC33079" H 3158 5395 50  0000 L CNN
F 2 "" H 3150 5450 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/MC33078-D.PDF" H 3250 5550 50  0001 C CNN
	5    3200 5350
	1    0    0    1   
$EndComp
$Comp
L Device:R R9
U 1 1 5BD2E4A1
P 6400 4600
F 0 "R9" H 6470 4646 50  0000 L CNN
F 1 "2k" H 6470 4555 50  0000 L CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6330 4600 50  0001 C CNN
F 3 "~" H 6400 4600 50  0001 C CNN
	1    6400 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R10
U 1 1 5BD2E547
P 7050 4250
F 0 "R10" V 6843 4250 50  0000 C CNN
F 1 "47k" V 6934 4250 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6980 4250 50  0001 C CNN
F 3 "~" H 7050 4250 50  0001 C CNN
	1    7050 4250
	0    1    1    0   
$EndComp
$Comp
L Device:R R11
U 1 1 5BD2E5E1
P 7600 3700
F 0 "R11" H 7670 3746 50  0000 L CNN
F 1 "47k" H 7670 3655 50  0000 L CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7530 3700 50  0001 C CNN
F 3 "~" H 7600 3700 50  0001 C CNN
	1    7600 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R12
U 1 1 5BD2E643
P 8000 3700
F 0 "R12" H 8070 3746 50  0000 L CNN
F 1 "47k" H 8070 3655 50  0000 L CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7930 3700 50  0001 C CNN
F 3 "~" H 8000 3700 50  0001 C CNN
	1    8000 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5BD2E697
P 8400 4250
F 0 "R13" V 8193 4250 50  0000 C CNN
F 1 "47k" V 8284 4250 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8330 4250 50  0001 C CNN
F 3 "~" H 8400 4250 50  0001 C CNN
	1    8400 4250
	0    1    1    0   
$EndComp
$Comp
L Device:C C6
U 1 1 5BD2E8F9
P 7100 3400
F 0 "C6" V 6848 3400 50  0000 C CNN
F 1 "33n" V 6939 3400 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7138 3250 50  0001 C CNN
F 3 "~" H 7100 3400 50  0001 C CNN
	1    7100 3400
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 5BD2E9AD
P 8350 3400
F 0 "C9" V 8098 3400 50  0000 C CNN
F 1 "33n" V 8189 3400 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8388 3250 50  0001 C CNN
F 3 "~" H 8350 3400 50  0001 C CNN
	1    8350 3400
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 5BD2FD07
P 7550 4600
F 0 "C7" H 7665 4646 50  0000 L CNN
F 1 "33n" H 7665 4555 50  0000 L CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7588 4450 50  0001 C CNN
F 3 "~" H 7550 4600 50  0001 C CNN
	1    7550 4600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 5BD2FD6D
P 8000 4600
F 0 "C8" H 8115 4646 50  0000 L CNN
F 1 "33n" H 8115 4555 50  0000 L CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8038 4450 50  0001 C CNN
F 3 "~" H 8000 4600 50  0001 C CNN
	1    8000 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3850 6400 3850
Connection ~ 6000 3850
Wire Wire Line
	6400 4150 6400 4250
Wire Wire Line
	6900 4250 6750 4250
Connection ~ 6400 4250
Wire Wire Line
	6400 4250 6400 4450
Wire Wire Line
	6750 3400 6950 3400
Connection ~ 6750 4250
Wire Wire Line
	6750 4250 6400 4250
Wire Wire Line
	7250 3400 7600 3400
Wire Wire Line
	7600 3550 7600 3400
Connection ~ 7600 3400
Wire Wire Line
	7600 3400 8000 3400
Wire Wire Line
	8000 3550 8000 3400
Connection ~ 8000 3400
Wire Wire Line
	8000 3400 8200 3400
Wire Wire Line
	7550 4250 7550 4450
Wire Wire Line
	8000 4250 8000 4450
Wire Wire Line
	8250 4250 8000 4250
Connection ~ 8000 4250
Wire Wire Line
	8500 3400 8700 3400
Wire Wire Line
	8700 3400 8700 4250
Wire Wire Line
	8700 4250 8550 4250
Wire Wire Line
	8700 4250 8950 4250
Wire Wire Line
	8950 4250 8950 4050
Wire Wire Line
	8950 4050 9050 4050
Connection ~ 8700 4250
Wire Wire Line
	7600 3850 8000 3850
Wire Wire Line
	8000 3850 8900 3850
Connection ~ 8000 3850
Wire Wire Line
	9650 3950 9650 3400
Wire Wire Line
	9650 3400 8900 3400
Wire Wire Line
	8900 3400 8900 3850
Connection ~ 8900 3850
Wire Wire Line
	8900 3850 9050 3850
$Comp
L power:GND #PWR0101
U 1 1 5BD4CB78
P 6400 4850
F 0 "#PWR0101" H 6400 4600 50  0001 C CNN
F 1 "GND" H 6405 4677 50  0000 C CNN
F 2 "" H 6400 4850 50  0001 C CNN
F 3 "" H 6400 4850 50  0001 C CNN
	1    6400 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5BD4CBC7
P 7550 4850
F 0 "#PWR0102" H 7550 4600 50  0001 C CNN
F 1 "GND" H 7555 4677 50  0000 C CNN
F 2 "" H 7550 4850 50  0001 C CNN
F 3 "" H 7550 4850 50  0001 C CNN
	1    7550 4850
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5BD4CC16
P 8000 4850
F 0 "#PWR0103" H 8000 4600 50  0001 C CNN
F 1 "GND" H 8005 4677 50  0000 C CNN
F 2 "" H 8000 4850 50  0001 C CNN
F 3 "" H 8000 4850 50  0001 C CNN
	1    8000 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 4750 6400 4850
Wire Wire Line
	7550 4750 7550 4850
Wire Wire Line
	8000 4750 8000 4850
Wire Wire Line
	12500 2650 13400 2650
Wire Wire Line
	13400 2650 13400 2750
Wire Wire Line
	13400 2750 13750 2750
Wire Wire Line
	12500 2750 13300 2750
Wire Wire Line
	13300 2750 13300 2850
Wire Wire Line
	13300 2850 13750 2850
Wire Wire Line
	12500 2850 13200 2850
Wire Wire Line
	13200 2850 13200 2950
Wire Wire Line
	13200 2950 13750 2950
Wire Wire Line
	12500 2950 13100 2950
Wire Wire Line
	13100 2950 13100 3050
Wire Wire Line
	13100 3050 13750 3050
Wire Wire Line
	12500 3050 13000 3050
Wire Wire Line
	13000 3050 13000 3150
Wire Wire Line
	13000 3150 13750 3150
Wire Wire Line
	12500 3150 12900 3150
Wire Wire Line
	12900 3150 12900 3250
Wire Wire Line
	12900 3250 13750 3250
Wire Wire Line
	12500 3250 12750 3250
Wire Wire Line
	12750 3350 13750 3350
Wire Wire Line
	12750 3250 12750 3350
$Comp
L Device:LED D1
U 1 1 5BD9992C
P 13900 4650
F 0 "D1" H 13892 4395 50  0000 C CNN
F 1 "RED" H 13892 4486 50  0000 C CNN
F 2 "HWP:LED_D3.0mm" H 13900 4650 50  0001 C CNN
F 3 "~" H 13900 4650 50  0001 C CNN
	1    13900 4650
	-1   0    0    1   
$EndComp
$Comp
L Device:LED D3
U 1 1 5BD99C12
P 13900 5350
F 0 "D3" H 13892 5095 50  0000 C CNN
F 1 "RED" H 13892 5186 50  0000 C CNN
F 2 "HWP:LED_D3.0mm" H 13900 5350 50  0001 C CNN
F 3 "~" H 13900 5350 50  0001 C CNN
	1    13900 5350
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5BDA4C73
P 15400 5550
F 0 "#PWR0104" H 15400 5300 50  0001 C CNN
F 1 "GND" H 15405 5377 50  0000 C CNN
F 2 "" H 15400 5550 50  0001 C CNN
F 3 "" H 15400 5550 50  0001 C CNN
	1    15400 5550
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 5BDC1EAE
P 8000 6000
F 0 "SW1" V 7954 6148 50  0000 L CNN
F 1 "SW_Push" V 8045 6148 50  0000 L CNN
F 2 "HWP:SW_PUSH_6mm" H 8000 6200 50  0001 C CNN
F 3 "" H 8000 6200 50  0001 C CNN
	1    8000 6000
	0    1    1    0   
$EndComp
$Comp
L Device:LED D2
U 1 1 5BD99AFE
P 13900 5000
F 0 "D2" H 13892 4745 50  0000 C CNN
F 1 "RED" H 13892 4836 50  0000 C CNN
F 2 "HWP:LED_D3.0mm" H 13900 5000 50  0001 C CNN
F 3 "~" H 13900 5000 50  0001 C CNN
	1    13900 5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	9650 3950 9950 3950
Wire Wire Line
	9950 3950 9950 3750
Wire Wire Line
	9950 3750 10300 3750
Connection ~ 9650 3950
Wire Wire Line
	10000 3850 10300 3850
$Comp
L Switch:SW_Push SW2
U 1 1 5BE5AAF5
P 8000 7000
F 0 "SW2" V 7954 7148 50  0000 L CNN
F 1 "SW_Push" V 8045 7148 50  0000 L CNN
F 2 "HWP:SW_PUSH_6mm" H 8000 7200 50  0001 C CNN
F 3 "" H 8000 7200 50  0001 C CNN
	1    8000 7000
	0    1    1    0   
$EndComp
Wire Wire Line
	10300 3950 10100 3950
Wire Wire Line
	10000 3850 10000 5550
$Comp
L Device:R R14
U 1 1 5BE9D3C8
P 7700 5550
F 0 "R14" V 7907 5550 50  0000 C CNN
F 1 "47k" V 7816 5550 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7630 5550 50  0001 C CNN
F 3 "~" H 7700 5550 50  0001 C CNN
	1    7700 5550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R16
U 1 1 5BEA4C09
P 8400 5550
F 0 "R16" V 8607 5550 50  0000 C CNN
F 1 "30k" V 8516 5550 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8330 5550 50  0001 C CNN
F 3 "~" H 8400 5550 50  0001 C CNN
	1    8400 5550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R15
U 1 1 5BEAC442
P 7700 6650
F 0 "R15" V 7493 6650 50  0000 C CNN
F 1 "47k" V 7584 6650 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7630 6650 50  0001 C CNN
F 3 "~" H 7700 6650 50  0001 C CNN
	1    7700 6650
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 5BEAC51A
P 8500 6650
F 0 "R17" V 8293 6650 50  0000 C CNN
F 1 "30k" V 8384 6650 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8430 6650 50  0001 C CNN
F 3 "~" H 8500 6650 50  0001 C CNN
	1    8500 6650
	0    1    1    0   
$EndComp
$Comp
L Device:C C11
U 1 1 5BEAC5EF
P 9000 5700
F 0 "C11" H 9115 5746 50  0000 L CNN
F 1 "100n" H 9115 5655 50  0000 L CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 9038 5550 50  0001 C CNN
F 3 "~" H 9000 5700 50  0001 C CNN
	1    9000 5700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5BEAC6C3
P 8950 6850
F 0 "C10" H 9065 6896 50  0000 L CNN
F 1 "100n" H 9065 6805 50  0000 L CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8988 6700 50  0001 C CNN
F 3 "~" H 8950 6850 50  0001 C CNN
	1    8950 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	8550 5550 9000 5550
Wire Wire Line
	8000 5800 8000 5550
Wire Wire Line
	7850 5550 8000 5550
Connection ~ 8000 5550
Wire Wire Line
	8000 5550 8250 5550
Wire Wire Line
	9000 5850 9000 6250
Wire Wire Line
	9000 6250 8750 6250
Wire Wire Line
	8000 6250 8000 6200
Wire Wire Line
	7850 6650 8000 6650
Wire Wire Line
	8000 6800 8000 6650
Connection ~ 8000 6650
Wire Wire Line
	8650 6650 8950 6650
Wire Wire Line
	8950 6650 8950 6700
Wire Wire Line
	8950 7000 8950 7300
Wire Wire Line
	8950 7300 8450 7300
Wire Wire Line
	8000 7300 8000 7200
Wire Wire Line
	9000 5550 10000 5550
Connection ~ 9000 5550
Wire Wire Line
	10100 6650 8950 6650
Wire Wire Line
	10100 3950 10100 6650
Connection ~ 8950 6650
Wire Wire Line
	8000 6650 8350 6650
$Comp
L power:GND #PWR0105
U 1 1 5BF43537
P 8450 7500
F 0 "#PWR0105" H 8450 7250 50  0001 C CNN
F 1 "GND" H 8455 7327 50  0000 C CNN
F 2 "" H 8450 7500 50  0001 C CNN
F 3 "" H 8450 7500 50  0001 C CNN
	1    8450 7500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5BF43607
P 8750 6350
F 0 "#PWR0106" H 8750 6100 50  0001 C CNN
F 1 "GND" H 8755 6177 50  0000 C CNN
F 2 "" H 8750 6350 50  0001 C CNN
F 3 "" H 8750 6350 50  0001 C CNN
	1    8750 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 6350 8750 6250
Connection ~ 8750 6250
Wire Wire Line
	8750 6250 8000 6250
Wire Wire Line
	8450 7500 8450 7300
Connection ~ 8450 7300
Wire Wire Line
	8450 7300 8000 7300
$Comp
L power:+3.3V #PWR0107
U 1 1 5BF553A3
P 7450 5550
F 0 "#PWR0107" H 7450 5400 50  0001 C CNN
F 1 "+3.3V" V 7465 5678 50  0000 L CNN
F 2 "" H 7450 5550 50  0001 C CNN
F 3 "" H 7450 5550 50  0001 C CNN
	1    7450 5550
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0108
U 1 1 5BF5540D
P 7400 6650
F 0 "#PWR0108" H 7400 6500 50  0001 C CNN
F 1 "+3.3V" V 7415 6778 50  0000 L CNN
F 2 "" H 7400 6650 50  0001 C CNN
F 3 "" H 7400 6650 50  0001 C CNN
	1    7400 6650
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0109
U 1 1 5BF8D891
P 11100 1350
F 0 "#PWR0109" H 11100 1200 50  0001 C CNN
F 1 "+3.3V" H 11115 1523 50  0000 C CNN
F 2 "" H 11100 1350 50  0001 C CNN
F 3 "" H 11100 1350 50  0001 C CNN
	1    11100 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	11100 1350 11100 1550
Wire Wire Line
	7450 5550 7550 5550
Wire Wire Line
	7400 6650 7550 6650
NoConn ~ 12500 3450
NoConn ~ 12500 3350
$Comp
L Device:R R18
U 1 1 5BE59B38
P 14500 4650
F 0 "R18" V 14293 4650 50  0000 C CNN
F 1 "180" V 14384 4650 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 14430 4650 50  0001 C CNN
F 3 "~" H 14500 4650 50  0001 C CNN
	1    14500 4650
	0    1    1    0   
$EndComp
$Comp
L Device:R R20
U 1 1 5BE59D1A
P 14500 5000
F 0 "R20" V 14293 5000 50  0000 C CNN
F 1 "180" V 14384 5000 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 14430 5000 50  0001 C CNN
F 3 "~" H 14500 5000 50  0001 C CNN
	1    14500 5000
	0    1    1    0   
$EndComp
$Comp
L Device:R R21
U 1 1 5BE59DDC
P 14500 5350
F 0 "R21" V 14293 5350 50  0000 C CNN
F 1 "180" V 14384 5350 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 14430 5350 50  0001 C CNN
F 3 "~" H 14500 5350 50  0001 C CNN
	1    14500 5350
	0    1    1    0   
$EndComp
Wire Wire Line
	14050 4650 14350 4650
Wire Wire Line
	14050 5000 14350 5000
Wire Wire Line
	14350 5350 14050 5350
Wire Wire Line
	14650 5350 15050 5350
Wire Wire Line
	15050 5550 15400 5550
Wire Wire Line
	14650 5000 15050 5000
Connection ~ 15050 5350
Wire Wire Line
	15050 5350 15050 5550
Wire Wire Line
	14650 4650 15050 4650
Wire Wire Line
	15050 4650 15050 5000
Connection ~ 15050 5000
Wire Wire Line
	15050 5000 15050 5350
NoConn ~ 10300 3150
NoConn ~ 10300 3250
NoConn ~ 10300 3400
NoConn ~ 10300 3500
NoConn ~ 10300 4050
NoConn ~ 10300 4150
NoConn ~ 10300 4250
NoConn ~ 10300 4350
NoConn ~ 10300 4450
NoConn ~ 10300 4550
NoConn ~ 10300 4650
NoConn ~ 10300 4750
NoConn ~ 10300 4850
NoConn ~ 10300 4950
NoConn ~ 10300 5050
NoConn ~ 10300 5150
NoConn ~ 12500 2050
NoConn ~ 12500 2150
NoConn ~ 12500 2250
NoConn ~ 12500 2350
NoConn ~ 12500 2550
NoConn ~ 12500 4050
NoConn ~ 12500 4150
NoConn ~ 12500 4250
NoConn ~ 12500 4350
NoConn ~ 12500 4450
NoConn ~ 12500 4550
NoConn ~ 12500 4650
NoConn ~ 12500 4750
NoConn ~ 12500 4850
NoConn ~ 12500 4950
Wire Wire Line
	14100 2000 14100 2450
Wire Wire Line
	14100 2450 14150 2450
$Comp
L power:GND #PWR014
U 1 1 5BF84885
P 14100 2000
F 0 "#PWR014" H 14100 1750 50  0001 C CNN
F 1 "GND" H 14105 1827 50  0000 C CNN
F 2 "" H 14100 2000 50  0001 C CNN
F 3 "" H 14100 2000 50  0001 C CNN
	1    14100 2000
	-1   0    0    1   
$EndComp
Wire Wire Line
	6750 4250 6750 3400
Wire Wire Line
	6750 3400 6650 3400
Connection ~ 6750 3400
Text GLabel 6650 3400 0    50   Input ~ 0
Sig_unfiltered
Wire Wire Line
	10200 3650 10300 3650
Text GLabel 10200 3650 0    50   Input ~ 0
Sig_unfiltered
$Comp
L power:+5V #PWR07
U 1 1 5BFC3393
P 2050 4150
F 0 "#PWR07" H 2050 4000 50  0001 C CNN
F 1 "+5V" H 2065 4323 50  0000 C CNN
F 2 "" H 2050 4150 50  0001 C CNN
F 3 "" H 2050 4150 50  0001 C CNN
	1    2050 4150
	1    0    0    -1  
$EndComp
Wire Wire Line
	13750 5350 12500 5350
Wire Wire Line
	12500 5350 12500 5250
Wire Wire Line
	13750 5000 13750 5200
Wire Wire Line
	13750 5200 12500 5200
Wire Wire Line
	12500 5200 12500 5150
Wire Wire Line
	13750 4650 13650 4650
Wire Wire Line
	13650 4650 13650 5050
Wire Wire Line
	13650 5050 12500 5050
Wire Wire Line
	13750 3650 12500 3650
Wire Wire Line
	12500 3650 12500 3750
Wire Wire Line
	12500 3850 12650 3850
Wire Wire Line
	12650 3850 12650 3750
Wire Wire Line
	12650 3750 13750 3750
Wire Wire Line
	12500 3950 12750 3950
Wire Wire Line
	12750 3950 12750 3850
Wire Wire Line
	12750 3850 13750 3850
Wire Wire Line
	15150 3550 15150 3800
$Comp
L power:GND #PWR016
U 1 1 5C0463E1
P 15000 3400
F 0 "#PWR016" H 15000 3150 50  0001 C CNN
F 1 "GND" H 15005 3227 50  0000 C CNN
F 2 "" H 15000 3400 50  0001 C CNN
F 3 "" H 15000 3400 50  0001 C CNN
	1    15000 3400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	14550 3450 15000 3450
Wire Wire Line
	15000 3450 15000 3400
$Comp
L power:+5V #PWR015
U 1 1 5C04C4B5
P 14150 4050
F 0 "#PWR015" H 14150 3900 50  0001 C CNN
F 1 "+5V" H 14165 4223 50  0000 C CNN
F 2 "" H 14150 4050 50  0001 C CNN
F 3 "" H 14150 4050 50  0001 C CNN
	1    14150 4050
	-1   0    0    1   
$EndComp
Text GLabel 13600 2500 0    50   Input ~ 0
LCD_D7
Text GLabel 13100 3500 2    50   Input ~ 0
LCD_D7
Wire Wire Line
	12500 3550 12800 3550
Wire Wire Line
	12800 3550 12800 3500
Wire Wire Line
	12800 3500 13100 3500
Wire Wire Line
	13600 2500 13650 2500
Wire Wire Line
	13650 2500 13650 2650
Wire Wire Line
	13650 2650 13750 2650
Text GLabel 14600 3850 2    50   Input ~ 0
LCD_V0
Wire Wire Line
	14550 3850 14600 3850
Text GLabel 12600 2450 2    50   Input ~ 0
LCD_V0
Wire Wire Line
	12500 2450 12600 2450
$Comp
L power:GND #PWR013
U 1 1 5C07D75A
P 11100 6100
F 0 "#PWR013" H 11100 5850 50  0001 C CNN
F 1 "GND" H 11105 5927 50  0000 C CNN
F 2 "" H 11100 6100 50  0001 C CNN
F 3 "" H 11100 6100 50  0001 C CNN
	1    11100 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	10800 5550 10800 6100
Wire Wire Line
	10900 5550 10900 6100
Wire Wire Line
	10800 6100 10900 6100
Connection ~ 10900 6100
Wire Wire Line
	10900 6100 11000 6100
Wire Wire Line
	11000 5550 11000 6100
Connection ~ 11000 6100
Wire Wire Line
	11000 6100 11100 6100
Connection ~ 11100 6100
Wire Wire Line
	11200 5550 11200 6100
Connection ~ 11200 6100
Wire Wire Line
	11200 6100 11100 6100
Wire Wire Line
	11100 5550 11100 6100
$Comp
L power:+5V #PWR012
U 1 1 5C0B1281
P 10900 1350
F 0 "#PWR012" H 10900 1200 50  0001 C CNN
F 1 "+5V" H 10915 1523 50  0000 C CNN
F 2 "" H 10900 1350 50  0001 C CNN
F 3 "" H 10900 1350 50  0001 C CNN
	1    10900 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C12
U 1 1 5C0C2A85
P 2800 1350
F 0 "C12" V 2548 1350 50  0000 C CNN
F 1 "100n" V 2639 1350 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2838 1200 50  0001 C CNN
F 3 "~" H 2800 1350 50  0001 C CNN
	1    2800 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1350 1350 1350 1500
$Comp
L HWP:HYG_RSM_1650 HYGRSM1
U 1 1 5C11E4D9
P 1200 3800
F 0 "HYGRSM1" V 1322 3888 50  0000 L CNN
F 1 "HYG_RSM_1650" V 1413 3888 50  0000 L CNN
F 2 "HWP:HYG_RSM-1650" H 1200 3800 50  0001 C CNN
F 3 "" H 1200 3800 50  0001 C CNN
	1    1200 3800
	-1   0    0    1   
$EndComp
Wire Wire Line
	1250 1150 1550 1150
Wire Wire Line
	1200 3700 1700 3700
Wire Wire Line
	1700 3700 1700 3750
$Comp
L power:+5V #PWR06
U 1 1 5C18550D
P 1200 3800
F 0 "#PWR06" H 1200 3650 50  0001 C CNN
F 1 "+5V" H 1215 3973 50  0000 C CNN
F 2 "" H 1200 3800 50  0001 C CNN
F 3 "" H 1200 3800 50  0001 C CNN
	1    1200 3800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 5C18557E
P 1400 3600
F 0 "#PWR05" H 1400 3350 50  0001 C CNN
F 1 "GND" H 1405 3427 50  0000 C CNN
F 2 "" H 1400 3600 50  0001 C CNN
F 3 "" H 1400 3600 50  0001 C CNN
	1    1400 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	11900 5550 11900 6100
$Comp
L Amplifier_Operational:MC33079 U1
U 3 1 5C22CD4E
P 4500 5700
F 0 "U1" H 4500 5333 50  0000 C CNN
F 1 "MC33079" H 4500 5424 50  0000 C CNN
F 2 "HWP:SOIC-16_3.9x9.9mm_P1.27mm" H 4450 5800 50  0001 C CNN
F 3 "https://www.onsemi.com/pub/Collateral/MC33078-D.PDF" H 4550 5900 50  0001 C CNN
	3    4500 5700
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5C22D2DD
P 3750 5700
F 0 "#PWR010" H 3750 5450 50  0001 C CNN
F 1 "GND" V 3755 5572 50  0000 R CNN
F 2 "" H 3750 5700 50  0001 C CNN
F 3 "" H 3750 5700 50  0001 C CNN
	1    3750 5700
	0    1    1    0   
$EndComp
Wire Wire Line
	3750 5600 4200 5600
Wire Wire Line
	4200 5800 3750 5800
Wire Wire Line
	3750 5600 3750 5700
Connection ~ 3750 5700
Wire Wire Line
	3750 5700 3750 5800
$Comp
L Device:CP C13
U 1 1 5C27B7F9
P 3150 1350
F 0 "C13" H 3268 1396 50  0000 L CNN
F 1 "47u" H 3268 1305 50  0000 L CNN
F 2 "HWP:CP_Radial_D7.5mm_P2.50mm" H 3188 1200 50  0001 C CNN
F 3 "~" H 3150 1350 50  0001 C CNN
	1    3150 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5C299087
P 3100 5000
F 0 "#PWR08" H 3100 4750 50  0001 C CNN
F 1 "GND" H 3105 4827 50  0000 C CNN
F 2 "" H 3100 5000 50  0001 C CNN
F 3 "" H 3100 5000 50  0001 C CNN
	1    3100 5000
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 5050 3100 5000
$Comp
L power:+5V #PWR09
U 1 1 5C2AC1B1
P 3100 5750
F 0 "#PWR09" H 3100 5600 50  0001 C CNN
F 1 "+5V" H 3115 5923 50  0000 C CNN
F 2 "" H 3100 5750 50  0001 C CNN
F 3 "" H 3100 5750 50  0001 C CNN
	1    3100 5750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3100 5650 3100 5750
NoConn ~ 4800 5700
Wire Wire Line
	7200 4250 7550 4250
Connection ~ 7550 4250
Wire Wire Line
	7550 4250 8000 4250
NoConn ~ 10300 2050
NoConn ~ 10300 2150
NoConn ~ 10800 1550
NoConn ~ 11000 1550
NoConn ~ 11200 1550
NoConn ~ 11300 1550
NoConn ~ 11400 1550
NoConn ~ 11500 1550
Wire Wire Line
	10900 1550 10900 1350
NoConn ~ 11900 1550
Wire Wire Line
	1200 3600 1400 3600
$Comp
L Device:C C16
U 1 1 5C3BCC25
P 2400 1350
F 0 "C16" V 2148 1350 50  0000 C CNN
F 1 "100n" V 2239 1350 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2438 1200 50  0001 C CNN
F 3 "~" H 2400 1350 50  0001 C CNN
	1    2400 1350
	-1   0    0    1   
$EndComp
$Comp
L Device:C C14
U 1 1 5C3BCCB1
P 1550 1350
F 0 "C14" V 1298 1350 50  0000 C CNN
F 1 "100n" V 1389 1350 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1588 1200 50  0001 C CNN
F 3 "~" H 1550 1350 50  0001 C CNN
	1    1550 1350
	-1   0    0    1   
$EndComp
Connection ~ 1550 1500
Wire Wire Line
	1550 1500 1350 1500
Wire Wire Line
	1550 1200 1550 1150
Connection ~ 1550 1150
Connection ~ 3150 1500
Wire Wire Line
	3150 1500 3350 1500
Connection ~ 2800 1500
Wire Wire Line
	2800 1500 3150 1500
Connection ~ 2400 1500
Wire Wire Line
	2400 1500 2800 1500
Wire Wire Line
	1550 1500 2000 1500
Wire Wire Line
	2000 1500 2400 1500
Connection ~ 2000 1500
$Comp
L Device:C C15
U 1 1 5C3D33DA
P 2000 1350
F 0 "C15" V 1748 1350 50  0000 C CNN
F 1 "100n" V 1839 1350 50  0000 C CNN
F 2 "HWP:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2038 1200 50  0001 C CNN
F 3 "~" H 2000 1350 50  0001 C CNN
	1    2000 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	1550 1150 2000 1150
Wire Wire Line
	2000 1200 2000 1150
Connection ~ 2000 1150
Wire Wire Line
	2000 1150 2400 1150
Wire Wire Line
	2400 1200 2400 1150
Connection ~ 2400 1150
Wire Wire Line
	2400 1150 2800 1150
Wire Wire Line
	2800 1200 2800 1150
Connection ~ 2800 1150
Wire Wire Line
	2800 1150 3150 1150
Wire Wire Line
	3150 1200 3150 1150
Connection ~ 3150 1150
Wire Wire Line
	3150 1150 3350 1150
$Comp
L Device:R R19
U 1 1 5BE53F7A
P 14800 3550
F 0 "R19" V 14593 3550 50  0000 C CNN
F 1 "100" V 14684 3550 50  0000 C CNN
F 2 "HWP:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 14730 3550 50  0001 C CNN
F 3 "~" H 14800 3550 50  0001 C CNN
	1    14800 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	14550 3550 14650 3550
Wire Wire Line
	14950 3550 15150 3550
Wire Wire Line
	11200 6100 11300 6100
Wire Wire Line
	11300 5550 11300 6100
Connection ~ 11300 6100
Wire Wire Line
	11300 6100 11900 6100
$EndSCHEMATC