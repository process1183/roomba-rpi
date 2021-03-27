EESchema Schematic File Version 4
EELAYER 26 0
EELAYER END
$Descr USLetter 11000 8500
encoding utf-8
Sheet 1 1
Title "Roomba RPi Wiring"
Date "2021-03-26"
Rev ""
Comp ""
Comment1 "https://github.com/process1183/roomba-rpi"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector:Raspberry_Pi_2_3 J14
U 1 1 603C876B
P 8650 2650
F 0 "J14" H 8000 3950 50  0000 C CNN
F 1 "Raspberry_Pi_Zero_W" H 9300 3950 50  0000 C CNN
F 2 "" H 8650 2650 50  0001 C CNN
F 3 "" H 8650 2650 50  0001 C CNN
	1    8650 2650
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW1
U 1 1 60409EC9
P 9350 6050
F 0 "SW1" H 9350 6335 50  0000 C CNN
F 1 "SW_Push" H 9350 6244 50  0000 C CNN
F 2 "" H 9350 6250 50  0001 C CNN
F 3 "" H 9350 6250 50  0001 C CNN
	1    9350 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 6040A1CE
P 9700 5850
F 0 "R1" H 9768 5896 50  0000 L CNN
F 1 "10K" H 9768 5805 50  0000 L CNN
F 2 "" V 9740 5840 50  0001 C CNN
F 3 "" H 9700 5850 50  0001 C CNN
	1    9700 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x05 J16
U 1 1 6040A49D
P 8800 5600
F 0 "J16" H 8800 5950 50  0000 C CNN
F 1 "RPi_Serial_PWR_M" V 8900 5600 50  0000 C CNN
F 2 "" H 8800 5600 50  0001 C CNN
F 3 "" H 8800 5600 50  0001 C CNN
	1    8800 5600
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J17
U 1 1 6040A9C3
P 9450 5200
F 0 "J17" V 9416 5012 50  0000 R CNN
F 1 "Conn_01x03" V 9325 5012 50  0000 R CNN
F 2 "" H 9450 5200 50  0001 C CNN
F 3 "" H 9450 5200 50  0001 C CNN
	1    9450 5200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9000 5400 9350 5400
Wire Wire Line
	9000 5500 9450 5500
Wire Wire Line
	9450 5500 9450 5400
Wire Wire Line
	9000 5600 9550 5600
Wire Wire Line
	9550 5600 9550 5400
Wire Wire Line
	9000 5700 9700 5700
Wire Wire Line
	9000 5800 9050 5800
Wire Wire Line
	9050 5800 9050 6050
Wire Wire Line
	9050 6050 9150 6050
Wire Wire Line
	9550 6050 9700 6050
Wire Wire Line
	9700 6050 9700 6000
Wire Notes Line style solid rgb(255, 0, 0)
	10200 6150 8600 6150
Wire Notes Line style solid rgb(255, 0, 0)
	8600 4900 10200 4900
Text Notes 8700 5000 0    50   ~ 0
RPi Serial and Power Button Board
$Comp
L Connector_Generic:Conn_01x05 J15
U 1 1 6040B3C8
P 7250 1850
F 0 "J15" H 7250 1550 50  0000 C CNN
F 1 "RPi_Serial_PWR_F" V 7400 1800 50  0000 C CNN
F 2 "" H 7250 1850 50  0001 C CNN
F 3 "" H 7250 1850 50  0001 C CNN
	1    7250 1850
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Counter_Clockwise J10
U 1 1 6040B547
P 2850 6750
F 0 "J10" H 2900 7000 50  0000 C CNN
F 1 "RTC_Conn_B" H 2900 6900 50  0000 C CNN
F 2 "" H 2850 6750 50  0001 C CNN
F 3 "" H 2850 6750 50  0001 C CNN
	1    2850 6750
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Counter_Clockwise J8
U 1 1 6040B65C
P 2850 5400
F 0 "J8" H 2900 5617 50  0000 C CNN
F 1 "IMU_Conn_B" H 2900 5526 50  0000 C CNN
F 2 "" H 2850 5400 50  0001 C CNN
F 3 "" H 2850 5400 50  0001 C CNN
	1    2850 5400
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x02_Counter_Clockwise J11
U 1 1 6040B7A6
P 4100 6750
F 0 "J11" H 4150 6967 50  0000 C CNN
F 1 "RTC_Conn_A" H 4150 6876 50  0000 C CNN
F 2 "" H 4100 6750 50  0001 C CNN
F 3 "" H 4100 6750 50  0001 C CNN
	1    4100 6750
	1    0    0    -1  
$EndComp
$Comp
L Adafruit_DS3231_Breakout:Adafruit_DS3231_Breakout A4
U 1 1 6045A57F
P 1900 6800
F 0 "A4" H 2200 7250 50  0000 C CNN
F 1 "Adafruit_DS3231_Breakout" H 2500 6350 50  0000 C CNN
F 2 "" H 1550 7000 50  0001 C CNN
F 3 "https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout?view=all" H 800 5900 50  0001 L CNN
F 4 "https://github.com/process1183/kicad-library" H 800 5750 50  0001 L CNN "GitHub"
F 5 "https://www.adafruit.com/product/3013" H 800 6050 50  0001 L CNN "Product URL"
	1    1900 6800
	-1   0    0    -1  
$EndComp
$Comp
L Adafruit_BNO055_Breakout:Adafruit_BNO055_Breakout A3
U 1 1 6046EF8D
P 1800 5350
F 0 "A3" H 1400 5800 50  0000 C CNN
F 1 "Adafruit_BNO055_Breakout" H 1200 4900 50  0000 C CNN
F 2 "" H 1800 5350 50  0001 C CNN
F 3 "https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor?view=all" H 900 4350 50  0001 L CNN
F 4 "https://www.adafruit.com/product/2472" H 900 4500 50  0001 L CNN "Product URL"
F 5 "https://github.com/process1183/kicad-library" H 900 4200 50  0001 L CNN "GitHub"
	1    1800 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	8250 3950 8250 4000
Wire Wire Line
	7850 2850 7700 2850
Wire Wire Line
	8850 1350 8850 1250
Text GLabel 8850 1250 1    50   Input ~ 0
RPi_3v3_17
Wire Wire Line
	7850 1750 7450 1750
Wire Wire Line
	7850 1850 7450 1850
Wire Wire Line
	7450 1950 7700 1950
Wire Wire Line
	7700 1950 7700 2850
Wire Wire Line
	6950 1500 6950 4000
Wire Wire Line
	6950 4000 8250 4000
Wire Wire Line
	7450 2050 7600 2050
Wire Wire Line
	7600 2050 7600 2150
Text GLabel 7600 2150 3    50   Input ~ 0
RPi_3v3_17
Wire Wire Line
	7450 1650 7500 1650
Wire Wire Line
	7500 1650 7500 1500
Wire Wire Line
	6950 1500 7500 1500
Wire Wire Line
	8750 1250 8750 1350
Text GLabel 8750 1250 1    50   Input ~ 0
RPi_3v3_1
Wire Wire Line
	2550 6750 2250 6750
Wire Wire Line
	2550 6850 2250 6850
Wire Wire Line
	3050 6750 3100 6750
Wire Wire Line
	3100 6750 3100 6300
Wire Wire Line
	3100 6300 2000 6300
Wire Wire Line
	2000 6300 2000 6350
Wire Wire Line
	3050 6850 3100 6850
Wire Wire Line
	3100 6850 3100 7300
Wire Wire Line
	3100 7300 1900 7300
Wire Wire Line
	1900 7300 1900 7250
Wire Wire Line
	2250 5400 2550 5400
Wire Wire Line
	2250 5500 2550 5500
Wire Wire Line
	3050 5400 3100 5400
Wire Wire Line
	3100 5400 3100 4850
Wire Wire Line
	3100 4850 1700 4850
Wire Wire Line
	1700 4850 1700 4900
Wire Wire Line
	3050 5500 3100 5500
Wire Wire Line
	3100 5500 3100 5850
Wire Wire Line
	3100 5850 1800 5850
Wire Wire Line
	1800 5850 1800 5800
Wire Wire Line
	4400 6750 4450 6750
Wire Wire Line
	4400 6850 4450 6850
Text GLabel 4450 6750 2    50   Input ~ 0
RPi_SCL
Text GLabel 4450 6850 2    50   Input ~ 0
RPi_SDA
Wire Wire Line
	3900 6850 3850 6850
Text GLabel 3850 6850 0    50   Input ~ 0
RPi_GND_9
Wire Wire Line
	3900 6750 3850 6750
Text GLabel 3850 6750 0    50   Input ~ 0
RPi_3v3_1
Wire Wire Line
	8350 3950 8350 4050
Text GLabel 8350 4050 3    50   Input ~ 0
RPi_GND_9
$Comp
L Connector_Generic:Conn_02x02_Counter_Clockwise J9
U 1 1 605E4632
P 4100 5400
F 0 "J9" H 4150 5617 50  0000 C CNN
F 1 "IMU_Conn_A" H 4150 5526 50  0000 C CNN
F 2 "" H 4100 5400 50  0001 C CNN
F 3 "" H 4100 5400 50  0001 C CNN
	1    4100 5400
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 5400 4450 5400
Wire Wire Line
	4400 5500 4450 5500
Text GLabel 4450 5400 2    50   Input ~ 0
RPi_SCL
Text GLabel 4450 5500 2    50   Input ~ 0
RPi_SDA
Wire Wire Line
	3900 5500 3850 5500
Text GLabel 3850 5500 0    50   Input ~ 0
RPi_GND_9
Wire Wire Line
	3900 5400 3850 5400
Text GLabel 3850 5400 0    50   Input ~ 0
RPi_3v3_1
Text GLabel 9500 2150 2    50   Input ~ 0
RPi_SCL
Text GLabel 9500 2050 2    50   Input ~ 0
RPi_SDA
Wire Wire Line
	9450 2050 9500 2050
Wire Wire Line
	9450 2150 9500 2150
Wire Notes Line rgb(255, 0, 0)
	8600 5050 10200 5050
Wire Notes Line style solid rgb(255, 0, 0)
	8600 4900 8600 6150
Wire Notes Line style solid rgb(255, 0, 0)
	10200 4900 10200 6150
$Comp
L Connector:USB_B_Micro J12
U 1 1 60614807
P 6000 5700
F 0 "J12" H 6000 6200 50  0000 C CNN
F 1 "USB_B_Micro-CP2104" H 6000 6100 50  0000 C CNN
F 2 "" H 6150 5650 50  0001 C CNN
F 3 "~" H 6150 5650 50  0001 C CNN
	1    6000 5700
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J13
U 1 1 60614906
P 7550 5700
F 0 "J13" H 7600 6200 50  0000 R CNN
F 1 "USB_B_Micro-RPi_OTG" H 7950 6100 50  0000 R CNN
F 2 "" H 7700 5650 50  0001 C CNN
F 3 "~" H 7700 5650 50  0001 C CNN
	1    7550 5700
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7250 5900 7250 6150
Wire Wire Line
	7250 6150 7550 6150
Wire Wire Line
	7550 6150 7550 6100
Wire Wire Line
	6000 6150 6000 6100
Connection ~ 7250 6150
Wire Wire Line
	6000 6150 7250 6150
Wire Wire Line
	6300 5500 7250 5500
Wire Wire Line
	6300 5700 7250 5700
Wire Wire Line
	6300 5800 7250 5800
Wire Notes Line style solid rgb(255, 0, 0)
	5550 4900 8050 4900
Wire Notes Line style solid rgb(255, 0, 0)
	8050 4900 8050 6300
Wire Notes Line style solid rgb(255, 0, 0)
	8050 6300 5550 6300
Wire Notes Line style solid rgb(255, 0, 0)
	5550 6300 5550 4900
Wire Notes Line rgb(255, 0, 0)
	5550 5050 8050 5050
Text Notes 5900 5000 0    50   ~ 0
Micro USB B Cable - RPi OTG to CP2104 Board
$Comp
L Connector_Generic:Conn_01x04 J2
U 1 1 60631F60
P 1900 2650
F 0 "J2" V 1866 2362 50  0000 R CNN
F 1 "Roomba_OI_M" V 1775 2362 50  0000 R CNN
F 2 "" H 1900 2650 50  0001 C CNN
F 3 "~" H 1900 2650 50  0001 C CNN
	1    1900 2650
	0    -1   -1   0   
$EndComp
$Comp
L Connector_Generic:Conn_01x04 J1
U 1 1 6063662B
P 1900 2400
F 0 "J1" V 1773 2112 50  0000 R CNN
F 1 "Roomba_OI_F" V 1864 2112 50  0000 R CNN
F 2 "" H 1900 2400 50  0001 C CNN
F 3 "~" H 1900 2400 50  0001 C CNN
	1    1900 2400
	0    -1   1    0   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J3
U 1 1 6063C150
P 2100 4050
F 0 "J3" H 2100 3850 50  0000 C CNN
F 1 "UBEC_IN_F" H 1950 4150 50  0000 C CNN
F 2 "" H 2100 4050 50  0001 C CNN
F 3 "~" H 2100 4050 50  0001 C CNN
	1    2100 4050
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J4
U 1 1 6063C2ED
P 2350 4050
F 0 "J4" H 2350 3850 50  0000 C CNN
F 1 "UBEC_IN_M" H 2200 4150 50  0000 C CNN
F 2 "" H 2350 4050 50  0001 C CNN
F 3 "~" H 2350 4050 50  0001 C CNN
	1    2350 4050
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J5
U 1 1 6063C411
P 3300 3000
F 0 "J5" H 3219 2475 50  0000 C CNN
F 1 "CP2104_F" H 3219 2566 50  0000 C CNN
F 2 "" H 3300 3000 50  0001 C CNN
F 3 "~" H 3300 3000 50  0001 C CNN
	1    3300 3000
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x06 J6
U 1 1 6063C6D4
P 3600 3000
F 0 "J6" H 3520 2475 50  0000 C CNN
F 1 "CP2104_M" H 3520 2566 50  0000 C CNN
F 2 "" H 3600 3000 50  0001 C CNN
F 3 "~" H 3600 3000 50  0001 C CNN
	1    3600 3000
	-1   0    0    1   
$EndComp
Wire Wire Line
	4500 2600 4000 2600
Wire Wire Line
	4000 2600 4000 2800
Wire Wire Line
	4000 2800 3800 2800
Wire Wire Line
	4500 2900 3800 2900
Wire Wire Line
	3800 3200 4900 3200
$Comp
L Regulator_UBEC:MP2307_UBEC A1
U 1 1 6066F888
P 3150 4000
F 0 "A1" H 3378 4046 50  0000 L CNN
F 1 "MP2307_UBEC" H 3378 3955 50  0000 L CNN
F 2 "" H 3150 4000 50  0001 C CNN
F 3 "https://cdn-shop.adafruit.com/datasheets/MP2307_r1.9.pdf" H 2250 3250 50  0001 L CNN
F 4 "https://www.adafruit.com/product/1385" H 2250 3150 50  0001 L CNN "Product URL"
F 5 "https://github.com/process1183/kicad-library" H 2250 3050 50  0001 L CNN "GitHub"
	1    3150 4000
	1    0    0    -1  
$EndComp
Text GLabel 1800 1650 1    50   Input ~ 0
Roomba_7_GND
Text GLabel 1900 1650 1    50   Input ~ 0
Roomba_1_Vpwr
Text GLabel 2100 1650 1    50   Input ~ 0
Roomba_4_TXD
Text GLabel 2200 1650 1    50   Input ~ 0
Roomba_3_RXD
Text GLabel 1700 1650 1    50   Input ~ 0
Roomba_6_GND
Text GLabel 2000 1650 1    50   Input ~ 0
Roomba_2_Vpwr
Wire Wire Line
	1800 1650 1800 1750
Wire Wire Line
	1900 1650 1900 1750
Wire Wire Line
	2000 1650 2000 1750
Wire Wire Line
	2000 1750 1900 1750
Connection ~ 1900 1750
Wire Wire Line
	1900 1750 1900 2200
Wire Wire Line
	1700 1650 1700 1750
Wire Wire Line
	1700 1750 1800 1750
Connection ~ 1800 1750
Wire Wire Line
	1800 1750 1800 2200
Wire Wire Line
	2100 1650 2100 1850
Wire Wire Line
	2100 1850 2000 1850
Wire Wire Line
	2000 1850 2000 2200
Wire Wire Line
	2200 1650 2200 1950
Wire Wire Line
	2200 1950 2100 1950
Wire Wire Line
	2100 1950 2100 2200
Text GLabel 2300 1650 1    50   Input ~ 0
Roomba_5_BRC
Wire Notes Line
	1600 950  2400 950 
Wire Notes Line
	2400 1800 1600 1800
Wire Notes Line
	1600 850  2400 850 
Wire Notes Line
	2400 850  2400 1800
Wire Notes Line
	1600 850  1600 1800
Text Notes 1600 950  0    39   ~ 0
Roomba Serial Connector
Wire Wire Line
	2300 1650 2300 1700
NoConn ~ 2300 1700
Wire Wire Line
	1900 2850 1900 3950
Wire Wire Line
	1800 4050 1900 4050
Wire Wire Line
	2550 4050 2750 4050
Wire Wire Line
	2750 4050 2750 4300
Wire Wire Line
	2750 4300 3050 4300
Wire Wire Line
	3050 3700 2750 3700
Wire Wire Line
	2750 3700 2750 3950
Wire Wire Line
	2750 3950 2550 3950
Wire Wire Line
	1800 2850 1800 3200
Wire Wire Line
	3100 3200 1800 3200
Connection ~ 1800 3200
Wire Wire Line
	1800 3200 1800 4050
Wire Wire Line
	2000 2850 2000 3000
Wire Wire Line
	2000 3000 2900 3000
Wire Wire Line
	2900 3000 2900 2800
Wire Wire Line
	2900 2800 3100 2800
Wire Wire Line
	3100 2900 2100 2900
Wire Wire Line
	2100 2900 2100 2850
$Comp
L Connector:USB_B_Micro J7
U 1 1 6061478B
P 4550 3900
F 0 "J7" H 4320 3891 50  0000 R CNN
F 1 "USB_B_Micro-RPi_PWR" H 4320 3800 50  0000 R CNN
F 2 "" H 4700 3850 50  0001 C CNN
F 3 "~" H 4700 3850 50  0001 C CNN
	1    4550 3900
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3250 3700 4250 3700
Wire Wire Line
	3250 4300 4550 4300
Wire Wire Line
	4500 2000 3900 2000
Wire Wire Line
	3900 2000 3900 2700
Wire Wire Line
	3900 2700 3800 2700
$Comp
L Adafruit_CP2104_Friend:Adafruit_CP2104_Friend A2
U 1 1 6045A4E9
P 5000 2300
F 0 "A2" H 5500 3050 50  0000 C CNN
F 1 "Adafruit_CP2104_Friend" H 5900 2950 50  0000 C CNN
F 2 "" H 5000 2200 50  0001 C CNN
F 3 "https://www.silabs.com/documents/public/data-sheets/cp2104.pdf" H 4000 900 50  0001 L CNN
F 4 "https://github.com/process1183/kicad-library" H 4000 750 50  0001 L CNN "GitHub"
F 5 "https://www.adafruit.com/product/3309" H 4000 1050 50  0001 L CNN "Product URL"
	1    5000 2300
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3800 3100 3900 3100
Wire Wire Line
	3800 3000 3900 3000
Wire Wire Line
	4500 2300 4450 2300
Text GLabel 3900 3100 2    50   Input ~ 0
CP2104_P10
Text GLabel 3900 3000 2    50   Input ~ 0
CP2104_P11
Text GLabel 4450 2300 0    50   Input ~ 0
CP2104_P10
Text GLabel 5000 1300 0    50   Input ~ 0
CP2104_P11
Wire Wire Line
	5000 1300 5050 1300
Wire Wire Line
	5050 1300 5050 1400
$EndSCHEMATC
