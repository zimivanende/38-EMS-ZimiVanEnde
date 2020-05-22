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
L ttgo_esp32:TTGO_ESP32_TDISPLAY_V1.1 TTGO?
U 1 1 5EC7E55C
P 2700 3050
F 0 "TTGO?" H 3050 4487 60  0000 C CNN
F 1 "TTGO_ESP32_TDISPLAY_V1.1" H 3050 4381 60  0000 C CNN
F 2 "" H 2700 3050 60  0001 C CNN
F 3 "" H 2700 3050 60  0001 C CNN
	1    2700 3050
	1    0    0    -1  
$EndComp
$Comp
L Sensor:DHT11 U?
U 1 1 5EC83560
P 5050 3300
F 0 "U?" H 4806 3346 50  0000 R CNN
F 1 "DHT11" H 4806 3255 50  0000 R CNN
F 2 "Sensor:Aosong_DHT11_5.5x12.0_P2.54mm" H 5050 2900 50  0001 C CNN
F 3 "http://akizukidenshi.com/download/ds/aosong/DHT11.pdf" H 5200 3550 50  0001 C CNN
	1    5050 3300
	1    0    0    -1  
$EndComp
$Comp
L T6713-6H:T6713-6H IC?
U 1 1 5EC83FE7
P 5050 4800
F 0 "IC?" H 6292 5065 50  0000 C CNN
F 1 "T6713-6H" H 6292 4974 50  0000 C CNN
F 2 "T67136H" H 7600 4900 50  0001 L CNN
F 3 "https://www.amphenol-sensors.com/en/component/edocman/297-t6713-datasheet/download?Itemid=8486%20%27" H 7600 4800 50  0001 L CNN
F 4 "AMPHENOL ADVANCED SENSORS - T6713-6H. - GAS DETECTION SENSOR, CO2, 5000PPM" H 7600 4700 50  0001 L CNN "Description"
F 5 "3" H 7600 4600 50  0001 L CNN "Height"
F 6 "Amphenol" H 7600 4500 50  0001 L CNN "Manufacturer_Name"
F 7 "T6713-6H" H 7600 4400 50  0001 L CNN "Manufacturer_Part_Number"
F 8 "527-T6713-6H" H 7600 4300 50  0001 L CNN "Mouser Part Number"
F 9 "https://www.mouser.co.uk/ProductDetail/Amphenol-Advanced-Sensors/T6713-6H?qs=y%2FafN2TeCurWSLpbUp60Jg%3D%3D" H 7600 4200 50  0001 L CNN "Mouser Price/Stock"
F 10 "" H 7600 4100 50  0001 L CNN "RS Part Number"
F 11 "" H 7600 4000 50  0001 L CNN "RS Price/Stock"
	1    5050 4800
	1    0    0    -1  
$EndComp
Text GLabel 4800 1600 0    50   UnSpc ~ 0
GND
$Comp
L Device:LED_RGBC D?
U 1 1 5EC81359
P 5000 1600
F 0 "D?" H 5000 2097 50  0000 C CNN
F 1 "LED_RGBC" H 5000 2006 50  0000 C CNN
F 2 "" H 5000 1550 50  0001 C CNN
F 3 "~" H 5000 1550 50  0001 C CNN
	1    5000 1600
	1    0    0    -1  
$EndComp
Text GLabel 5200 1400 2    50   Input ~ 0
R
Text GLabel 5200 1600 2    50   Input ~ 0
G
Text GLabel 5200 1800 2    50   Input ~ 0
B
Text GLabel 5050 3600 3    50   UnSpc ~ 0
GND
Text GLabel 5050 3000 1    50   Input ~ 0
VCC
Text GLabel 5350 3300 2    50   Output ~ 0
DHT_OUT
Text GLabel 3600 2900 2    50   UnSpc ~ 0
GND
Text GLabel 3600 3000 2    50   Output ~ 0
VCC
Text GLabel 3600 2800 2    50   Input ~ 0
DHT_OUT
Text GLabel 3600 2700 2    50   Output ~ 0
R
Text GLabel 3600 2600 2    50   Output ~ 0
G
Text GLabel 3600 2500 2    50   Output ~ 0
B
Text GLabel 2500 2100 0    50   BiDi ~ 0
SDA
Text GLabel 2500 2200 0    50   BiDi ~ 0
SCL
Text GLabel 5050 4800 0    50   BiDi ~ 0
SDA
Text GLabel 5050 4900 0    50   BiDi ~ 0
SCL
Text GLabel 5050 5000 0    50   Input ~ 0
VCC
Text GLabel 5050 5100 0    50   UnSpc ~ 0
GND
Text GLabel 5050 5300 0    50   UnSpc ~ 0
GND
Text Notes 7400 7500 0    50   ~ 0
EMS
Text Notes 8150 7650 0    50   ~ 0
22/05/2020
Text Notes 7050 6750 0    50   ~ 0
Zimi Van Ende\n1642931
$EndSCHEMATC
