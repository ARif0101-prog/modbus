# modbus

WORKINGMODBUSesp8266 is intended for N46IB02 ADC device
WORKINGMODBUSesp8266N4IA04 is intended for N4IA04 ADC device
WORKINGMODBUSESP32 is intended for N4IA04 device
WORKINGMODBUSesp8266withoutTFT is intended for N46IB02 ADC device

if you want to use ESP8266 instead ESP32, then softwareSerial Library is used as phisical serial device replacement. you do not need this on ESP32

Don't forget to modifiy  Modbus_request(0x01, 0x03, 0x00, 0x04) code line, 0x01 with your device id's

Credit to http://shahrulnizam.com as origin file source

Cheers
