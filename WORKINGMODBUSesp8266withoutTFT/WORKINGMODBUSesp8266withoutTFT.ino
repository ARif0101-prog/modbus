/*
  Project: Modbus Analog Acquisition N4AIA04
  Programmer: Shahrulnizam Mat Rejab
  Board: TTGO ESP32 Display
  Last Modified: 29 May 2021
  Website: http://shahrulnizam.com
*/

//#include <TFT_eSPI.h>
#include <SoftwareSerial.h>
#define TROUBLESHOOT
#define RX            D1   //pin TX0 max485
#define RE            D0    //pin E max485
#define TX            D2   //pin RX0 max485
SoftwareSerial  modbus;
//SoftwareSerial     modbus=new SoftwareSerial(RX,TX, false, 128, true);

// #define TROUBLESHOOT
// #define RX            17   //pin 1 MAX485
// #define RE            2    //pin 2&3 MAX485
// #define TX            16   //pin 4 MAX485
// #define modbus        Serial1

//TFT_eSPI tft = TFT_eSPI();

uint8_t counter, crc_l, crc_h, address[20];
uint16_t crc;
uint32_t i, j, data, timer1;
float voltage1, voltage2, current1, current2;
String message;

void setup()
{
  Serial.begin(115200);
  //modbus.begin(9600, SERIAL_8N1);
  modbus.begin(9600, SWSERIAL_8N1, RX, TX);
  pinMode(RE, OUTPUT);
  digitalWrite(RE, LOW);
  // tft.init();
  // tft.invertDisplay(true);
  // tft.setRotation(3);
  // tft.setTextSize(2);
  // tft.fillScreen(TFT_BLACK);
}

void loop()
{
  if (millis() - timer1 >= 1000)
  {
    timer1 = millis();
    //Modbus_request(0x01, 0x03, 0x00, 0x04);
    Modbus_request(0x02, 0x03, 0x00, 0x04);
  }

  if (modbus.available())
  {
    i = 0;
    while (modbus.available()) address[i++] = modbus.read();

    while (address[0] == 0)
    {
      for (j = 0; j < i; j++) address[j] = address[j + 1];
      i--;
    }

#if defined (TROUBLESHOOT)
    if (i > 2)
    {
      for (j = 0; j < i; j++)
      {
        if (address[j] < 0x10) Serial.print("0");
        Serial.print(address[j], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
#endif

    if (i > 2)
    {
      crc = CRC16(i - 2);
      crc_l = crc & 0xFF;
      crc_h = (crc >> 8) & 0xFF;

      if ((address[i - 2] == crc_l) & (address[i - 1] == crc_h))
      {
        //Serial.println("RAW data"+ String(address[9]));
        //Serial.println("RAW data2"+ String(address[10]));
       // voltage1 = 0.01 * (256 * address[3] + address[4]);
        voltage1 = 0.1 * (256 * address[3] + address[4]);
        voltage2 = 0.01 * (256 * address[5] + address[6]);
        current1 = 0.1 * (256 * address[7] + address[8]);
        current2 = 0.1 * (256 * address[9] + address[10]);
        Serial.println("Voltage 1=" + String(voltage1) + "V Voltage 2=" + String(voltage2) + "V");
        Serial.println("Current 1=" + String(current1) + "mA Current 2=" + String(current2) + "mA");
      }
      
    }
  }

//  tft.setCursor(0, 0);
//   tft.println(" Analog Acquisition ");
//  tft.println();
//   message = "Voltage 1 = " + String(voltage1) + " V";
 //delay(14);
  // while (message.length() < 20) message += " ";
  // tft.println(message);
  // message = "Voltage 2 = " + String(voltage2) + " V";
  // while (message.length() < 20) message += " ";
  // tft.println(message);
  // message = "Current 1 = " + String(current1) + " mA";
  // while (message.length() < 20) message += " ";
  // tft.println(message);
  // message = "Current 2 = " + String(current2) + " mA";
  // while (message.length() < 20) message += " ";
  // tft.println(message);

}

void Modbus_request(unsigned char id, unsigned char code, unsigned int addr, unsigned int quantity)
{
  address[0] = id;
  address[1] = code;
  address[2] = addr >> 8;
  address[3] = addr;
  address[4] = quantity >> 8;
  address[5] = quantity;
  crc = CRC16(6);
  address[6] = crc & 0xFF;
  address[7] = (crc >> 8) & 0xFF;

#if defined (TROUBLESHOOT)
  for (i = 0; i < 8; i++)
  {
    if (address[i] < 0x10) Serial.print("0");
    Serial.print(address[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
#endif

  digitalWrite(RE, HIGH);
  delay(1);
  for (i = 0; i < 8; i++) modbus.write(address[i]);
  
  digitalWrite(RE, LOW);
  delay(18); //adjust this delay untill the output reach 13 bytes
}

int CRC16(int DataLength)
{
  unsigned int i, j, CheckSum;
  CheckSum = 0xFFFF;
  for (j = 0; j < DataLength; j++) {
    CheckSum = CheckSum ^ address[j];
    for (i = 0; i < 8; i++) {
      if ((CheckSum) & 0x0001 == 1) CheckSum = (CheckSum >> 1) ^ 0xA001;
      else CheckSum = CheckSum >> 1;
    }
  }
  return CheckSum;
}