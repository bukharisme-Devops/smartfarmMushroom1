// Fill-in information from your Blynk Template here
//=======Change log for version 0.2.0=======//
//update add check blynk connected
//update add read modbus sensor data 3in1
//update add read modbus sensor data Drop Level
//update add read modbus sensor data Water Flow
//update add read modbus sensor data CO2
//update add read modbus sensor data PH
//update add read modbus sensor data PZEM-AC
//update add reset configure button to gpio36
//update add button on/off
//update add read Date&Time
//=======Change log for version 0.2.0=======//
//=============New Blynk IoT================//
#define BLYNK_TEMPLATE_ID "TMPLcsm9HlnH"
#define BLYNK_DEVICE_NAME "YRUmushroom"
#define BLYNK_AUTH_TOKEN "Pehonbn2RNTT5j6lUVFFXCLWFgwPdYjU"
#define BLYNK_FIRMWARE_VERSION        "0.4.0"
#define BLYNK_PRINT Serial
//#define BLYNK_DEBUG
#define APP_DEBUG
// Uncomment your board, or configure a custom board in Settings.h
#define USE_WROVER_BOARD
#include "BlynkEdgent.h"
#include <SimpleTimer.h>
SimpleTimer timer;
//=============New Blynk IoT================//

//============Senddata2GGSheet=================//
#include <WiFi.h>
#include <HTTPClient.h>
const char* host = "script.google.com";
const char* httpsPort = "443";
String GAS_ID = "AKfycbwrm4Hz-p-x9KhUY22N4eDk7apdc46ZA7rEhN4WkRl9dWQGjQ8";
//============Senddata2GGSheet=================//

//==================NTP=====================//
#include <NTPClient.h>
#include <WiFiUdp.h>
// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
// Variables to save date and time
String formattedDate;
String dayStamp;
String timeStamp;
//==================NTP=====================//
//===================Line Notify====================//
#include <TridentTD_LineNotify.h>
#define LINE_TOKEN  "CEpQBJB0h8tfyk6Fun4xJjE4SYyRo7IF7YwJd5h3KvQ"
uint8_t lastLineMsgtemp1 = 0;  //Temperature < setLimit
uint8_t lastLineMsghumi1 = 0;  //Humidity < setLimit
uint8_t lastLineMsgtemp1humi1 = 0;  //Temp > setLimit || Humidity > setLimit
//===================Line Notify====================//
//===================ปุ่ม เปิด-ปิด sw1_Solenoid=================//
//V10 Auto&Manual Solenoid
//V11 Slider waterlevel
//Manual & Auto Switch solenoid
#define Widget_Btn_sw1_solenoid        V0           //ปุ่มเปิดปิด solenoid
#define Widget_LED_sw1_solenoid        V20          //สถานะปุ่มเปิดปิด solenoid
WidgetLED LedBlynkSW1(Widget_LED_sw1_solenoid);
//Slider for set water level limit
bool switchStatus1 = 0; // 0 = manual,1=auto
int WaterLevelLimit1 = 0;
bool manualSwitch1 = 0;
//===================ปุ่ม เปิด-ปิด sw1_solenoid=================//
//===================ปุ่ม เปิด-ปิด sw2_Heater=================//
//V12 Auto&Manual Heater
//V13 Slider temperature
//Manual & Auto Switch Heater
#define Widget_Btn_sw2_heater        V1           //ปุ่มเปิดปิด Heater
#define Widget_LED_sw2_heater        V21          //สถานะปุ่มเปิดปิด Heater
WidgetLED LedBlynkSW2(Widget_LED_sw2_heater);
//Slider for set temp limit
bool switchStatus2 = 0; // 0 = manual,1=auto
int TempLevelLimit2 = 0;
bool manualSwitch2 = 0;
//===================ปุ่ม เปิด-ปิด sw2_Heater=================//

//===================ปุ่ม เปิด-ปิด sw3_ultra=================//
//V14 Auto&Manual Ultrasonic
//V15 Slider Humidity
#define Widget_Btn_sw3_ultra        V2           //ปุ่มเปิดปิดปั้มพ่นหมอก
#define Widget_LED_sw3_ultra        V22          //สถานะปุ่มเปิดปิดปั้มพ่นหมอก
WidgetLED LedBlynkSW3(Widget_LED_sw3_ultra);
//Slider for set Humi limit
bool switchStatus3 = 0; // 0 = manual,1=auto
int HumiLevellimit3 = 0;
bool manualSwitch3 = 0;
//===================ปุ่ม เปิด-ปิด sw3_ultra=================//

//===================ปุ่ม เปิด-ปิด sw4_fan==================//
//V15 Auto&Manual พัดลม
#define Widget_Btn_sw4_fan          V3            //ปุ่มเปิดปิดพัดลม
#define Widget_LED_sw4_fan          V23           //สถานะปุ่มเปิดปิดพัดลม
WidgetLED LedBlynkSW4(Widget_LED_sw4_fan);
bool switchStatus4 = 0; // 0 = manual,1=auto
//int LuxLevellimit4 = 0;
bool manualSwitch4 = 0;
//===================ปุ่ม เปิด-ปิด sw4_fan==================

//============Blynk Virtual Pin============//
//V0    sw1_solenoid relay1
//V10   Auto/Manual Solenoid
//V11   Slider waterlevel
//V20   สถานะปุ่มเปิดปิดsolenoid
//V1    sw2_heater relay2
//V21   สถานะปุ่มเปิดปิด Heater
//V2    sw3_ultra relay3
//V22   สถานะปุ่มเปิดปิดปั้มพ่นหมอก
//V3    sw4_fan relay4
//V23   สถานะปุ่มเปิดปิดพัดลม

//V7    CurrentDate
//V8    CurrentTime

//V12   Auto&Manual cfan
//V13   Slider temperature

//V14   Auto&Manual Ultrasonic
//V15   Slider Humidity

//V16   Auto&Manual Light
//V17   Slider Light

//V31     ett_temperature (C)
//V32     ett_humidity (%)
//V33     ett_light (Lux, %)
//V34     drop_level (m)
//V35     Flow_rate m3/h
//V36     Velocity m/s
//V37     Flow_day
//V38     Flow_month
//V39     Flow_year
//V40     co2 (ppm)
//V41     ph_temp
//V42     ph
//V43     pzem_votage (V)
//V44     pzem_current (A)
//V45     pzem_power  (W)
//V46     pzem_energy (Wh)
//V47     reset_pzem_energy
//============Blynk Virtual Pin============//

//==================PZEM-AC======================//
static uint8_t pzemSlaveAddr = 0x06;
float pz_voltage = 0;
float pz_current = 0;
float pz_power = 0;
float pz_energy = 0;
int Reset_Energy = 0;
//===============================================//

//===============================================//
#include <Wire.h>
#include "ETT_PCF8574.h"
ETT_PCF8574 master_relay(PCF8574_ID_DEV0);
//===============================================//

//==============Hardware Serial==================//
#include <HardwareSerial.h>
//==============Hardware Serial==================//
#define SerialDebug           Serial             // USB Serial(Serial0)
//===============================================//
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define SerialRS485           Serial2           // Serial2(IO27=TXD,IO26=RXD)
//==============================================//
#define I2C_SCL_PIN           22                // ESP32-WROVER : IO22(SCL1)
#define I2C_SDA_PIN           21                // ESP32-WROVER : IO21(SDA1)
//==============================================//
#define LED_PIN               2                 // ESP-WROVER  : IO2
#define LedON                 1
#define LedOFF                0
//==============================================//
#define USER_SW_PIN           36                // ESP32-WROVER :IO36
#define SW_PRESS              LOW
#define SW_RELEASE            HIGH
//==============================================//
#define RTC_INT_PIN           39                // ESP32-WROVER :IO39
#define RTC_INT_ACTIVE        LOW
#define RTC_INT_DEACTIVE      HIGH
//=============================================//
// End of Default Hardware : ET-ESP32(WROVER) RS485 V2
//=============================================//

//============================================//
#include "ModbusMaster.h"                     // https://github.com/4-20ma/ModbusMaster
//============================================//
ModbusMaster node1;                           // Slave ID1(ETT3IN1)
ModbusMaster node2;                           // Slave ID2(Drop Sensor)
ModbusMaster node3;                           // Slave ID3(Water Flow)
ModbusMaster node4;                           // Slave ID3(CO2)
ModbusMaster node5;                           // Slave ID3(PH)
ModbusMaster node6;                           // Slave ID3(PZEM-AC)
//============================================//

//=============Setup Function=================//
void setup()
{
  //=========================================//
  // Start of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //=========================================//
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LedOFF);
  //========================================//
  pinMode(USER_SW_PIN, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  //========================================//
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  //========================================//
  SerialDebug.begin(9600);
  while (!SerialDebug);
  //========================================//
  // End of Initial Default Hardware : ET-ESP32(WROVER) RS485 V2
  //========================================//

  //========================================//
  SerialDebug.println();
  SerialDebug.println("ET-ESP32(WROVER)RS485 V2.....Ready");
  //========================================//

  //========================================//
  SerialRS485.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  while (!SerialRS485);
  //========================================//
  node1.begin(1, SerialRS485);              // Modbus slave ID 1(ETT3IN1)
  node2.begin(2, SerialRS485);              // Modbus slave ID 2(Drop Sensor)
  node3.begin(3, SerialRS485);              // Modbus slave ID 3(Water Flow)
  node4.begin(4, SerialRS485);              // Modbus slave ID 1(CO2)
  node5.begin(5, SerialRS485);              // Modbus slave ID 2(PH)
  node6.begin(pzemSlaveAddr, SerialRS485);  // Modbus slave ID 3(PZEM-AC)
  //========================================//

  //========================================//
  master_relay.begin(0xFF);
  //========================================//
  master_relay.writePin(RELAY_OUT3_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT2_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT1_PIN, RELAY_OFF);
  master_relay.writePin(RELAY_OUT0_PIN, RELAY_OFF);
  //========================================//
  //LineNotify
  LINE.setToken(LINE_TOKEN);  // กำหนด Line Token

  BlynkEdgent.begin();
  timer.setInterval(5000L, datetime);
  timer.setInterval(5000L, ett3in1);
  timer.setInterval(6000L, drop_level);
  timer.setInterval(7000L, water_flow);
  timer.setInterval(8000L, co2);
  timer.setInterval(9000L, ph);
  timer.setInterval(4000L, pzem);
  timer.setInterval(5000L, sendData2GGSheet);
}
//=============Setup Function=================//
//==Update switchStatus2 on Temperature for Heater==//
BLYNK_WRITE(V12)
{
  switchStatus2 = param.asInt(); // Get value as integer
}
// Update Temperature setting
BLYNK_WRITE(V13)
{
  TempLevelLimit2 = param.asInt(); // Get value as integer
}
// Update manualSwitch2
BLYNK_WRITE(V1)
{
  manualSwitch2 = param.asInt();
}
//==Update switchStatus2 on Temperature for Heater==//

//====Update switchStatus3 on Humidity for Froggy===//
BLYNK_WRITE(V14)
{
  switchStatus3 = param.asInt(); // Get value as integer
}
// Update Humidity setting
BLYNK_WRITE(V15)
{
  HumiLevellimit3 = param.asInt(); // Get value as integer
}
// Update manualSwitch3
BLYNK_WRITE(V2)
{
  manualSwitch3 = param.asInt();
}
//====Update switchStatus3 on Humidity for Froggy===//

//======Update switchStatus4 for cooling fan========//
BLYNK_WRITE(V16)
{
  switchStatus4 = param.asInt(); // Get value as integer
}

// Update manualSwitch4
BLYNK_WRITE(V3)
{
  manualSwitch4 = param.asInt();
}
//======Update switchStatus4 for cooling fan========//
//===============ETT3IN1==============//
void ett3in1()
{
  uint8_t result1;
  float temp1 = (node1.getResponseBuffer(1) / 10.0f);
  float humi1 = (node1.getResponseBuffer(2) / 10.0f);
  float lux1 = (node1.getResponseBuffer(4));
  float lux_per1;
  Serial.println("ETT3IN1 Data1");
  result1 = node1.readInputRegisters(0x0000, 5); // Read 3 registers starting at 1)
  if (result1 == node1.ku8MBSuccess)
  {
    /* Serial.print("Temperature: ");
      Serial.print(node1.getResponseBuffer(1) / 10.0f);
      Serial.println(" C");
      Serial.print("Humidity: ");
      Serial.print(node1.getResponseBuffer(2) / 10.0f);
      Serial.println(" %RH"); */
    lux_per1 = (lux1 = node1.getResponseBuffer(4));
    //lux_per1 = map(lux_per1, 0, 65535, 0, 100);
    /* Serial.print("lux1: ");
      Serial.print(lux_per1);
      Serial.println("%");*/
  }
  Blynk.virtualWrite(V31, temp1);
  Blynk.virtualWrite(V32, humi1);
  Blynk.virtualWrite(V33, lux1);

  //=============Auto/Manual Sw2================//
  if (switchStatus2)
  {
    // auto
    if (temp1 < TempLevelLimit2)
    {
      master_relay.writePin(RELAY_OUT1_PIN, RELAY_ON);  //ต่อไปที่ Heater
      Blynk.virtualWrite(V1, 1);

      //LED Status
      Blynk.setProperty(Widget_LED_sw2_heater, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw2_heater, "label", "เปิด Heater");
      LedBlynkSW2.on();
    }
    else
    {

      master_relay.writePin(RELAY_OUT1_PIN, RELAY_OFF);
      Blynk.virtualWrite(V1, 0);

      //LED Status
      Blynk.virtualWrite(Widget_LED_sw2_heater, 0);
      Blynk.setProperty(Widget_LED_sw2_heater, "label", "ปิด Heater");
      LedBlynkSW2.off();

      //Line notify
      LINE.notify("" + String("อุณหภูมิ") + " " + String(temp1) + " " + "องศาเซลเซียส" + " " + "อยู่ในเกณฑ์ปกติ" + " " + "ปิดตัวทำความร้อน");


    }
  }
  else
  {
    if (manualSwitch2)
    {
      master_relay.writePin(RELAY_OUT1_PIN, RELAY_ON);
      //LED Status
      Blynk.setProperty(Widget_LED_sw2_heater, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw2_heater, "label", "เปิด Heater");
      LedBlynkSW2.on();
    }
    else
    {
      master_relay.writePin(RELAY_OUT1_PIN, RELAY_OFF);
      //LED Status
      Blynk.setProperty(Widget_LED_sw2_heater, "label", "ปิด Heater");
      LedBlynkSW2.off();
    }
    // manaul
  }
  //=============Auto/Manual Sw2================//


  //=============Auto/Manual Sw3================//
  if (switchStatus3)
  {
    // auto
    if (humi1 < HumiLevellimit3)
    {
      master_relay.writePin(RELAY_OUT2_PIN, RELAY_ON);  //ต่อไปที่ตัวพ่นหมอก
      Blynk.virtualWrite(V2, 1);

      //LED Status
      Blynk.setProperty(Widget_LED_sw3_ultra, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw3_ultra, "label", "เปิดปั้มพ่นหมอก");
      LedBlynkSW3.on();

      //Line notify
      //LINE.notify(""+String("ความชื้นสัมพัทธ์")+" "+String(humi1)+" "+"เปอร์เซ็นต์"+" "+"ตำกว่าเกณฑ์ที่กำหนด"+" "+"เปิดปั้มพ่นหมอก");
      // lastLineMsghumi1 = 1;
    }
    else
    {

      master_relay.writePin(RELAY_OUT2_PIN, RELAY_OFF);
      Blynk.virtualWrite(V2, 0);

      //LED Status
      Blynk.virtualWrite(Widget_LED_sw3_ultra, 0);
      Blynk.setProperty(Widget_LED_sw3_ultra, "label", "ปิดปั้มพ่นหมอก");
      LedBlynkSW3.off();

      //Line notify
      LINE.notify("" + String("ความชื้นสัมพัทธ์") + " " + String(humi1) + " " + "เปอร์เซ็นต์" + " " + "อยู่ในเกณฑ์ปกติ" + " " + "ปิดปั้มพ่นหมอก");
      //lastLineMsghumi1 = 2;

    }
  }
  else
  {
    if (manualSwitch3)
    {
      master_relay.writePin(RELAY_OUT2_PIN, RELAY_ON);
      //LED Status
      Blynk.setProperty(Widget_LED_sw3_ultra, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw3_ultra, "label", "เปิดปั้มพ่นหมอก");
      LedBlynkSW3.on();
    }
    else
    {
      master_relay.writePin(RELAY_OUT2_PIN, RELAY_OFF);
      //LED Status
      Blynk.setProperty(Widget_LED_sw3_ultra, "label", "ปิดปั้มพ่นหมอก");
      LedBlynkSW3.off();
    }
    // manaul
  }


  //=============Auto/Manual Sw3================//

  //=============Auto/Manual Sw4================//
  if (switchStatus4)
  {
    // auto
    if ((temp1 > TempLevelLimit2 || humi1 > HumiLevellimit3))
    {
      master_relay.writePin(RELAY_OUT3_PIN, RELAY_ON);  //ต่อไปที่พัดลม
      Blynk.virtualWrite(V3, 1);

      //LED Status
      Blynk.setProperty(Widget_LED_sw4_fan, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw4_fan, "label", "เปิดพัดลม");
      LedBlynkSW4.on();

      //Line notify
      LINE.notify("" + String("อุณหภูมิ") + " " + String(temp1) + " " + "องศาเซลเซียส" + " " + String("ความชื้นสัมพัทธ์") + " " + String(humi1) + "เปอร์เซ็นต์" + " " + "สูงกว่าเกณฑ์ที่กำหนด" + " " + "เปิดพัดลม");
      //lastLineMsgtemp1humi1 = 1;
    }
    else
    {
      if ((temp1 < TempLevelLimit2 && humi1 < HumiLevellimit3))
      {
        master_relay.writePin(RELAY_OUT3_PIN, RELAY_OFF);
        Blynk.virtualWrite(V3, 0);
        //lastLineMsgtemp1humi1 = 2;

        //Line notify
        LINE.notify("" + String("อุณหภูมิ") + " " + String(temp1) + " " + "องศาเซลเซียส" + " " + String("ความชื้นสัมพัทธ์") + " " + String(humi1) + "เปอร์เซ็นต์" + " " + "อยู่ในเกณฑ์ปกติ" + " " + "ปิดพัดลม");
        //lastLineMsgtemp1humi1 = 2;

      }
      else
      {

        master_relay.writePin(RELAY_OUT3_PIN, RELAY_ON);
        Blynk.virtualWrite(V3, 1);
        //}

        //LED Status
        Blynk.virtualWrite(Widget_LED_sw4_fan, 1);
        Blynk.setProperty(Widget_LED_sw4_fan, "label", "เปิดพัดลม");
        LedBlynkSW4.on();

        //Line notify
        LINE.notify("" + String("อุณหภูมิ") + " " + String(temp1) + " " + "องศาเซลเซียส" + " " + String("ความชื้นสัมพัทธ์") + " " + String(humi1) + "เปอร์เซ็นต์" + " " + "อยู่ในเกณฑ์ปกติ" + " " + "ปิดพัดลม");
        //lastLineMsgtemp1humi1 = 1;

      }
    }
  }
  else
  {
    if (manualSwitch4)
    {
      master_relay.writePin(RELAY_OUT3_PIN, RELAY_ON);
      //LED Status
      Blynk.setProperty(Widget_LED_sw4_fan, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw4_fan, "label", "เปิดพัดลม");
      LedBlynkSW4.on();

    }
    else
    {
      master_relay.writePin(RELAY_OUT3_PIN, RELAY_OFF);
      //LED Status
      Blynk.setProperty(Widget_LED_sw4_fan, "label", "ปิดพัดลม");
      LedBlynkSW4.off();
    }
    // manaul
  }
  //=============Auto/Manual Sw4================//
}
//===============ETT3IN1==============//

//==Update switchStatus1 on waterlevel for Solinoid==//
BLYNK_WRITE(V10)
{
  switchStatus1 = param.asInt(); // Get value as integer
}
// Update waterlevel setting
BLYNK_WRITE(V11)
{
  WaterLevelLimit1 = param.asInt(); // Get value as integer
}
// Update manualSwitch1
BLYNK_WRITE(V0)
{
  manualSwitch1 = param.asInt();
}
//==Update waterlevel for Solinoid==//

//=======Slave ID 2:Drop Level========//
void drop_level()
{
  uint8_t result2;
  float drop_level = (node2.getResponseBuffer(0) / 100.0f);
  //Serial.println("Drop Level");
  result2 = node2.readHoldingRegisters(0x0000, 7); // Read 2 registers starting at 0)
  if (result2 == node2.ku8MBSuccess)
  {
    //Serial.print("Drop Level : ");
    //Serial.print(node2.getResponseBuffer(0) / 100.0f);
    //Serial.println(" เมตร");
  }
  Blynk.virtualWrite(V34, drop_level);

  //=============Auto/Manual Sw1================//
  if (switchStatus1)
  {
    // auto
    if (drop_level < WaterLevelLimit1)
    {
      master_relay.writePin(RELAY_OUT0_PIN, RELAY_ON);  //ต่อไปที่ solenoid
      Blynk.virtualWrite(V0, 1);

      //LED Status
      Blynk.setProperty(Widget_LED_sw1_solenoid, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw1_solenoid, "label", "เปิด Solenoid");
      LedBlynkSW1.on();
    }
    else
    {

      master_relay.writePin(RELAY_OUT0_PIN, RELAY_OFF);
      Blynk.virtualWrite(V0, 0);

      //LED Status
      Blynk.virtualWrite(Widget_LED_sw1_solenoid, 0);
      Blynk.setProperty(Widget_LED_sw1_solenoid, "label", "ปิด Solenoid");
      LedBlynkSW1.off();

      //Line notify
      //LINE.notify(""+String("อุณหภูมิ")+" "+String(temp1)+" "+"องศาเซลเซียส"+" "+"อยู่ในเกณฑ์ปกติ"+" "+"ปิดตัวทำความร้อน");


    }
  }
  else
  {
    if (manualSwitch1)
    {


      master_relay.writePin(RELAY_OUT0_PIN, RELAY_ON);  //ต่อไปที่ solenoid
      //LED Status
      Blynk.setProperty(Widget_LED_sw1_solenoid, "color", "#C70039");
      Blynk.setProperty(Widget_LED_sw1_solenoid, "label", "เปิด Solenoid");
      LedBlynkSW1.on();
    }
    else
    {
      master_relay.writePin(RELAY_OUT0_PIN, RELAY_OFF);
      //LED Status
      Blynk.setProperty(Widget_LED_sw1_solenoid, "label", "ปิด Solenoid");
      LedBlynkSW1.off();
    }
    // manaul
  }
  //=============Auto/Manual Sw2================//
}
//=======Slave ID 2:Drop Level========//

//=======Slave ID 3:Water Flow========//
void water_flow()
{
  union
  {
    uint32_t i;
    float f;
  } u;

  uint8_t flow = node3.readHoldingRegisters(0x00, 2); // Flow Rate Table Reg 0001-0002
  //Serial.println("flow : " + String(flow,HEX));// to get error type
  if (flow == node3.ku8MBSuccess)
  {
    u.i = (((unsigned long)node3.getResponseBuffer(0x01) << 16) | (node3.getResponseBuffer(0x00))); // shifting 2nd register to left & stiching with 1st register
  }
  //Serial.println("Flow Rate = " + String(u.f, 6) + "m3/h..................Flow Rate");
  delay(100);
  float Ff =  (u.f) * 1000000;
  //Serial.println(Ff);
  int Fi = Ff;
  //Serial.println(Fi);

  unsigned long Fu;
  char Fc[32];
  Fu = Fi;
  sprintf(Fc, "%08x", Fu);
  //Serial.println("String =" + String(Fc)); //Hex String 10 Byte
  Blynk.virtualWrite(V35, flow);


  uint8_t Vel = node3.readHoldingRegisters(0x04, 2); // Velocity Table Reg 0005-0006
  //Serial.println("Vec : " + String(Vel,HEX));// to get error type
  if (Vel == node3.ku8MBSuccess)
  {
    u.i = (((unsigned long)node3.getResponseBuffer(0x01) << 16) | (node3.getResponseBuffer(0x00))); // shifting 2nd register to left & stiching with 1st register
  }
  // Serial.println("Velocity = " + String(u.f, 6) + "m/s....................Velocity");
  delay(100);

  float Vf =  (u.f) * 1000000;
  //Serial.println(Vf);
  int Vi = Vf;
  //Serial.println(Vi);

  unsigned long Vu;
  char Vc[32];
  Vu = Vi;
  sprintf(Vc, "%08x", Vu);
  // Serial.println("String =" + String(Vc)); //Hex String 5Byte
  Blynk.virtualWrite(V36, Vel);

  uint8_t Day = node3.readHoldingRegisters(0x7c, 2); // Flow fo Today  Table Reg 0125-0126
  //Serial.println("Day : " + String(Day,HEX));// to get error type
  if (Day == node3.ku8MBSuccess)
  {
    u.i = (((unsigned long)node3.getResponseBuffer(0x01) << 16) | (node3.getResponseBuffer(0x00))); // shifting 2nd register to left & stiching with 1st register
  }
  //Serial.println("flow_Day = " + String(u.f, 6) + "m3...................Flow for to day");
  delay(100);

  float Df =  (u.f) * 1000000;
  //Serial.println(Df);
  int Di = Df;
  //Serial.println(Di);

  unsigned long Du;
  char Dc[32];
  Du = Di;
  sprintf(Dc, "%08x", Du);
  //Serial.println ("String =" + String(Dc)); // Hex String 5Byte
  Blynk.virtualWrite(V37, Day);

  uint8_t Month = node3.readHoldingRegisters(0x7e, 2); // Flow for Month  Reg 0127-0128
  //Serial.println("Month : " + String(Month,HEX));// to get error type
  if (Month == node3.ku8MBSuccess)
  {
    u.i = (((unsigned long)node3.getResponseBuffer(0x01) << 16) | (node3.getResponseBuffer(0x00))); // shifting 2nd register to left & stiching with 1st register
  }

  //Serial.println("flow_Month = " + String(u.f, 6) + "m3....................Flow for to this Month");
  delay(100);

  float Mf =  (u.f) * 1000000;
  //Serial.println(Mf);
  int Mi = Mf;
  //Serial.println(Mi);
  String Ms = String(Mi, HEX);
  //Serial.println (Ms);

  unsigned long Mu;
  char Mc[32];
  Mu = Mi;
  sprintf(Mc, "%08x", Mu);
  //Serial.println ("String =" + String(Mc)); //Hex String 5 Byte
  delay(100);
  Blynk.virtualWrite(V38, Month);

  uint8_t Yearf;
  int Yd;
  char Yc[6];
  Yearf = node3.readHoldingRegisters(0x90, 1);    // Flow for year  Reg 145(unsign)
  if (Yearf == node3.ku8MBSuccess) {

    Yd = node3.getResponseBuffer(0x00);
    sprintf( Yc, "%04x", Yd);
  }

  // Serial.println("flow_year(Int) = " + String(Yd) + "m3....................Flow for this Year");
  // Serial.println ("String =" + String(Yc)); //Hex String 2 Byte
  //Blynk.virtualWrite(V39, Yearf);
}
//=======Slave ID 3:Water Flow========//

//===========Slave ID 4:CO2===========//
void co2()
{
  uint8_t result4;
  float co2 = (node4.getResponseBuffer(5));
  //Serial.println("CO2");
  result4 = node4.readHoldingRegisters(0x0000, 7); // Read 2 registers starting at 0)
  if (result4 == node4.ku8MBSuccess)
  {
    //Serial.print("CO2 : ");
    //Serial.print(node4.getResponseBuffer(5));
    // Serial.println(" ppm");
  }
  Blynk.virtualWrite(V40, co2);
}
//===========Slave ID 4:CO2===========//

//===========Slave ID 5:PH============//
void ph()
{
  uint8_t result5;
  float temp = (node5.getResponseBuffer(0) / 10.0f);
  float ph = (node5.getResponseBuffer(1) / 10.0f);
  //Serial.println("PH Sensor Data");
  result5 = node5.readHoldingRegisters(0x0000, 4); // Read 2 registers starting at 0)
  if (result5 == node5.ku8MBSuccess)
  {
    /* Serial.print("Temperature : ");
      Serial.print(node5.getResponseBuffer(0) / 10.0f);
      Serial.println(" C");

      Serial.print("PH : ");
      Serial.print(node5.getResponseBuffer(1) / 10.0f); */
  }
  Blynk.virtualWrite(V41, temp);
  Blynk.virtualWrite(V42, ph);
}
//===========Slave ID 5:PH============//

//=========Slave ID 6:PZEM-AC=========//
void pzem()
{
  uint8_t result6;
  result6 = node6.readInputRegisters(0x0000, 9); //read the 9 registers of the PZEM-014 / 016
  if (result6 == node6.ku8MBSuccess)
  {
    uint32_t tempdouble = 0x00000000;

    float pz_voltage = node6.getResponseBuffer(0x0000) / 10.0;  //get the 16bit value for the voltage, divide it by 10 and cast in the float variable

    tempdouble =  (node6.getResponseBuffer(0x0002) << 16) + node6.getResponseBuffer(0x0001);  // Get the 2 16bits registers and combine them to an unsigned 32bit
    float pz_current = tempdouble / 1000.00;   // Divide the unsigned 32bit by 1000 and put in the current float variable

    tempdouble =  (node6.getResponseBuffer(0x0004) << 16) + node6.getResponseBuffer(0x0003);
    float pz_power = tempdouble / 10.0;

    tempdouble =  (node6.getResponseBuffer(0x0006) << 16) + node6.getResponseBuffer(0x0005);
    float pz_energy = tempdouble;

    float pz_hz = node6.getResponseBuffer(0x0007) / 10.0;
    float pz_pf = node6.getResponseBuffer(0x0008) / 100.00;

    Serial.print(pz_voltage, 1);  // Print Voltage with 1 decimal
    Serial.print("V   ");

    Serial.print(pz_current, 3);
    Serial.print("A   ");

    Serial.print(pz_power, 1);
    Serial.print("W  ");

    Serial.print(pz_energy, 0);
    Serial.print("Wh  ");

    Serial.print(pz_pf, 2);
    Serial.print("pf   ");

    Serial.print(pz_hz, 1);
    Serial.print("Hz   ");

    Serial.println();
    Blynk.virtualWrite(V43, pz_voltage);
    Blynk.virtualWrite(V44, pz_current);
    //Blynk.virtualWrite(V45, pz_power);
    Blynk.virtualWrite(V46, pz_energy);
    delay(1000);
  }

}
//=========Slave ID 6:PZEM-AC=========//

//========Reset energy counter========//
void resetEnergy(uint8_t slaveAddr)
{
  uint16_t u16CRC = 0xFFFF;
  static uint8_t resetCommand = 0x42;
  u16CRC = crc16_update(u16CRC, slaveAddr);
  u16CRC = crc16_update(u16CRC, resetCommand);
  Serial.println("Resetting Energy");
  //preTransmission();
  Serial2.write(slaveAddr);
  Serial2.write(resetCommand);
  Serial2.write(lowByte(u16CRC));
  Serial2.write(highByte(u16CRC));
  delay(10);
  //postTransmission();
  delay(100);
  while (Serial2.available()) {         // Prints the response from the Pzem, do something with it if you like
    Serial.print(char(Serial2.read()), HEX);
    Serial.print(" ");
  }
}
//========Reset energy counter========//

//========Reset energy counter========//
BLYNK_WRITE(V47)
{
  if (param.asInt() == 1)
  {
    Reset_Energy = Reset_Energy + 1;
    Serial.println(Reset_Energy);
    if (Reset_Energy > 2) {
      Serial.println("ResetEnergy");
      uint16_t u16CRC = 0xFFFF;
      static uint8_t resetCommand = 0x42;
      uint8_t slaveAddr = pzemSlaveAddr;
      u16CRC = crc16_update(u16CRC, slaveAddr);
      u16CRC = crc16_update(u16CRC, resetCommand);
      Serial2.write(slaveAddr);
      Serial2.write(resetCommand);
      Serial2.write(lowByte(u16CRC));
      Serial2.write(highByte(u16CRC));
      delay(100);
    }
  }
}
//========Reset energy counter========//


//==========Send data to google sheet=========//
void sendData2GGSheet() {

  uint8_t result1;
  float temp1 = (node1.getResponseBuffer(1) / 10.0f);
  float humi1 = (node1.getResponseBuffer(2) / 10.0f);
  float lux1 = (node1.getResponseBuffer(4));
  //float lux_per1;
  //lux_per1 = (lux1 = node1.getResponseBuffer(4));
  //lux_per1 = map(lux_per1, 0, 65535, 0, 100);

  HTTPClient http;
  //String url = "https://script.google.com/macros/s/" + GAS_ID + "/exec?temp=" + temp1 + "&humi=" + humi1 + "&lux=" + lux_per1;
  String url = "https://script.google.com/macros/s/" + GAS_ID + "/exec?temp1=" + temp1 + "&humi1=" + humi1 + "&lux1=" + lux1;
  Serial.print(url);
  Serial.println("Posting Temp, humi and lux data to Google Sheet");
  //---------------------------------------------------------------------
  //starts posting data to google sheet
  http.begin(url.c_str());
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int httpCode = http.GET();
  Serial.print("HTTP Status Code: ");
  Serial.println(httpCode);
  //---------------------------------------------------------------------
  //getting response from google sheet
  String payload;
  if (httpCode > 0) {
    payload = http.getString();
    Serial.println("Payload: " + payload);
  }
  //---------------------------------------------------------------------
  http.end();
}
//==========Send data to google sheet=========//
//=========Display Date&Time==========//
void datetime() {

  // Initialize a NTPClient to get time
  timeClient.begin();
  // Set offset time in seconds to adjust for your timezone, for example:
  // GMT +1 = 3600
  // GMT +8 = 28800
  // GMT -1 = -3600
  // GMT 0 = 0
  timeClient.setTimeOffset(25200);

  while (!timeClient.update()) {
    timeClient.forceUpdate();
  }
  // The formattedDate comes with the following format:
  // 2018-05-28T16:00:13Z
  // We need to extract date and time
  formattedDate = timeClient.getFormattedTime();
  // Serial.println(formattedDate);

  // Extract date
  int splitT = formattedDate.indexOf("T");
  dayStamp = formattedDate.substring(0, splitT);
  //Serial.print("DATE: ");
  //Serial.println(dayStamp);
  //Blynk.virtualWrite(V7, dayStamp);

  // Extract time
  timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
  //Serial.print("HOUR: ");
  //Serial.println(timeStamp);
  Blynk.virtualWrite(V8, timeStamp);
  delay(1000);
}
//=========Display Date&Time==========//

//===========Blynk conneted===========//
BLYNK_CONNECTED()
{
  if (Blynk.connected())
  {
    //master_relay.writePin(RELAY_OUT0_PIN, RELAY_ON);
    //Serial.println("Blynk connected");
    Blynk.syncAll();
  }
}
//===========Blynk conneted===========//

//============Loop Function===========//
void loop() {
  BlynkEdgent.run();
  timer.run();
}
//============Loop Function===========//
