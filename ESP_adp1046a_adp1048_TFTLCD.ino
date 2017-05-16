// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#define TFT_DC 2 // D4
#define TFT_CS 0 // D3
int lcd_line = 20;
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// esp8266 http://microcontrollerkits.blogspot.sg/2015/11/esp8266-wifi-tftlcd.html
//TFT LCD Pin1 VCC
//TFT LCD Pin2 GND
//TFT LCD Pin3 CS  to GPIO_0            = D3
//TFT LCD Pin4 RST to RST or 3V3 or 5V
//TFT LCD Pin5 DC to GPIO_2             = D4
//TFT LCD Pin6 MOSI to GPIO_13          = D7
//TFT LCD Pin7 CLK to GPIO_14           = D5
//TFT LCD Pin8 LED to +3.3 V.
//TFT LCD Pin9 MISO GPIO_12 ( optionl ) = D6 (for reading the TFT)

#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Wire.h> // 
#include <EEPROM.h>
const char* ssid = "SingaporeMilitia";
const char* password = "123Qweasd";
bool wifi_enabled = false;
bool isBlynkConnected, isWifiConnected, isI2CdevicePresent;

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
char auth[] = "0b3aa4daf1c945d6a8e440048eb379f1";

#include <SimpleTimer.h>

#include <ThingSpeak.h>
unsigned long myChannelNumber = 242527;
const char * myWriteAPIKey = "QWBPI46Y26QNT8AV";
WiFiClient  client;

WidgetTerminal terminal(V0);
WidgetLED statled(V1);
SimpleTimer timerScope1046a, timerBlink, timerScope1048, timerThinkSpeak;
int timerScope1046a_id, timerScope1048_id, timerBlink_id, timerThinkSpeak_id;
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

#define page0 0x8b
#define page1 0x8c
#define page2 0x8d
#define page3 0x8e
#define page4 0x8f
#define page5 0x90
#define flag_id 0x10 // First flag ID
#define rtd_curr 0x11 // RTD current source
#define hf_adc 0x12 // HF ADC reading
#define cs1 0x13 // CS1 value (input current)
#define acsns 0x14 // ACSNS value
#define vs1 0x15 // VS1 voltage value
#define vs2 0x16 // VS2 voltage value
#define vs3 0x17 // VS3 voltage value (output voltage)
#define cs2 0x18 // CS2 value (output current)
#define power 0x19 // CS2 × VS3 value (output power)
#define rtd_temp 0x1A // RTD temperature value
#define temp 0x1B // Read temperature
#define R1 6
#define R2 7
#define Rsense 4
#define Vo_LSB 390.625
byte i2c_address;
byte regs[256];
byte PSON_old, PSON_new;
bool enableDatalog = false;

#define page0_1048 0xb0
#define page1_1048 0xb1
#define page2_1048 0xb2
#define page3_1048 0xb3
#define page4_1048 0xb4
#define page5_1048 0xb5

void setup() {

  Serial.begin(115200);Serial.println("");
  Serial.println("Booting");
  pinMode(10, INPUT_PULLUP);
  EEPROM.begin(256);

  init_tft_ldc();

  init_wifi();

  init_i2c();

  init_i2cDevices();

  //init_timer();

  /*
      Testing which cause red led on pgood1-2
  */
  //  write_byte(0x08, 0x08); // or write_byte(0x08, 0x80); // disable flags

  /*
      Always on
  */
  //  write_byte(0x2c, 0xac);

}

bool IoT_enabled = false;

void loop() {

  if (WiFi.status() == WL_CONNECTED)
  {
    isWifiConnected = true;
    ArduinoOTA.handle();
    if (digitalRead(10) == HIGH) // IoT connected
    {

      if (IoT_enabled == false)
      {
        timerBlink.restartTimer(timerBlink_id);
        timerBlink.enable(timerBlink_id);
        IoT_enabled == true;
        init_terminal();
        ThingSpeak.begin(client);
      }
      if (Blynk.connected())
      {
        isBlynkConnected = true;
        Blynk.run();
      }
    } else // no IoT
    {
      timerBlink.disable(timerBlink_id);
      IoT_enabled = false;
    }
  }

  serialEvent();
  timerScope1046a.run();
  timerScope1048.run();
}

void init_tft_ldc()
{
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, lcd_line);
  tft.setTextColor(ILI9341_GREEN);
  //tft.println("1234567890123456789012345");
  tft.println("Device is Booting...");
  tft.setCursor(10, lcd_line += 20);
  tft.println("Trying to connect to wifi");
}

void init_wifi()
{
  WiFi.mode(WIFI_STA);
  for (int cntr = 0; cntr <= 4; cntr++)
  {
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      WiFi.begin(ssid, password);
    Blynk_Delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("no Wifi connection..");
    tft.setCursor(10, lcd_line += 20);
    tft.println("no Wifi connection..");
    isWifiConnected = false;
    Blynk_Delay(3000);
  }
  else
  {
    tft.setCursor(10, lcd_line += 20);
    tft.print("Wifi ok : "); tft.println(WiFi.localIP());
    Blynk_Delay(3000);

    esp_ota();
    isWifiConnected = true;

    Blynk.config(auth);
    for (int cntr = 0; cntr <= 5000; cntr++)
      Blynk.run();

    if (Blynk.connected()) {
      isBlynkConnected = true;
      tft.setCursor(10, lcd_line += 20);
      tft.println("IoT connected..");
      Serial.println("IoT connected..");
      Blynk_Delay(3000);
    } else {
      isBlynkConnected = false;
      tft.setCursor(10, lcd_line += 20);
      tft.println("IoT not connected..");
      Serial.println("IoT not connected..");
      Blynk_Delay(3000);
    }

    if (digitalRead(10) == HIGH)
    {
      tft.setCursor(10, lcd_line += 20);
      tft.println("IoT enabled..");
      Serial.println("IoT enabled..");
      IoT_enabled = true;
      timerBlink_id = timerBlink.setInterval(1000L, timer_blinkLedWidget);
      init_terminal();
      ThingSpeak.begin(client);
    } else // disabled
    {
      tft.setCursor(10, lcd_line += 20);
      tft.println("IoT not enabled..");
      Serial.println("IoT not enabled..");
    }
  }
}

byte i2c_addresses[127];
int nDevices;

void init_i2c()
{
  Wire.begin();

  // i2c scanner
  byte error, address;

  Serial.println("Scanning I2C...");
  //tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(10, lcd_line += 20);
  tft.print("Scanning I2C...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      i2c_addresses[nDevices] = address;
      nDevices++;
    }
    else if (error == 4)
    {
      tft.setCursor(10, lcd_line += 20);
      Serial.print("Unknow error at address ");
      tft.print("Unknow error at");
      tft.setCursor(10, lcd_line += 20);
      Serial.print("0x"); Serial.println(address, HEX);
      tft.print("0x"); tft.println(address, HEX);
    }
  }

  if (nDevices == 0)
  {
    Serial.print("No I2C devices found");
    tft.setCursor(10, lcd_line += 20);
    tft.print("No I2C devices found");
    tft.setCursor(10, lcd_line += 20);
    tft.print("check connections");
    isI2CdevicePresent = false;
  }
  else
  {
    tft.setCursor(10, lcd_line += 20);
    tft.print("I2C device/s found :");
    Serial.print("I2C device/s found :");
    for (int cntr = 0; cntr < nDevices ; cntr++)
    {
      tft.setCursor(10, lcd_line += 20);
      tft.print("0x"); tft.print(i2c_addresses[cntr], HEX);
      Serial.print("0x"); Serial.println(i2c_addresses[cntr], HEX);
    }
    i2c_address = i2c_addresses[0];
    tft.setCursor(10, lcd_line += 20);
    tft.print("done..");
    Serial.println("done\n");
    isI2CdevicePresent = true;
  }
}

void init_i2cDevices()
{
  if (isI2CdevicePresent)
  {
    if ((i2c_address >= 0x50) && (i2c_address <= 0x57))
    {
      tft.setCursor(10, lcd_line += 20);
      tft.print("ADP1046a Scope..");
      Serial.println("ADP1046a Scope..");
      Blynk_Delay(5000);
      init_adp1046a();
      isI2CdevicePresent = true;
      timerScope1046a_id = timerScope1046a.setInterval(25L, timer_displayScope1046a);
      tft.setRotation(2);
      tft.fillScreen(ILI9341_BLACK);
      tft.setTextSize(1);
    }
    else
    {
      if ((i2c_address >= 0x58) && (i2c_address <= 0x5f))
      {
        tft.setCursor(10, lcd_line += 20);
        tft.print("ADP1048 Scope..");
        Serial.println("ADP1048 Scope..");
        Blynk_Delay(5000);
        init_adp1048();
        isI2CdevicePresent = true;
        timerScope1048_id = timerScope1048.setInterval(50L, timer_displayScope1048);
        tft.setRotation(2);
        tft.fillScreen(ILI9341_BLACK);
        tft.setTextSize(1);
      }
    }
  } else
  {

  }
}

float r1, r2, Vo_LSB_const, CS2_adc_range_const, Vout_setting, Iout_setting;
int CS2_adc_range;

void init_adp1046a()
{
  // adp1046a communications
  eeprom_unlock();

  int boardreg1, boardreg2, boardreg3;
  boardreg1 = read_adpeeprom(page2, 0x00);
  boardreg2 = read_adpeeprom(page2, 0x01);
  boardreg3 = read_adpeeprom(page2, 0x02);
  if ((boardreg1 == 1) && (boardreg2 == 2) && (boardreg3 == 3))
  {

  } else
  {
    tft.fillScreen(ILI9341_BLACK);
    tft.setCursor(10, 20);
    tft.print("No board data,");
    tft.setCursor(10, 40);
    tft.print("cannot measure accurate!");
    Blynk_Delay(5000);
  }

  r1 = read_board_reg(R1);
  r2 = read_board_reg(R2);
  Vo_LSB_const = (Vo_LSB * ( r1 + r2 ) / r2) / 1000000;
  Vout_setting = 3584 * Vo_LSB_const;
  //Serial.print("Vout_setting : "); Serial.println(Vout_setting);

  if ((read_reg(0x027) & 0x20)) // check if 60mV or 120mV
    CS2_adc_range = 120;
  else
    CS2_adc_range = 60;

  CS2_adc_range_const = CS2_adc_range / (4096 * read_board_reg(Rsense));
  Iout_setting = CS2_adc_range_const * 3754;
  //Serial.print("Iout_setting : "); Serial.println(Iout_setting);
}

void init_timer()
{
  timerBlink_id = timerBlink.setInterval(1000L, timer_blinkLedWidget);
  timerThinkSpeak_id = timerThinkSpeak.setInterval(20000L, timer_thingSpeakDatalog);
  //timerScope_id = timer.setInterval(25L, timer_displayScope);
}

int  msg_cntr = 0;
int code = 0, reg, data, start_reg, end_reg, eeprompage;

BLYNK_WRITE(V0)
{
  String terminalString;
  int reciv, opcode;
  char c_opcode[3];

  terminalString = param.asStr();
  reciv = terminalString.toInt();

  if ((terminalString == "RESET") || (terminalString == "reset"))
  {
    msg_cntr = 0;
    reciv = 0;
  }

  if ((terminalString == "HELP") || (terminalString == "help"))
  {
    terminal.println("Codes :");
    terminal.println("1 = Send Byte/Command");
    terminal.println("2 = Write Byte to Reg");
    terminal.println("3 = Display reg/s");
    terminal.println("4 = Display 1 reg only");
    terminal.println("reset = reset the input codes");
    terminal.println("help = this display");
    terminal.flush();
  }

  if (msg_cntr == 0)
  {
    code = reciv;
    msg_cntr++;

    // swtich code here??
  }

  switch (code)
  {
    case 0: // reset
      msg_cntr = 0;
      reciv = 0;
      terminal.println("Reseting...");
      terminal.flush();
      break;

    case 1: // write/send byte (1 byte)
      switch (msg_cntr)
      {
        case 1:
          terminal.println("Sendbyte, send nxt hex for command");
          terminal.flush();
          msg_cntr++;
          break;
        case 2:
          terminalString.toCharArray(c_opcode, 3);
          reg = strtoul(c_opcode, NULL, 16);
          send_byte_command(reg);
          msg_cntr = 0;
          reciv = 0;
          break;
      }

      //      if (msg_cntr == 2)
      //      {
      //        terminalString.toCharArray(c_opcode, 3);
      //        reg = strtoul(c_opcode, NULL, 16);
      //        terminal.print("reg : ");
      //        terminal.println(reg);
      //        send_byte_command(reg);
      //        msg_cntr = 0;
      //        reciv = 0;
      //      } else
      //      {
      //        terminal.println("Sendbyte, send nxt hex for command");
      //        msg_cntr++;
      //      }

      break;

    case 2: // write reg (2 bytes = 1 reg + 1 data)
      switch (msg_cntr)
      {
        case 1:
          terminal.println("Writebyte, send nxt hex for reg address");
          terminal.flush();
          msg_cntr++;
          break;
        case 2:
          terminalString.toCharArray(c_opcode, 3);
          reg = strtoul(c_opcode, NULL, 16);
          terminal.println("Writebyte, send nxt hex for reg data");
          terminal.flush();
          msg_cntr++;
          break;
        case 3:
          terminalString.toCharArray(c_opcode, 3);
          data = strtoul(c_opcode, NULL, 16);
          terminal.flush();
          write_byte(reg, data);
          msg_cntr = 0;
          reciv = 0;
          break;
      }
      break;
    case 3: // read/display regs
      switch (msg_cntr)
      {
        case 1:
          terminal.println("Read reg/s, send nxt hex for start reg address");
          terminal.flush();
          msg_cntr++;
          break;
        case 2:
          terminalString.toCharArray(c_opcode, 3);
          start_reg = strtoul(c_opcode, NULL, 16);
          terminal.println("Read reg/s, send nxt hex for end reg address");
          terminal.flush();
          msg_cntr++;
          break;
        case 3:
          terminalString.toCharArray(c_opcode, 3);
          end_reg = strtoul(c_opcode, NULL, 16);
          terminal.flush();
          display_regs(start_reg, end_reg);
          msg_cntr = 0;
          reciv = 0;
          break;
      }
      break;

    case 4: // display 1 reg
      switch (msg_cntr)
      {
        case 1:
          terminal.println("Read 1 reg, send nxt hex for reg address");
          terminal.flush();
          msg_cntr++;
          break;
        case 2:
          terminalString.toCharArray(c_opcode, 3);
          reg = strtoul(c_opcode, NULL, 16);
          terminal.println(read_reg(reg), HEX);
          terminal.flush();
          msg_cntr = 0;
          reciv = 0;
          break;
      }
      break;

    case 5: // eeprom_unlock()
      eeprom_unlock();
      terminal.println("EEprom Unlocked");
      terminal.flush();
      msg_cntr = 0;
      reciv = 0;
      break;

    case 6: // display_adpeeprom(char page, char start_add, char end_add)
      eeprom_unlock();
      switch (msg_cntr)
      {
        case 1:
          terminal.println("Display Eeprom page no.");
          terminal.flush();
          msg_cntr++;
          break;
        case 2:
          terminalString.toCharArray(c_opcode, 3);
          eeprompage = strtoul(c_opcode, NULL, 16);
          terminal.println("Eeprom start address");
          terminal.flush();
          msg_cntr++;
          break;
        case 3:
          terminalString.toCharArray(c_opcode, 3);
          start_reg = strtoul(c_opcode, NULL, 16);
          terminal.println("Eeprom end address");
          terminal.flush();
          msg_cntr++;
          break;
        case 4:
          terminalString.toCharArray(c_opcode, 3);
          end_reg = strtoul(c_opcode, NULL, 16);
          display_adpeeprom((eeprompage + 0x8b), start_reg, end_reg);
          msg_cntr = 0;
          reciv = 0;
          break;
      }
      break;

    case 7: // esp_eeprom_display
      display_esp_eeprom();
      msg_cntr = 0;
      reciv = 0;
      break;

    case 8: // save_userreg_espeeprom
      save_userreg_espeeprom();
      msg_cntr = 0;
      reciv = 0;
      break;

    case 9: // espeeprom_to_reg
      espeeprom_to_reg();
      msg_cntr = 0;
      reciv = 0;
      break;

    case 10: // espeeprom_to_reg and save to adp eeprom
      espeeprom_to_reg();
      send_byte_command(0x82); // store reg to user eeprom
      msg_cntr = 0;
      reciv = 0;
      break;

    case 11: // reg to espeeprom
      save_userreg_espeeprom();
      msg_cntr = 0;
      reciv = 0;
      break;

    case 12: // measure V & I @ terminal
      eeprom_unlock();
      //char read_adpeeprom(char page, char eeprom_add)
      int boardreg1, boardreg2, boardreg3;
      boardreg1 = read_adpeeprom(page2, 0x00);
      boardreg2 = read_adpeeprom(page2, 0x01);
      boardreg3 = read_adpeeprom(page2, 0x02);
      if ((boardreg1 == 1) && (boardreg2 == 2) && (boardreg3 == 3))
      {
        // continue
        //        v_lsb = read_reg();
        //        v_msb = read_reg();
        //        r1_lsb_man = read_adpeeprom(page2, reg);
        //        r1_lsb_man = read_adpeeprom(page2, reg);
        //        r1_lsb_man = read_adpeeprom(page2, reg);
        //        r1_lsb_man = read_adpeeprom(page2, reg);
        //        r1_lsb_man = read_adpeeprom(page2, reg);
        //        r1_lsb_man = read_adpeeprom(page2, reg);
        Serial.println("board data ok");
        terminal.println("board data ok");
        terminal.flush();
      } else
      {
        Serial.println("No board data, cannot measure accurate!");
        terminal.println("No board data, cannot measure accurate!");
        terminal.flush();
      }
      display_Vout_Iout();
      msg_cntr = 0;
      reciv = 0;
      break;

    case 13: // measure V and I to thingSpeak
      switch (msg_cntr)
      {
        case 1:
          terminal.println("Enable/Disable Web Datalog");
          terminal.flush();
          msg_cntr++;
          break;
        case 2:
          terminalString.toCharArray(c_opcode, 3);
          reg = strtoul(c_opcode, NULL, 16);
          if (reg == 1)
          {
            enableDatalog = true;
            terminal.println("Web Datalog : Enabled");
            terminal.flush();
          } else
          {
            enableDatalog = false;
            terminal.println("Web Datalog : Disabled");
            terminal.flush();
          }
          msg_cntr = 0;
          reciv = 0;
          break;
      }
      break;

    case 14: // Change i2c address

      break;

    case 99: // i2c scanner

      byte error, address;
      int nDevices;

      Serial.println("Scanning...");
      terminal.println("Scanning...");

      nDevices = 0;
      for (address = 1; address < 127; address++ )
      {
        // The i2c_scanner uses the return value of
        // the Write.endTransmisstion to see if
        // a device did acknowledge to the address.
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
          Serial.print("I2C device found at address 0x");
          terminal.print("I2C device found at address 0x");
          if (address < 16)
          {
            Serial.print("0");
            terminal.print("0");
          }
          Serial.print(address, HEX);
          terminal.print(address, HEX);
          Serial.println("  !");
          terminal.println("  !");

          nDevices++;
        }
        else if (error == 4)
        {
          Serial.print("Unknow error at address 0x");
          terminal.print("Unknow error at address 0x");
          if (address < 16)
          {
            Serial.print("0");
            terminal.print("0");
          }
          Serial.println(address, HEX);
          terminal.println(address, HEX);
        }
      }
      if (nDevices == 0)
      {
        Serial.println("No I2C devices found\n");
        terminal.println("No I2C devices found\n");
      }
      else
      {
        Serial.println("done\n");
        terminal.println("done\n");
      }
      terminal.flush();
      msg_cntr = 0;
      reciv = 0;
      break;
  }

}

BLYNK_WRITE(V2) // SW PS on
{
  if (param.asInt())
  {
    PS_on();
  } else
  {
    PS_off();
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }

  if (stringComplete) {
    Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

void init_terminal()
{
  terminal.println(F("v" BLYNK_VERSION ": Device started"));
  terminal.println(F("----------------------------"));
  terminal.println("Codes :");
  terminal.println("1 = Send Byte/Command");
  terminal.println("2 = Write Byte to Reg");
  terminal.println("3 = Display reg/s");
  terminal.println("4 = Display 1 reg only");
  terminal.println("reset = reset the input codes");
  terminal.println("help = this display");
  terminal.println(F("----------------------------"));
  terminal.flush();
}

void timer_blinkLedWidget()
{
  if (Blynk.connected()) {
    if (statled.getValue()) {
      statled.off();
    } else {
      statled.on();
    }
  }
}

void timer_thingSpeakDatalog()
{
  if (enableDatalog)
  {
    float Vout1 = Vo_LSB_const * read_reg_12bit(vs1);
    float Vout2 = Vo_LSB_const * read_reg_12bit(vs2);
    float Vout3 = Vo_LSB_const * read_reg_12bit(vs3);

    float Iout = read_reg_12bit(cs2) * CS2_adc_range_const;

    Serial.print("VS1 : "); Serial.println(Vout1);
    Serial.print("VS2 : "); Serial.println(Vout2);
    Serial.print("VS3 : "); Serial.println(Vout3);
    terminal.print("VS1 : "); terminal.println(Vout1);
    terminal.print("VS2 : "); terminal.println(Vout2);
    terminal.print("VS3 : "); terminal.println(Vout3);

    Serial.print("Iout : "); Serial.println(Iout);
    terminal.print("Iout : "); terminal.println(Iout);

    float Power = read_reg_16bit(power);
    Serial.print("Pout : "); Serial.println(Power);
    terminal.print("Pout : "); terminal.println(Power);
    terminal.flush();

    ThingSpeak.setField(1, Vout1); // <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/242527/charts/1?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&type=line&update=15"></iframe>
    ThingSpeak.setField(2, Vout2); // <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/242527/charts/2?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&type=line&update=15"></iframe>
    ThingSpeak.setField(3, Vout3); // <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/242527/charts/2?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&type=line&update=15"></iframe>
    ThingSpeak.setField(4, Iout);  // <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/242527/charts/4?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&type=line&update=15"></iframe>
    ThingSpeak.setField(5, Power); // <iframe width="450" height="260" style="border: 1px solid #cccccc;" src="https://thingspeak.com/channels/242527/charts/5?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&type=line&update=15"></iframe>
    ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey); // https://thingspeak.com/channels/242527
  } else
  {

  }
}

void PS_on()
{
  PSON_old = read_reg(0x2c);
  PSON_new = (PSON_old & 0x1f) | 0x80; // enable Software PS_ON
  PSON_new = PSON_new | 0x20; // power on PSON
  write_byte(0x2c, PSON_new);
}

void PS_off()
{
  PSON_old = read_reg(0x2c);
  PSON_new = (PSON_old & 0x1f) | 0x80; // enable Software PS_ON
  PSON_new = PSON_new & 0xdf; // power on PSOff
  write_byte(0x2c, PSON_new);
}

bool is_120mV()
{
  bool is_120mV;
  byte cs_FS;

  cs_FS = read_reg(0x027);
  is_120mV = (cs_FS & 0x20);
  return is_120mV;
}

void display_Vout_Iout()
{
  float Vout1 = Vo_LSB_const * read_reg_12bit(vs1);
  float Vout2 = Vo_LSB_const * read_reg_12bit(vs2);
  float Vout3 = Vo_LSB_const * read_reg_12bit(vs3);
  float Iout = read_reg_12bit(cs2) * CS2_adc_range_const;

  Serial.print("VS1 : "); Serial.println(Vout1);
  Serial.print("VS2 : "); Serial.println(Vout2);
  Serial.print("VS3 : "); Serial.println(Vout3);
  Serial.print("Iout : "); Serial.println(Iout);

  terminal.print("VS1 : "); terminal.println(Vout1);
  terminal.print("VS2 : "); terminal.println(Vout2);
  terminal.print("VS3 : "); terminal.println(Vout3);
  terminal.print("Iout : "); terminal.println(Iout);
  terminal.flush();
}

void display_all_board_reg()
{
  for (int cntr = 1; cntr <= 41 ; cntr++)
  {
    Serial.print(cntr); Serial.print(" ");
    Serial.println(read_board_reg(cntr));
  }
}

void display_all_board_reg_1048()
{
  for (int cntr = 1; cntr <= 21 ; cntr++)
  {
    Serial.print(cntr); Serial.print(" ");
    Serial.println(read_board_reg_1048(cntr));
  }
}

void espeeprom_to_boardreg() //
{
  for (int cntr = 100; cntr < (42 * 3) ; cntr++)
  {
    write_byte(cntr, EEPROM.read(cntr));
    //espeeprom_cntr++;
  }
}

void espeeprom_to_reg()
{
  int cntr, espeeprom_cntr = 0;

  for ( cntr = 0x08; cntr <= 0x0F ; cntr++)
  {
    write_byte(cntr, EEPROM.read(espeeprom_cntr));
    espeeprom_cntr++;
  }

  write_byte(0x11, EEPROM.read(espeeprom_cntr));
  espeeprom_cntr++;

  write_byte(0x22, EEPROM.read(espeeprom_cntr));
  espeeprom_cntr++;

  for (cntr = 0x26; cntr <= 0x2A ; cntr++)
  {
    write_byte(cntr, EEPROM.read(espeeprom_cntr));
    espeeprom_cntr++;
  }

  for ( cntr = 0x2C; cntr <= 0x37 ; cntr++)
  {
    write_byte(cntr, EEPROM.read(espeeprom_cntr));
    espeeprom_cntr++;
  }

  write_byte(0x3B, EEPROM.read(espeeprom_cntr));
  espeeprom_cntr++;

  for ( cntr = 0x3F; cntr <= 0x5D ; cntr++)
  {
    write_byte(cntr, EEPROM.read(espeeprom_cntr));
    espeeprom_cntr++;
  }

  for ( cntr = 0x5F; cntr <= 0x7D ; cntr++)
  {
    write_byte(cntr, EEPROM.read(espeeprom_cntr));
    espeeprom_cntr++;
  }

}

void save_boardreg_espeeprom()
{
  byte adp_eeprom;
  for (int cntr = 0; cntr < (42 * 3) ; cntr++)
  {
    adp_eeprom = read_adpeeprom(page2, cntr);
    EEPROM.write((cntr + 100), adp_eeprom);
    //    Serial.print("cntr : "); Serial.print(cntr,HEX);
    //    Serial.print(" | adp_eeprom : "); Serial.println(adp_eeprom,HEX);
    Blynk_Delay(1);
  }
  EEPROM.commit();
}

void save_userreg_espeeprom()
{
  int cntr = 0, reg_cntr = 0;

  send_byte_command(0x83); // restore user

  for ( cntr = 0x08; cntr <= 0x0F ; cntr++)
  {
    regs[reg_cntr] = read_reg(cntr);
    reg_cntr++;
  }

  regs[reg_cntr] = read_reg(0x11);
  reg_cntr++;

  regs[reg_cntr] = read_reg(0x22);
  reg_cntr++;

  for (cntr = 0x26; cntr <= 0x2A ; cntr++)
  {
    regs[reg_cntr] = read_reg(cntr);
    reg_cntr++;
  }

  for ( cntr = 0x2C; cntr <= 0x37 ; cntr++)
  {
    regs[reg_cntr] = read_reg(cntr);
    reg_cntr++;
  }

  regs[reg_cntr] = read_reg(0x3B);
  reg_cntr++;

  for ( cntr = 0x3F; cntr <= 0x5D ; cntr++)
  {
    regs[reg_cntr] = read_reg(cntr);
    reg_cntr++;
  }

  for ( cntr = 0x5F; cntr <= 0x7D ; cntr++)
  {
    regs[reg_cntr] = read_reg(cntr);
    reg_cntr++;
  }

  for (cntr = 0; cntr < reg_cntr ; cntr++)
  {
    EEPROM.write(cntr, regs[cntr]);
  }
  EEPROM.commit();
}

void display_esp_eeprom()
{
  for (int cntr = 0; cntr < 256 ; cntr++)
  {
    Serial.print("es");
    Serial.print(cntr); Serial.print(" ");
    Serial.println(EEPROM.read(cntr), HEX);
    //    terminal.print("es");
    //    terminal.print(cntr + 1); Serial.print(" ");
    //    terminal.println(EEPROM.read(cntr), HEX);
    //    terminal.flush();
    Blynk_Delay(1);
  }
}

void eeprom_unlock()
{
  write_byte(0x88, 0xff); //eeprom password command
  write_byte(0x88, 0xff);
  byte result = read_reg(0x03); //check result eeprom unlocked
  result = result & 0x01;
  if (result == 0x01)
    Serial.println("adp1046a eeprom unlocked");
  else
    Serial.println("adp1046a eeprom unlocked error");
}

void eeprom_unlock_1048()
{
  write_byte(0xd5, 0xff);
  write_byte(0xd5, 0xff);
  byte result = read_reg_ext(0xfe81); //check result eeprom unlocked
  result = ((result >> 6) & 0x01);
  if (result == 0x01)
    Serial.println("adp1048 eeprom unlocked");
  else
    Serial.println("adp1048 eeprom unlocked error");
}

void write_byte(byte reg, byte data)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);
  Wire.write(data);
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("write byte nok : ");
    Serial.println(result);
    terminal.print("write byte nok : ");
    terminal.println(result);
    terminal.flush();
  }
}

void write_reg_ext_byte(unsigned int reg_add, byte data)
{
  byte ext_code = byte(reg_add >> 8);
  byte ext_command = byte(reg_add);
  Wire.beginTransmission(i2c_address);
  Wire.write(ext_code);
  Wire.write(ext_command);
  Wire.write(data);
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("write byte nok : ");
    Serial.println(result);
    terminal.print("write byte nok : ");
    terminal.println(result);
    terminal.flush();
  }
}

void send_byte_command(byte data)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(data);
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("send byte nok : ");
    Serial.println(result);
    terminal.print("send byte nok : ");
    terminal.println(result);
    terminal.flush();
  }
}

float read_board_reg(byte eeprom_add)
{
  eeprom_add = eeprom_add * 3;
  int hi_man = read_adpeeprom(page2, eeprom_add);
  int lo_man = read_adpeeprom(page2, ++eeprom_add);
  int8_t exponent = read_adpeeprom(page2, ++eeprom_add);;
  float result;
  result = (hi_man * 256 + lo_man) * pow(2, exponent);
  return result;
}

float read_board_reg_1048(byte eeprom_add)
{
  eeprom_add = eeprom_add * 3;
  int hi_man = read_adpeeprom_1048(page2_1048, eeprom_add);
  int lo_man = read_adpeeprom_1048(page2_1048, ++eeprom_add);
  int8_t exponent = read_adpeeprom_1048(page2_1048, ++eeprom_add);;
  float result;
  result = (hi_man * 256 + lo_man) * pow(2, exponent);
  return result;
}

byte read_adpeeprom(byte page, byte eeprom_add)
{
  // write num of bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(0x86); // num of bytes address
  Wire.write(1); // num of bytes
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in num of bytes : ");
    Serial.println(result);
    terminal.print("Error in num of bytes : ");
    terminal.println(result);
    terminal.flush();
  }

  // offset
  Wire.beginTransmission(i2c_address);
  Wire.write(0x85); // offset address
  Wire.write(eeprom_add); // number of offset
  result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in Offset : ");
    Serial.println(result);
  }
  //  Serial.print("e"); Serial.print(eeprom_add, HEX); Serial.print(" ");

  // read bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(page);
  Wire.endTransmission(false);
  result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error in Read bytes : ");
    Serial.println(result);
  }
  byte num_bytes = Wire.read(); // receive a byte as character
  byte data = Wire.read(); // receive a byte as character
  //  Serial.println(data, HEX); // print the character
  return data;
}

byte read_adpeeprom_1048(byte page, byte eeprom_add)
{
  // write num of bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(0xd2); // num of bytes address
  Wire.write(1); // num of bytes
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in num of bytes : ");
    Serial.println(result);
    terminal.print("Error in num of bytes : ");
    terminal.println(result);
    terminal.flush();
  }

  // offset
  Wire.beginTransmission(i2c_address);
  Wire.write(0xd3); // offset address
  Wire.write(eeprom_add); // number of offset
  result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in Offset : ");
    Serial.println(result);
  }
  //  Serial.print("e"); Serial.print(eeprom_add, HEX); Serial.print(" ");

  // read bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(page);
  Wire.endTransmission(false);
  result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error in Read bytes : ");
    Serial.println(result);
  }
  byte num_bytes = Wire.read(); // receive a byte as character
  byte data = Wire.read(); // receive a byte as character
  //  Serial.println(data, HEX); // print the character
  return data;
}

void write_adpeeprom(byte page, byte adpeeprom_add, byte adpeeprom_data)
{
  // write num of bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(0x86); // num of bytes address
  Wire.write(1); // num of bytes
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in num of bytes : ");
    Serial.println(result);
    terminal.print("Error in num of bytes : ");
    terminal.println(result);
    terminal.flush();
  }

  // offset
  Wire.beginTransmission(i2c_address);
  Wire.write(0x85); // offset address
  Wire.write(adpeeprom_add); // number of offset
  result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in Offset : ");
    Serial.println(result);
    terminal.print("Error in Offset : ");
    terminal.println(result);
    terminal.flush();
  }

  // write byte
  Wire.beginTransmission(i2c_address);
  Wire.write(page);
  Wire.write(1); // byte count
  Wire.write(adpeeprom_data); // data
  result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in write byte : ");
    Serial.println(result);
    terminal.print("Error in write byte : ");
    terminal.println(result);
    terminal.flush();
  }
}

void display_adpeeprom(byte page, byte start_add, byte end_add)
{
  // write num of bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(0x86); // num of bytes address
  Wire.write(1); // num of bytes
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in num of bytes : ");
    Serial.println(result);
    terminal.print("Error in num of bytes : ");
    terminal.println(result);
    terminal.flush();
  }

  for (int cntr = start_add; cntr <= end_add; cntr++)
  {
    // offset
    Wire.beginTransmission(i2c_address);
    Wire.write(0x85); // offset address
    Wire.write(cntr); // number of offset
    int result = Wire.endTransmission();
    if (result != 0)
    {
      Serial.print("Error in Offset : ");
      Serial.println(result);
      terminal.print("Error in Offset : ");
      terminal.println(result);
      terminal.flush();
    }
    Serial.print("e"); Serial.print(cntr, HEX); Serial.print(" ");
    terminal.print("e"); terminal.print(cntr, HEX); terminal.print(" ");

    // read bytes
    Wire.beginTransmission(i2c_address);
    Wire.write(page);
    Wire.endTransmission(false);
    result = Wire.requestFrom(i2c_address, 2, true);
    if (result != 2)
    {
      Serial.print("Error in Read bytes : ");
      Serial.println(result);
      terminal.print("Error in Read bytes : ");
      terminal.println(result);
      terminal.flush();
    }
    byte num_bytes = Wire.read(); // receive a byte as character
    byte data = Wire.read(); // receive a byte as character
    Serial.println(data, HEX); // print the character
    terminal.println(data, HEX); // print the character
    terminal.flush();
  }
}

void display_adpeeprom_1048(byte page, byte start_add, byte end_add)
{
  // write num of bytes
  Wire.beginTransmission(i2c_address);
  Wire.write(0xd2); // num of bytes address
  Wire.write(1); // num of bytes
  int result = Wire.endTransmission();
  if (result != 0)
  {
    Serial.print("Error in num of bytes : ");
    Serial.println(result);
    terminal.print("Error in num of bytes : ");
    terminal.println(result);
    terminal.flush();
  }

  for (int cntr = start_add; cntr <= end_add; cntr++)
  {
    // offset
    Wire.beginTransmission(i2c_address);
    Wire.write(0xd3); // offset address
    Wire.write(cntr); // number of offset
    int result = Wire.endTransmission();
    if (result != 0)
    {
      Serial.print("Error in Offset : ");
      Serial.println(result);
      terminal.print("Error in Offset : ");
      terminal.println(result);
      terminal.flush();
    }
    Serial.print("e"); Serial.print(cntr, HEX); Serial.print(" ");
    terminal.print("e"); terminal.print(cntr, HEX); terminal.print(" ");

    // read bytes
    Wire.beginTransmission(i2c_address);
    Wire.write(page);
    Wire.endTransmission(false);
    result = Wire.requestFrom(i2c_address, 2, true);
    if (result != 2)
    {
      Serial.print("Error in Read bytes : ");
      Serial.println(result);
      terminal.print("Error in Read bytes : ");
      terminal.println(result);
      terminal.flush();
    }
    byte num_bytes = Wire.read(); // receive a byte as character
    byte data = Wire.read(); // receive a byte as character
    Serial.println(data, HEX); // print the character
    terminal.println(data, HEX); // print the character
    terminal.flush();
  }
}

unsigned int read_reg_12bit(byte reg_add)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_add);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
  }
  //  Serial.print("r");
  //  Serial.print(reg_add, HEX);
  //  Serial.print(" ");
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  //  Serial.print(data1, HEX); // print the character
  //  Serial.print(" ");
  //  Serial.print(data2, HEX); // print the character
  //  Serial.print(" ");
  int data = (data1 / 16) + (data2 * 16);
  //  Serial.println(data, HEX); // print the character
  return data;
}

void display_reg_12bit(byte reg_add)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_add);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
  }
  Serial.print("r");
  Serial.print(reg_add, HEX);
  Serial.print(" ");
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  Serial.print(data1, HEX); // print the character
  Serial.print(" ");
  Serial.print(data2, HEX); // print the character
  Serial.print(" ");
  int data = (data1 / 16) + (data2 * 16);
  Serial.println(data, HEX); // print the character
}

unsigned int read_reg_16bit(byte reg_add)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_add);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
  }
  //  Serial.print("r");
  //  Serial.print(reg_add, HEX);
  //  Serial.print(" ");
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  //  Serial.print(data1, HEX); // print the character
  //  Serial.print(" ");
  //  Serial.print(data2, HEX); // print the character
  //  Serial.print(" ");
  int data = data1 + (data2 * 256);
  //  Serial.println(data, HEX); // print the character
  return data;
}

void display_reg_16bit(byte reg_add)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_add);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
    terminal.print("Error reading register : ");
    terminal.print(result);
    terminal.print(" ");
  }
  Serial.print("r");
  Serial.print(reg_add, HEX);
  Serial.print(" ");
  terminal.print("r");
  terminal.print(reg_add, HEX);
  terminal.print(" ");
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  Serial.print(data1, HEX); // print the character
  Serial.print(" ");
  Serial.print(data2, HEX); // print the character
  Serial.print(" ");
  terminal.print(data1, HEX); // print the character
  terminal.print(" ");
  terminal.print(data2, HEX); // print the character
  terminal.print(" ");
  int data = data1 + (data2 * 256);
  Serial.println(data, HEX); // print the character
  terminal.println(data, HEX); // print the character
  terminal.flush();
}

void display_reg_ext_16bit(unsigned int reg_add)
{
  byte ext_code = byte(reg_add >> 8);
  byte ext_command = byte(reg_add);
  Wire.beginTransmission(i2c_address);
  Wire.write(ext_code);
  Wire.write(ext_command);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
    terminal.print("Error reading register : ");
    terminal.print(result);
    terminal.print(" ");
  }
  Serial.print("r");
  Serial.print(reg_add, HEX);
  Serial.print(" ");
  terminal.print("r");
  terminal.print(reg_add, HEX);
  terminal.print(" ");
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  Serial.print(data1, HEX); // print the character
  Serial.print(" ");
  Serial.print(data2, HEX); // print the character
  Serial.print(" ");
  terminal.print(data1, HEX); // print the character
  terminal.print(" ");
  terminal.print(data2, HEX); // print the character
  terminal.print(" ");
  int data = data1 + (data2 * 256);
  Serial.println(data, HEX); // print the character
  terminal.println(data, HEX); // print the character
  terminal.flush();
}

void display_reg_ext(unsigned int reg_add)
{
  byte ext_code = byte(reg_add >> 8);
  byte ext_command = byte(reg_add);
  Wire.beginTransmission(i2c_address);
  Wire.write(ext_code);
  Wire.write(ext_command);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 1, true);
  if (result != 1)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
  }
  Serial.print("r");
  Serial.print(reg_add, HEX);
  Serial.print(" ");
  byte data = Wire.read(); // receive a byte as character
  Serial.println(data, HEX); // print the character
}

void display_reg_word(byte reg_add)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_add);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
  }
  Serial.print("r");
  Serial.print(reg_add, HEX);
  Serial.print(" ");
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  Serial.print(data1, HEX); // print the character
  Serial.print(" ");
  Serial.println(data2, HEX); // print the character
  Blynk_Delay(100);
}

byte read_reg(byte reg_add)
{
  Wire.beginTransmission(i2c_address);
  Wire.write(reg_add);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 1, true);
  if (result != 1)
  {
    Serial.print("Error reading register : ");
    Serial.print(result);
    Serial.print(" ");
    terminal.print("Error reading register : ");
    terminal.print(result);
    terminal.print(" ");
    terminal.flush();
  }
  byte data = Wire.read(); // receive a byte as character
  return data;
}

byte read_reg_ext(unsigned int reg_add)
{
  byte ext_code = byte(reg_add >> 8);
  byte ext_command = byte(reg_add);
  Wire.beginTransmission(i2c_address);
  Wire.write(ext_code);
  Wire.write(ext_command);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 1, true);
  if (result != 1)
  {
    Serial.print("Error reading ext register : ");
    Serial.print(result);
    Serial.print(" ");
    terminal.print("Error reading ext register : ");
    terminal.print(result);
    terminal.print(" ");
    terminal.flush();
  }
  byte data = Wire.read(); // receive a byte as character
  return data;
}

unsigned int read_reg_ext_16bit(unsigned int reg_add)
{
  byte ext_code = byte(reg_add >> 8);
  byte ext_command = byte(reg_add);
  Wire.beginTransmission(i2c_address);
  Wire.write(ext_code);
  Wire.write(ext_command);
  Wire.endTransmission(false);
  int result = Wire.requestFrom(i2c_address, 2, true);
  if (result != 2)
  {
    Serial.print("Error reading ext register : ");
    Serial.print(result);
    Serial.print(" ");
    terminal.print("Error reading ext register : ");
    terminal.print(result);
    terminal.print(" ");
    terminal.flush();
  }
  byte data1 = Wire.read(); // receive a byte as character
  byte data2 = Wire.read(); // receive a byte as character
  unsigned int data = data1 + (data2 * 256);
  return data;
}

void display_regs(byte start_reg, byte end_reg)
{
  for (int cntr = start_reg; cntr <= end_reg ; cntr++) {
    Wire.beginTransmission(i2c_address);
    Wire.write(cntr);
    Wire.endTransmission(false);
    int result = Wire.requestFrom(i2c_address, 1, true);
    if (result != 1)
    {
      Serial.print("Error reading register : ");
      Serial.print(result);
      Serial.print(" ");
      terminal.print("Error reading register : ");
      terminal.print(result);
      terminal.print(" ");
      terminal.flush();
    }
    //Serial.print(result);
    //Serial.print(" ");
    Serial.print("r");
    Serial.print(cntr, HEX);
    Serial.print(" ");
    terminal.print("r");
    terminal.print(cntr, HEX);
    terminal.print(" ");
    byte data = Wire.read(); // receive a byte as character
    Serial.println(data, HEX); // print the character
    terminal.println(data, HEX); // print the character
    terminal.flush();
  }
  //  terminal.flush();
}

void esp_ota()
{
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword((const char *)"123");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void Blynk_loop()
{
  while (1) Blynk_Delay(10);
}

void Blynk_Delay(int milli)
{
  int end_time = millis() + milli;
  while (millis() < end_time)
  {
    if (Blynk.connected())
    {
      Blynk.run();
    }
    yield();
  }
}

/*
    LCD
*/

unsigned long mytestText() {
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 20);
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.println("12345678901234567890123456");
  tft.setCursor(0, 40);
  tft.println("1");
  tft.println("2");
  tft.println("3");
  tft.println("4");
  tft.println("5");
  tft.println("6");
  tft.println("7");
  tft.println("8");
  tft.println("9");
  tft.println("0");
  tft.println("1");
  tft.println("2");
}

void test_lcd()
{
  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);

  Serial.println(F("Benchmark                Time (microseconds)"));
  delay(10);
  Serial.print(F("Screen fill              "));
  Serial.println(testFillScreen());
  delay(500);

  Serial.print(F("Text                     "));
  Serial.println(testText());
  delay(3000);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(ILI9341_CYAN));
  delay(500);

  Serial.print(F("Horiz/Vert Lines         "));
  Serial.println(testFastLines(ILI9341_RED, ILI9341_BLUE));
  delay(500);

  Serial.print(F("Rectangles (outline)     "));
  Serial.println(testRects(ILI9341_GREEN));
  delay(500);

  Serial.print(F("Rectangles (filled)      "));
  Serial.println(testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
  delay(500);

  Serial.print(F("Circles (filled)         "));
  Serial.println(testFilledCircles(10, ILI9341_MAGENTA));

  Serial.print(F("Circles (outline)        "));
  Serial.println(testCircles(10, ILI9341_WHITE));
  delay(500);

  Serial.print(F("Triangles (outline)      "));
  Serial.println(testTriangles());
  delay(500);

  Serial.print(F("Triangles (filled)       "));
  Serial.println(testFilledTriangles());
  delay(500);

  Serial.print(F("Rounded rects (outline)  "));
  Serial.println(testRoundRects());
  delay(500);

  Serial.print(F("Rounded rects (filled)   "));
  Serial.println(testFilledRoundRects());
  delay(500);

  Serial.println(F("Done!"));
}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  tft.fillScreen(ILI9341_RED);
  yield();
  tft.fillScreen(ILI9341_GREEN);
  yield();
  tft.fillScreen(ILI9341_BLUE);
  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = 0;
  y2    = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = 0;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  t    += micros() - start;

  yield();
  tft.fillScreen(ILI9341_BLACK);
  yield();

  x1    = w - 1;
  y1    = h - 1;
  y2    = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = 0;
  for (y2 = 0; y2 < h; y2 += 6) tft.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (y = 0; y < h; y += 5) tft.drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(ILI9341_BLACK);
  n     = _min(tft.width(), tft.height());
  start = micros();
  for (i = 2; i < n; i += 6) {
    i2 = i / 2;
    tft.drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n = _min(tft.width(), tft.height());
  for (i = n; i > 0; i -= 6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx - i2, cy - i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx - i2, cy - i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (x = radius; x < w; x += r2) {
    for (y = radius; y < h; y += r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                      w = tft.width()  + radius,
                      h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for (x = 0; x < w; x += r2) {
    for (y = 0; y < h; y += r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  n     = _min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (i = _min(cx, cy); i > 10; i -= 5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(0, i * 10, i * 10));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     tft.color565(i * 10, i * 10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  w     = _min(tft.width(), tft.height());
  start = micros();
  for (i = 0; i < w; i += 6) {
    i2 = i / 2;
    tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(ILI9341_BLACK);
  start = micros();
  for (i = _min(tft.width(), tft.height()); i > 20; i -= 6) {
    i2 = i / 2;
    tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}

void Serial_display_Vout_Iout_Pout()
{
  unsigned long start = micros();

  float Vout1 = Vo_LSB_const * read_reg_12bit(vs1);
  float Vout2 = Vo_LSB_const * read_reg_12bit(vs2);
  float Vout3 = Vo_LSB_const * read_reg_12bit(vs3);
  float Iout = read_reg_12bit(cs2) * CS2_adc_range_const;

  unsigned long ends = micros();

  Serial.print("VS1 : "); Serial.println(Vout1);
  Serial.print("VS2 : "); Serial.println(Vout2);
  Serial.print("VS3 : "); Serial.println(Vout3);
  Serial.print("Iout : "); Serial.println(Iout);
  Serial.print("Pout : "); Serial.println(read_reg_16bit(power));
  Serial.print("start : "); Serial.println(start);
  Serial.print("ends : "); Serial.println(ends);
  Serial.print("ends - start : "); Serial.println(ends - start);

  /*
    VS1 : 103.97
    VS2 : 56.58
    VS3 : 0.00
    Iout : 0.00
    Pout : 0
    start : 35401724
    ends : 35412094
    ends - start : 10370
  */
}

unsigned long start_scope, end_scope;
int scopetimer = 0, scope_eraser = 20, texttimer_eraser_data, texttimer_cursor, texttimer_eraser_cursor;
float Vout1_array_graph[320];
float Vout2_array_graph[320];
float Vout3_array_graph[320];
float Iout_array_graph[320];
float Vout1_array_data[320];
float Vout2_array_data[320];
float Vout3_array_data[320];
float Iout_array_data[320];

void timer_displayScope1046a()
{
  start_scope = end_scope;
  end_scope = micros();

  if (scopetimer >= 320)
  {
    scopetimer = 0;
  }

  if (scope_eraser >= 320)
  {
    scope_eraser = 0;
  }

  float Vout1 = Vo_LSB_const * read_reg_12bit(vs1);
  float Vout2 = Vo_LSB_const * read_reg_12bit(vs2);
  float Vout3 = Vo_LSB_const * read_reg_12bit(vs3);
  float Iout = read_reg_12bit(cs2) * CS2_adc_range_const;

  Vout1_array_data[scopetimer] = Vout1;
  Vout2_array_data[scopetimer] = Vout2;
  Vout3_array_data[scopetimer] = Vout3;
  Iout_array_data[scopetimer] = Iout;

  Vout1_array_graph[scopetimer] = map(Vout1, 0, Vout_setting, 10, 240);
  Vout2_array_graph[scopetimer] = map(Vout2, 0, Vout_setting, 10, 240);
  Iout_array_graph[scopetimer] = map(Iout, 0, Iout_setting, 10, 240);

  texttimer_cursor = scopetimer;
  if (texttimer_cursor >= 280)
  {
    texttimer_cursor = 280;
    //texttimer_eraser_cursor = 280;
  }

  tft.setRotation(3);

  tft.setTextColor(BLACK);
  tft.setCursor(texttimer_eraser_cursor, 225 - Vout1_array_graph[texttimer_eraser_data]);
  tft.print(Vout1_array_data[texttimer_eraser_data]);

  //  tft.setTextColor(BLACK);
  //  tft.setCursor(texttimer_eraser_cursor, 225 - Vout2_array_graph[texttimer_eraser_data]);
  //  tft.print(Vout2_array_data[texttimer_eraser_data]);

  tft.setTextColor(BLACK);
  tft.setCursor(texttimer_eraser_cursor, 225 - Iout_array_graph[texttimer_eraser_data]);
  tft.print(Iout_array_data[texttimer_eraser_data]);

  tft.setTextColor(GREEN);
  tft.setCursor(texttimer_cursor, 225 - Vout1_array_graph[scopetimer]);
  tft.print(Vout1_array_data[scopetimer]);

  //  tft.setTextColor(RED);
  //  tft.setCursor(texttimer_cursor, 225 - Vout2_array_graph[scopetimer]);
  //  tft.print(Vout2_array_data[scopetimer]);

  tft.setTextColor(YELLOW);
  tft.setCursor(texttimer_cursor, 225 - Iout_array_graph[scopetimer]);
  tft.print(Iout_array_data[scopetimer]);

  tft.setRotation(2);

  tft.drawPixel(Vout1_array_graph[scopetimer], scopetimer, GREEN);
  //  tft.drawPixel(Vout2_array_graph[scopetimer], scopetimer, RED);
  tft.drawPixel(Iout_array_graph[scopetimer], scopetimer, YELLOW);

  tft.drawPixel(Vout1_array_graph[scope_eraser], scope_eraser, BLACK);
  //  tft.drawPixel(Vout2_array_graph[scope_eraser], scope_eraser, BLACK);
  tft.drawPixel(Iout_array_graph[scope_eraser], scope_eraser, BLACK);

  texttimer_eraser_cursor = texttimer_cursor;
  texttimer_eraser_data = scopetimer;
  scope_eraser++;
  scopetimer++;

  Serial.println(end_scope - start_scope);
}

int vin_maxrange, iin_maxrange, vout_maxrange, pin_maxrange;
float Vin_array_data[320];
float Iin_array_data[320];
float Vout_array_data[320];
float Pin_array_data[320];
float Vin_array_graph[320];
float Iin_array_graph[320];
float Vout_array_graph[320];
float Pin_array_graph[320];

void timer_displayScope1048()
{
  start_scope = end_scope;
  end_scope = micros();

  if (scopetimer >= 320)
  {
    scopetimer = 0;
  }

  if (scope_eraser >= 320)
  {
    scope_eraser = 0;
  }

  unsigned int read_vin = read_reg_16bit(0x88);
  unsigned int read_iin = read_reg_16bit(0x89);
  unsigned int read_vout = read_reg_16bit(0x8b);
  unsigned int read_pin = read_reg_16bit(0x97);

  unsigned int mantissa_vin = read_vin & 0x7ff;
  unsigned int mantissa_iin = read_iin & 0x7ff;
  unsigned int mantissa_vout = read_vout & 0x7ff;
  unsigned int mantissa_pin = read_pin & 0x7ff;

  int8_t exp_vin  = (read_vin >> 11) | 0xE0;
  int8_t exp_iin  = (read_iin >> 11) | 0xE0;
  int8_t exp_vout = (read_vout >> 11) | 0xE0;
  int8_t exp_pin  = (read_pin >> 11) | 0xE0;

  float f_read_vin = mantissa_vin * pow(2, exp_vin);
  float f_read_iin = mantissa_iin * pow(2, exp_iin);
  float f_read_vout = mantissa_vout * pow(2, exp_vout);
  float f_read_pin = mantissa_pin * pow(2, exp_pin);

  Vin_array_data[scopetimer] = f_read_vin;
  Iin_array_data[scopetimer] = f_read_iin;
  Vout_array_data[scopetimer] = f_read_vout;
  Pin_array_data[scopetimer] = f_read_pin;

  Vin_array_graph[scopetimer] = map(f_read_vin, 0, vin_maxrange, 10, 240);
  Iin_array_graph[scopetimer] = map(f_read_iin, 0, iin_maxrange, 10, 240);
  Vout_array_graph[scopetimer] = map(f_read_vout, 0, vout_maxrange, 10, 240);
  Pin_array_graph[scopetimer] = map(f_read_pin, 0, pin_maxrange, 10, 240);

  texttimer_cursor = scopetimer;
  if (texttimer_cursor >= 280)
  {
    texttimer_cursor = 280;
    //texttimer_eraser_cursor = 280;
  }

  tft.setRotation(3);

  tft.setTextColor(BLACK);
  tft.setCursor(texttimer_eraser_cursor, 225 - Vin_array_graph[texttimer_eraser_data]);
  tft.print(Vin_array_data[texttimer_eraser_data]);

  tft.setTextColor(BLACK);
  tft.setCursor(texttimer_eraser_cursor, 225 - Iin_array_graph[texttimer_eraser_data]);
  tft.print(Iin_array_data[texttimer_eraser_data]);

  tft.setTextColor(BLACK);
  tft.setCursor(texttimer_eraser_cursor, 225 - Vout_array_graph[texttimer_eraser_data]);
  tft.print(Vout_array_data[texttimer_eraser_data]);

  tft.setTextColor(BLACK);
  tft.setCursor(texttimer_eraser_cursor, 225 - Pin_array_graph[texttimer_eraser_data]);
  tft.print(Pin_array_data[texttimer_eraser_data]);

  tft.setTextColor(GREEN);
  tft.setCursor(texttimer_cursor, 225 - Vin_array_graph[scopetimer]);
  tft.print(Vin_array_data[scopetimer]);

  tft.setTextColor(RED);
  tft.setCursor(texttimer_cursor, 225 - Iin_array_graph[scopetimer]);
  tft.print(Iin_array_data[scopetimer]);

  tft.setTextColor(YELLOW);
  tft.setCursor(texttimer_cursor, 225 - Vout_array_graph[scopetimer]);
  tft.print(Vout_array_data[scopetimer]);

  tft.setTextColor(WHITE);
  tft.setCursor(texttimer_cursor, 225 - Pin_array_graph[scopetimer]);
  tft.print(Pin_array_data[scopetimer]);

  tft.setRotation(2);

  tft.drawPixel(Vin_array_graph[scopetimer], scopetimer, GREEN);
  tft.drawPixel(Iin_array_graph[scopetimer], scopetimer, RED);
  tft.drawPixel(Vout_array_graph[scopetimer], scopetimer, YELLOW);
  tft.drawPixel(Pin_array_graph[scopetimer], scopetimer, WHITE);

  tft.drawPixel(Vin_array_graph[scope_eraser], scope_eraser, BLACK);
  tft.drawPixel(Iin_array_graph[scope_eraser], scope_eraser, BLACK);
  tft.drawPixel(Vout_array_graph[scope_eraser], scope_eraser, BLACK);
  tft.drawPixel(Pin_array_graph[scope_eraser], scope_eraser, BLACK);

  texttimer_eraser_cursor = texttimer_cursor;
  texttimer_eraser_data = scopetimer;
  scope_eraser++;
  scopetimer++;

  Serial.println(end_scope - start_scope);
}

void init_adp1048()
{
  // i2c_address = 0x58;
  //  display_regs(0x98, 0x9b);
  //  display_reg_16bit(0x88);
  //  display_reg_16bit(0x89);
  //  display_reg_16bit(0x8b);
  //  display_reg_16bit(0x97);
  //  display_reg_ext_16bit(0xfe39);
  //  display_regs(0x20, 0x20);
  //  display_adpeeprom_1048((page0_1048 + 2), 0, (22 * 3));
  //  display_all_board_reg_1048();
  timerBlink_id = timerBlink.setInterval(1000L, timer_blinkLedWidget);
  terminal.println("================================");
  //terminal.println(F("v" BLYNK_VERSION ": Device started"));

  //  eeprom_unlock_1048();

  write_byte(0x02, 0x0b);
  //display_regs(0x02, 0x02);

  Blynk_Delay(3000);
  unsigned int read_vin = read_reg_16bit(0x88);
  unsigned int read_iin = read_reg_16bit(0x89);
  unsigned int read_vout = read_reg_16bit(0x8b);
  unsigned int read_pin = read_reg_16bit(0x97);

  //  terminal.print("read_vin,0x88 : "); terminal.print("0x"); terminal.println(read_vin, HEX);
  //  terminal.print("read_iin,0x89 : "); terminal.print("0x"); terminal.println(read_iin, HEX);
  //  terminal.print("read_vout,0x8b : "); terminal.print("0x"); terminal.println(read_vout, HEX);
  //  terminal.print("read_pin,0x97 : "); terminal.print("0x"); terminal.println(read_pin, HEX);

  unsigned int mantissa_vin = read_vin & 0x7ff;
  unsigned int mantissa_iin = read_iin & 0x7ff;
  unsigned int mantissa_vout = read_vout & 0x7ff;
  unsigned int mantissa_pin = read_pin & 0x7ff;

  //  terminal.print("mantissa_vin : "); terminal.println(mantissa_vin, HEX);
  //  terminal.print("mantissa_iin : "); terminal.println(mantissa_iin, HEX);
  //  terminal.print("mantissa_vout : "); terminal.println(mantissa_vout, HEX);
  //  terminal.print("mantissa_pin : "); terminal.println(mantissa_pin, HEX);

  int8_t exp_vin  = (read_vin >> 11) | 0xE0;
  int8_t exp_iin  = (read_iin >> 11) | 0xE0;
  int8_t exp_vout = (read_vout >> 11) | 0xE0;
  int8_t exp_pin  = (read_pin >> 11) | 0xE0;

  //int number = mantissa * pow(2, exp_vin);

  float f_read_vin = mantissa_vin * pow(2, exp_vin);
  float f_read_iin = mantissa_iin * pow(2, exp_iin);
  float f_read_vout = mantissa_vout * pow(2, exp_vout);
  float f_read_pin = mantissa_pin * pow(2, exp_pin);

  vin_maxrange = 0x7ff * pow(2, exp_vin);
  iin_maxrange = 0x7ff * pow(2, exp_iin);
  vout_maxrange = 0x7ff * pow(2, exp_vout);
  pin_maxrange = 0x7ff * pow(2, exp_pin);

  //  Serial.print("read_vin : "); Serial.println(f_read_vin, 3);
  //  Serial.print("read_iin : "); Serial.println(f_read_iin, 3);
  //  Serial.print("read_vout : "); Serial.println(f_read_vout, 3);
  //  Serial.print("read_pin : "); Serial.println(f_read_pin, 3);
  //
  //  Serial.print("exp_vin : "); Serial.println(exp_vin);
  //  Serial.print("exp_iin : "); Serial.println(exp_iin);
  //  Serial.print("exp_vout : "); Serial.println(exp_vout);
  //  Serial.print("exp_pin : "); Serial.println(exp_pin);
  //
  //  Serial.print("vin_maxrange : "); Serial.println(vin_maxrange);
  //  Serial.print("iin_maxrange : "); Serial.println(iin_maxrange);
  //  Serial.print("vout_maxrange : "); Serial.println(vout_maxrange);
  //  Serial.print("pin_maxrange : "); Serial.println(pin_maxrange);
  //
  //  terminal.print("read_vin : "); terminal.println(f_read_vin, 3);
  //  terminal.print("read_iin : "); terminal.println(f_read_iin, 3);
  //  terminal.print("read_vout : "); terminal.println(f_read_vout, 3);
  //  terminal.print("read_pin : "); terminal.println(f_read_pin, 3);
  //
  //  terminal.print("exp_vin : "); terminal.println(exp_vin);
  //  terminal.print("exp_iin : "); terminal.println(exp_iin);
  //  terminal.print("exp_vout : "); terminal.println(exp_vout);
  //  terminal.print("exp_pin : "); terminal.println(exp_pin);
  //
  //  terminal.print("vin_maxrange : "); terminal.println(vin_maxrange);
  //  terminal.print("iin_maxrange : "); terminal.println(iin_maxrange);
  //  terminal.print("vout_maxrange : "); terminal.println(vout_maxrange);
  //  terminal.print("pin_maxrange : "); terminal.println(pin_maxrange);

  int kvin = read_reg_ext_16bit(0xfe3b);
  int mantissa_kvin = kvin & 0x3ff;
  int8_t exp_kvin = (kvin >> 11) | 0xE0;
  float f_kvin = mantissa_kvin * pow(2, exp_kvin);

  //display_reg_ext_16bit(0xfe3b);
  //  terminal.print("kvin,0xfe3b : "); terminal.println(kvin, HEX);
  //  terminal.print("mantissa_kvin : "); terminal.println(mantissa_kvin);
  //  terminal.print("exp_kvin : "); terminal.println(exp_kvin);
  //  terminal.print("kvin : "); terminal.println(f_kvin, 3);

  int iin_gsense = read_reg_ext_16bit(0xfe3c);
  int mantissa_iin_gsense = iin_gsense & 0x3ff;
  int8_t exp_iin_gsense = (iin_gsense >> 11) | 0xE0;
  float f_iin_gsense = mantissa_iin_gsense * pow(2, exp_iin_gsense);

  //  terminal.print("iin_gsense,0xfe3c : "); terminal.println(iin_gsense, HEX);
  //  terminal.print("mantissa_iin_gsense : "); terminal.println(mantissa_iin_gsense);
  //  terminal.print("exp_iin_gsense : "); terminal.println(exp_iin_gsense);
  //  terminal.print("iin_gsense : "); terminal.println(f_iin_gsense, 3);

  //display_reg_16bit(0x2a);
  int vout_scale = read_reg_16bit(0x2a);
  int mantissa_vout_scale = vout_scale & 0x3ff;
  int8_t exp_vout_scale = (vout_scale >> 11) | 0xE0;
  float f_vout_scale = mantissa_vout_scale * pow(2, exp_vout_scale);

  //  terminal.print("vout_scale,0x2a : "); terminal.println(vout_scale, HEX);
  //  terminal.print("mantissa_vout_scale : "); terminal.println(mantissa_vout_scale);
  //  terminal.print("exp_vout_scale : "); terminal.println(exp_vout_scale);
  //  terminal.print("vout_scale : "); terminal.println(f_vout_scale, 3);

  terminal.flush();

}
