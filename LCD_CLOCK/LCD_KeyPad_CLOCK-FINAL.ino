//===[ LIBRARIES BGN ]===============================================================================
#include <Wire.h>;                  // I2C
#include <LiquidCrystal.h>          // LDC display
#include "RTClib.h";                // Tiny Real Time Clock
#include <SFE_BMP180.h>;            // Barometer
#include <dht11.h>;                 // Higrometer
#include <OneWire.h>                // OneWire for DS18B20
#include <DallasTemperature.h>      // DS18B20
//===[ LIBRARIES END ]===============================================================================

//===[ OBJECTS BGN ]=================================================================================
//---{ LCD BGN }-------------------------------------------------------------------------------------
// deine lcd pins here E(enable): D9, RS(data&signal): D8, RW(read&write): GND, DB7-DB4(data):D7-D4
#define LCD_E   9  // enable pin
#define LCD_RS  8  // data or signal display
#define LCD_DB7 7  // data 7
#define LCD_DB6 6  // data 6
#define LCD_DB5 5  // data 5
#define LCD_DB4 4  // data 4
#define LCD_BL 10  // back light pin
//#define LCD_RW     // read(@high) or write(@low), it is permanently on GND, i.e. write
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_DB4, LCD_DB5, LCD_DB6, LCD_DB7);	// LCD pins
//---{ LCD END }-------------------------------------------------------------------------------------
//---{ RTC BGN }-------------------------------------------------------------------------------------
RTC_DS1307 rtc;          // RTC setup
//---{ RTC END }-------------------------------------------------------------------------------------
//---{ BMP180 BGN }----------------------------------------------------------------------------------
SFE_BMP180 pressure;     // pressure sensor setup
//---{ BMP180 END }----------------------------------------------------------------------------------
//---{ DHT11 BGN }-----------------------------------------------------------------------------------
dht11 DHT11;             // humidity sensor setup
//---{ DHT11 END }-----------------------------------------------------------------------------------
//---{ DS18B20 BGN }---------------------------------------------------------------------------------
#define ONE_WIRE_BUS       1            // Temperature sensor pin
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensorDS18B20(&oneWire);
//---{ DS18B20 END }---------------------------------------------------------------------------------
//===[ OBJECTS END ]=================================================================================

//===[ GLOBAL VALUES BGN ]===========================================================================
// define some values used by the panel and buttons
#define btnRIGHT  0	// right button
#define btnUP     1	// up button
#define btnDOWN   2	// down button
#define btnLEFT   3	// left button
#define btnSELECT 4	// select button
#define btnNONE   5	// no button pressed

// define pins
#define buttonPIN          A0        // Button pin
#define pinDHT11           2	       // Humidity sensor pin

// declare backlight variables
byte backlightIntensity      = 100;
byte backlightIntensityDay   = 100;
byte backlightIntensityNight =  25;
byte backlightOffOnChanged   =   1;
byte backlightPIRState       =   1;
byte nightStartHour          =  17;
byte nightStartMinute        =  30;
byte nightEndHour            =   6;
byte nightEndMinute          =  30;

// define delays
#define debounceDELAY        50
#define	buttonDELAY        1000
#define displayDELAY        100

// sensor variables
int altitudeBMP             = 125;
int pressureOffset          =   0;
int humidityOffset          =   0;
#define NoOfTEMPSENSORS         3
int temperatureOffset[NoOfTEMPSENSORS+1];
String tempSensorName[]     = {"AllTemp", " DHT11", "BMP180", "DS18B2"};
byte tempSensor             =   1;	// defines which sensor's temperature read is displayed.
byte prevSensorReadSec;

// button operation variables
int buttonState;
int buttonStateCurr;
int buttonStatePrev;
int buttonMode = 1;
unsigned long buttonTime = 0;
unsigned long buttonModeTime = 0;
unsigned long debounceTime = 0;
unsigned long displayTime = 0;
unsigned long currTime = 0;

// menu operation variables
int currMenu = 0;

//===[ GLOBAL VALUES END ]===========================================================================

//===[ SETUP BGN ]===================================================================================
void setup() {
  pinMode(LCD_BL, OUTPUT);
  analogWrite(LCD_BL, backlightIntensity);
  backlightOffOnChanged = 2;
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print("BOOTING...");
  lcd.setCursor(0,1);
  lcd.print("RTC...");
  //setup RTC  
  rtc.begin();
  if (! rtc.isrunning()) {
    lcd.print("SETTING");
    rtc.adjust(DateTime(__DATE__, __TIME__)); //sets the RTC to the date & time this sketch was compiled
  } 
  else {
    lcd.print("OK");
    //rtc.adjust(DateTime(__DATE__, __TIME__)); // uncomment to force the clock being set to system time
  }
  delay(1500);
  //setup BTR180
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  lcd.print("BTR180...");
  if (pressure.begin()) {
    lcd.print("OK");
  } 
  else {
    lcd.print("ERROR"); 
  }
  delay(1500);
  lcd.setCursor(0,1);
  lcd.print("                ");
  lcd.setCursor(0,1);
  //setup DS18B20
  sensorDS18B20.begin();
  lcd.clear();
}
//===[ SETUP END ]===================================================================================

//===[ LOOP BGN ]====================================================================================
void loop() {

  buttonStateCurr = read_button_state();

  if ( buttonStateCurr != buttonStatePrev ) {
    debounceTime = millis();
    buttonMode = 1;
    buttonTime = 0;
    buttonModeTime = 0;
  }
  if ( (buttonStateCurr == buttonStatePrev) && ((millis() - debounceTime) > debounceDELAY) ) {

    if ( (buttonMode == 1) && (buttonTime == 0) ) {

      buttonTime = millis();
      buttonState = buttonStateCurr;
    } 
    else if ( (buttonMode >= 1) && ((millis() - buttonTime) > (buttonDELAY / buttonMode)) ) {
      buttonTime = millis();
      buttonState = buttonStateCurr;
      if ( (millis() - buttonModeTime) > (buttonDELAY * buttonMode) ) {
        buttonMode = min(5, buttonMode++);
        buttonModeTime = buttonTime; //millis();
      }

    } 
    else buttonState = btnNONE;		

  } 
  else buttonState = btnNONE;

  buttonStatePrev = buttonStateCurr;

  call_operation();

  // set backlight   --- Not efficient, has to be redone!!!
  if ( (nightStartHour + nightStartMinute/60) > (nightEndHour + nightEndMinute/60)  ) {
    if ( ( (rtc.now().hour() + rtc.now().minute()/60) > (nightEndHour + nightEndMinute/60) ) && ( (rtc.now().hour() + rtc.now().minute()/60) < (nightStartHour + nightStartMinute/60) ) ) {
      backlightIntensity = backlightIntensityDay;
      backlightOffOnChanged = 2;

    } 
    else {
      backlightIntensity = backlightIntensityNight;
      backlightOffOnChanged = 2;
    }
  } 
  else {
    if ( ( (rtc.now().hour() + rtc.now().minute()/60) < (nightEndHour + nightEndMinute/60) ) && ( (rtc.now().hour() + rtc.now().minute()/60) > (nightStartHour + nightStartMinute/60) ) ) {
      backlightIntensity = backlightIntensityNight;     
      backlightOffOnChanged = 2;           
    } 
    else {
      backlightIntensity = backlightIntensityDay;        
      backlightOffOnChanged = 2;
    }
  }

  if ( backlightOffOnChanged > 1 ) {
    if ( backlightOffOnChanged = 2 ) backlightOffOnChanged = 1;
    //if ( backlightOffOnChanged = 3 ) backlightOffOnChanged = 0;
    analogWrite(LCD_BL, (byte) backlightOffOnChanged * backlightIntensity * 255 / 100);
  }
}
//===[ LOOP END ]====================================================================================

//===[ GENERAL FUNCTIONS BGN ]=======================================================================

int read_button_state() {
 // read the value from the sensor my buttons when read are centered at these valies: 0, 144, 329, 504, 741
 // we add approx 50 to those values and check to see if we are close
 int adc_key_in = analogRead(buttonPIN); 
 if (adc_key_in > 1000) return btnNONE;    // We make this the 1st option for speed reasons since it will be the most likely result
 if (adc_key_in <   50) return btnRIGHT;  
 if (adc_key_in <  195) return btnUP; 
 if (adc_key_in <  380) return btnDOWN; 
 if (adc_key_in <  555) return btnLEFT; 
 if (adc_key_in <  790) return btnSELECT;   
 else btnNONE;                                // when all others fail, return this...
 }
 

void repeateChar(char character, byte repeate) {
  for (byte r=0; r < repeate; r++) lcd.print(character);
}

void outputMenu(String menuText, boolean del2ndRow) {
  lcd.setCursor(0,0);
  if (currMenu < 100) lcd.print(" ");
  if (currMenu < 10) lcd.print(" ");
  lcd.print(currMenu);
  lcd.print(menuText);
  if ( del2ndRow ) {
    lcd.setCursor(0,1);
    lcd.print("                ");
  }
}

void outputDateTime(byte row, DateTime now) {
  //OUTPUT DATE&TIME  
  int num; 
  lcd.setCursor(0,row);
  num = now.year();
  if (num < 10) lcd.print('0');
  lcd.print(num, DEC);
  lcd.print('-');
  num = now.month();
  if (num < 10) lcd.print('0'); 
  lcd.print(num, DEC);
  lcd.print('-');
  num = now.day();
  if (num < 10) lcd.print('0'); 
  lcd.print(num, DEC);
  lcd.print(' ');
  num = now.hour(); 
  if (num < 10) lcd.print('0'); 
  lcd.print(num, DEC);
  num = now.second();
  if (num % 2 == 0) {
    lcd.print(':');
  } 
  else {
    lcd.print(' ');
  }
  num = now.minute();
  if (num < 10) lcd.print('0'); 
  lcd.print(now.minute(), DEC);
}

float getDHT(byte sensor) {
  int chk = DHT11.read(pinDHT11);
  //delay(200);
  switch (chk) {
  case 0:
    switch (sensor) {
    case 1:
      return DHT11.humidity;
      break;
    case 2:
      return DHT11.temperature;
      break;
    }
    break;
  case -1: 
  case -2: 
  default: 
    return -100; 
    break;
  }
}

float getDS18B20() {
  sensorDS18B20.requestTemperatures();
  return sensorDS18B20.getTempCByIndex(0);
}

void outputHumidity(byte col, byte row) {
  lcd.setCursor(col, row);
  double hum = getDHT(1);
  if (hum <= 0) {
    lcd.print("HmEr");
  } 
  else {
    hum += humidityOffset;
    //if (abs(hum) < 100) lcd.print(" ");
    if (abs(hum) <  10) lcd.print(" ");
    if (hum >= 0) lcd.print(" ");
    lcd.print(hum, 0);
    lcd.print("%");
  }
}

void outputTemperature(byte col, byte row) {
  float T;
  switch (tempSensor) {
  case 0: 
    T = getDHT(2); 
    break;
  case 1: 
    T = getBMP(2); 
    break;
  case 2: 
    T = getDS18B20(); 
    break;
  }
  lcd.setCursor(col, row);
  if (T > -100) {
    T += temperatureOffset[tempSensor];
    if (abs(T) < 100) lcd.print(" ");
    if (abs(T) <  10) lcd.print(" ");
    if (T >= 0) lcd.print(" ");
    lcd.print(T,0);
    lcd.print((char) 223);
    lcd.print("C");
  } 
  else lcd.print(" TmEr ");
}

double getBMP(byte pres) {
  char status;
  double T,P,p0;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);

    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0) {
          p0 = pressure.sealevel(P,altitudeBMP);
          if (pres == 0){
            return p0;
          } 
          else if (pres == 1) {
            return P;
          }
          else {
            return T;
          }
        } 
        else return -100;
      } 
      else return -100;
    } 
    else return -100;
  }
}

void outputPressure(byte col, byte row) {
  double p0 = getBMP(0);
  lcd.setCursor(col, row);
  if (p0 == 0) {
    lcd.print("PrEr");
  }
  else {
    p0 += pressureOffset;
    if (p0 < 1000) lcd.print(" ");
    lcd.print(p0,0);
    lcd.print("mb");
  }
}

void outputAllTemp(int temp) {
  //if (abs(temp) < 100) lcd.print(" ");
  if (abs(temp) <  10) lcd.print(" ");
  if (temp >= 0) lcd.print(" ");
  lcd.print(temp);
  lcd.print((char) 223);
  lcd.print("C");
}

void call_operation() {
  switch ( currMenu ) {
  case 0: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          byte currSecond = rtc.now().second();
          if ( prevSensorReadSec != currSecond ) {
            prevSensorReadSec = currSecond;
            outputDateTime(0, rtc.now());    
            if ( tempSensor == 0) {
              lcd.setCursor(0,1);
              outputAllTemp(getDHT(2) + temperatureOffset[1]);
              outputAllTemp(getBMP(2) + temperatureOffset[2]);
              outputAllTemp(getDS18B20() + temperatureOffset[3]);
            } else {
              switch ( prevSensorReadSec % 3) {
                case 0:
                  outputPressure(0, 1);
                  break;
                case 1:
                  outputHumidity(6, 1);
                  break;
                case 2:
                  outputTemperature(10, 1);
                  break;
              }
            }
          }
          break;
        }
      case btnUP: 
        {
          backlightOffOnChanged = 1;
          break;
        }
      case btnDOWN: 
        {
          backlightOffOnChanged = 0;
          break;
        }
      case btnLEFT: 
        {
          break;
        }
      case btnRIGHT: 
        {
          break;
        }
      case btnSELECT: 
        {
          currMenu = 100;
          break;
        }
      }
      break;
    }
  case 100: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DISPLAY     ", true);
          break;
        }
      case btnUP: 
        {
          currMenu = 300;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 200;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 0;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 110;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 110;
          break;
        }
      }
      break;
    }
  case 110: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DAY BCKLIGHT", false);
          lcd.setCursor(0,1);
          lcd.print("INTENSITY:  ");
          if (backlightIntensityDay < 100) lcd.print(" ");
          if (backlightIntensityDay <  10) lcd.print(" ");
          lcd.print(backlightIntensityDay);
          lcd.print("% ");
          break;
        }
      case btnUP: 
        {
          currMenu = 140;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 120;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 100;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 111;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 111;
          break;
        }
      }
      break;
    }
  case 111: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DAY BCKLIGHT", false);
          lcd.setCursor(0,1);
          if (backlightIntensityDay < 100) lcd.print(" ");
          if (backlightIntensityDay <  10) lcd.print(" ");
          lcd.print(backlightIntensityDay);
          lcd.print("%[");
          byte decil = backlightIntensityDay / 10;
          repeateChar((char)255, decil);
          repeateChar((char)32, 10-decil);
          lcd.print("]");
          break;
        }
      case btnUP: 
        {
          backlightIntensityDay = min(100, backlightIntensityDay + 10);
          backlightOffOnChanged = 2;
          break;
        }
      case btnRIGHT: 
        {
          backlightIntensityDay = min(100, backlightIntensityDay + 1);
          backlightOffOnChanged = 2;
          break;
        }
      case btnDOWN: 
        {
          backlightIntensityDay = max(0, backlightIntensityDay - 10);
          backlightOffOnChanged = 2;                    
          break;
        }
      case btnLEFT: 
        {
          backlightIntensityDay = max(0, backlightIntensityDay - 1);
          backlightOffOnChanged = 2;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 110;
          break;
        }
      }
      break;
    }
  case 120: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" NIGHT BCKLHT", false);
          lcd.setCursor(0,1);
          lcd.print("INTENSITY:  ");
          if (backlightIntensityNight < 100) lcd.print(" ");
          if (backlightIntensityNight <  10) lcd.print(" ");
          lcd.print(backlightIntensityNight);
          lcd.print("% ");
          break;
        }
      case btnUP: 
        {
          currMenu = 110;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 130;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 100;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 121;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 121;
          break;
        }
      }
      break;
    }
  case 121: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" NIGHT BCKLHT", false);
          lcd.setCursor(0,1);
          if (backlightIntensityNight < 100) lcd.print(" ");
          if (backlightIntensityNight <  10) lcd.print(" ");
          lcd.print(backlightIntensityNight);
          lcd.print("%[");
          byte decil = backlightIntensityNight / 10;
          repeateChar((char)255, decil);
          repeateChar((char)32, 10-decil);
          lcd.print("]");
          break;
        }
      case btnUP: 
        {
          backlightIntensityNight = min(100, backlightIntensityNight + 10);
          backlightOffOnChanged = 2;
          break;
        }
      case btnRIGHT: 
        {
          backlightIntensityNight = min(100, backlightIntensityNight + 1);
          backlightOffOnChanged = 2;
          break;
        }
      case btnDOWN: 
        {
          backlightIntensityNight = max(0, backlightIntensityNight - 10);
          backlightOffOnChanged = 2;                    
          break;
        }
      case btnLEFT: 
        {
          backlightIntensityNight = max(0, backlightIntensityNight - 1);
          backlightOffOnChanged = 2;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 120;
          break;
        }
      }
      break;
    }
  case 130: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" NIGHT START ", false);
          lcd.setCursor(0,1);
          lcd.print("TIME:      ");
          if (nightStartHour < 10) lcd.print("0");
          lcd.print(nightStartHour);
          lcd.print(":");
          if (nightStartMinute < 10) lcd.print("0");
          lcd.print(nightStartMinute);
          break;
        }
      case btnUP: 
        {
          currMenu = 120;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 140;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 100;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 131;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 131;
          break;
        }
      }
      break;
    }
  case 131: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" NIGHT START ", false);
          lcd.setCursor(0,1);
          lcd.print("SET TIME:  ");
          if (nightStartHour < 10) lcd.print("0");
          lcd.print(nightStartHour);
          lcd.print(":");
          if (nightStartMinute < 10) lcd.print("0");
          lcd.print(nightStartMinute);
          break;
        }
      case btnUP: 
        {
          nightStartHour = min(23, nightStartHour + 1);
          break;
        }
      case btnRIGHT: 
        {
          nightStartMinute = min(59, nightStartMinute + 1);
          break;
        }
      case btnDOWN: 
        {
          nightStartHour = max(0, nightStartHour - 1);
          break;
        }
      case btnLEFT: 
        {
          nightStartMinute = max(0, nightStartMinute - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 130;
          break;
        }
      }
      break;
    }
  case 140: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" NIGHT END   ", false);
          lcd.setCursor(0,1);
          lcd.print("TIME:      ");
          if (nightEndHour < 10) lcd.print("0");
          lcd.print(nightEndHour);
          lcd.print(":");
          if (nightEndMinute < 10) lcd.print("0");
          lcd.print(nightEndMinute);
          break;
        }
      case btnUP: 
        {
          currMenu = 130;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 110;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 100;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 141;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 141;
          break;
        }
      }
      break;
    }
  case 141: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" NIGHT END   ", false);
          lcd.setCursor(0,1);
          lcd.print("SET TIME:  ");
          if (nightEndHour < 10) lcd.print("0");
          lcd.print(nightEndHour);
          lcd.print(":");
          if (nightEndMinute < 10) lcd.print("0");
          lcd.print(nightEndMinute);
          break;
        }
      case btnUP: 
        {
          nightEndHour = min(23, nightEndHour + 1);
          break;
        }
      case btnRIGHT: 
        {
          nightEndMinute = min(59, nightEndMinute + 1);
          break;
        }
      case btnDOWN: 
        {
          nightEndHour = max(0, nightEndHour - 1);
          break;
        }
      case btnLEFT: 
        {
          nightEndMinute = max(0, nightEndMinute - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 140;
          break;
        }
      }
      break;
    }
  case 200: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", true);
          break;
        }
      case btnUP: 
        {
          currMenu = 100;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 300;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 0;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 210;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 210;
          break;
        }
      }
      break;
    }
  case 210: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);
          lcd.setCursor(0,1);
          lcd.print("HOURS:       ");
          DateTime dt = rtc.now();
          if (dt.hour() < 10) lcd.print("0");
          lcd.print(dt.hour());
          lcd.print("h");
          break;
        }
      case btnUP: 
        {
          currMenu = 260;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 220;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 200;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 211;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 211;
          break;
        }
      }
      break;
    }
  case 211: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);
          lcd.setCursor(0,1);
          lcd.print("SET HOURS:   ");
          DateTime dt = rtc.now();
          if (dt.hour() < 10) lcd.print(" ");
          lcd.print(dt.hour());
          lcd.print("h");
          break;
        }
      case btnUP: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), min(23, dt.hour()+10), dt.minute(), rtc.now().second()));
          break;
        }
      case btnRIGHT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), min(23, dt.hour()+1), dt.minute(), rtc.now().second()));
          break;
        }
      case btnDOWN: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), max(0, dt.hour()-10), dt.minute(), rtc.now().second()));
          break;
        }
      case btnLEFT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), max(0, dt.hour()-1), dt.minute(), rtc.now().second()));
          break;
        }
      case btnSELECT: 
        {
          currMenu = 210;
          break;
        }
      }
      break;
    }
  case 220: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);                  
          lcd.setCursor(0,1);
          lcd.print("MINUTES:     ");
          DateTime dt = rtc.now();
          if (dt.minute() < 10) lcd.print("0");
          lcd.print(dt.minute());
          lcd.print("m");
          break;
        }
      case btnUP: 
        {
          currMenu = 210;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 230;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 200;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 221;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 221;
          break;
        }
      }
      break;
    }
  case 221: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);
          lcd.setCursor(0,1);
          lcd.print("SET MINUTES: ");
          DateTime dt = rtc.now();
          if (dt.minute() < 10) lcd.print("0");
          lcd.print(dt.minute());
          lcd.print("m");
          break;
        }
      case btnUP: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), min(59, dt.minute()+10), rtc.now().second()));
          break;
        }
      case btnRIGHT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), min(59, dt.minute()+1), rtc.now().second()));
          break;
        }
      case btnDOWN: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), max(0, dt.minute()-10), rtc.now().second()));
          break;
        }
      case btnLEFT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), max(0, dt.minute()-1), rtc.now().second()));
          break;
        }
      case btnSELECT: 
        {
          currMenu = 220;
          break;
        }
      }
      break;
    }
  case 230: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);
          lcd.setCursor(0,1);
          lcd.print("SECONDS:     ");
          DateTime dt = rtc.now();
          if (dt.second() < 10) lcd.print("0");
          lcd.print(dt.second());
          lcd.print("s");
          break;
        }
      case btnUP: 
        {
          currMenu = 220;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 240;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 200;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 231;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 231;
          break;
        }
      }
      break;
    }
  case 231: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);
          lcd.setCursor(0,1);
          lcd.print("SET SECONDS: ");
          DateTime dt = rtc.now();
          if (dt.second() < 10) lcd.print("0");
          lcd.print(dt.second());
          lcd.print("s");
          break;
        }
      case btnUP: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), 0));
          break;
        }
      case btnRIGHT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), 0));
          break;
        }
      case btnDOWN: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), 0));
          break;
        }
      case btnLEFT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), 0));
          break;
        }
      case btnSELECT: 
        {
          currMenu = 230;
          break;
        }
      }
      break;
    }
  case 240: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);                  
          lcd.setCursor(0,1);
          lcd.print("YEAR:      ");
          DateTime dt = rtc.now();
          if (dt.year() < 1000) lcd.print(" ");
          if (dt.year() < 100) lcd.print(" ");
          if (dt.year() < 10) lcd.print(" ");
          lcd.print(dt.year());
          lcd.print("y");
          break;
        }
      case btnUP: 
        {
          currMenu = 230;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 250;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 200;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 241;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 241;
          break;
        }
      }
      break;
    }
  case 241: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);                  
          lcd.setCursor(0,1);
          lcd.print("SET YEAR:  ");
          DateTime dt = rtc.now();
          if (dt.year() < 1000) lcd.print(" ");
          if (dt.year() < 100) lcd.print(" ");
          if (dt.year() < 10) lcd.print(" ");
          lcd.print(dt.year());
          lcd.print("y");
          break;
        }
      case btnUP: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year()+10, dt.month(), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnRIGHT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year()+1, dt.month(), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnDOWN: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year()-10, dt.month(), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnLEFT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year()-1, dt.month(), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnSELECT: 
        {
          currMenu = 240;
          break;
        }
      }
      break;
    }
  case 250: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);                  
          lcd.setCursor(0,1);
          lcd.print("MONTH:       ");
          DateTime dt = rtc.now();
          if (dt.month() < 10) lcd.print("0");
          lcd.print(dt.month());
          lcd.print("m");
          break;
        }
      case btnUP: 
        {
          currMenu = 240;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 260;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 200;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 251;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 251;
          break;
        }
      }
      break;
    }
  case 251: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);                  
          lcd.setCursor(0,1);
          lcd.print("SET MONTH:   ");
          DateTime dt = rtc.now();
          if (dt.month() < 10) lcd.print("0");
          lcd.print(dt.month());
          lcd.print("m");
          break;
        }
      case btnUP: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), min(12, dt.month()+3), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnRIGHT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), min(12, dt.month()+1), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnDOWN: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), max(1, dt.month()-3), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnLEFT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), max(1, dt.month()-1), dt.day(), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnSELECT: 
        {
          currMenu = 250;
          break;
        }
      }
      break;
    }
  case 260: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);                  
          lcd.setCursor(0,1);
          lcd.print("DAY:         ");
          DateTime dt = rtc.now();
          if (dt.day() < 10) lcd.print("0");
          lcd.print(dt.day());
          lcd.print("d");
          break;
        }
      case btnUP: 
        {
          currMenu = 250;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 270;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 200;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 261;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 261;
          break;
        }
      }
      break;
    }
  case 261: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" DATE & TIME ", false);
          lcd.setCursor(0,1);
          lcd.print("SET DAY:     ");
          DateTime dt = rtc.now();
          if (dt.day() < 10) lcd.print("0");
          lcd.print(dt.day());
          lcd.print("d");
          break;
        }
      case btnUP: 
        {
          DateTime dt = rtc.now();
          byte lastDay = DateTime( DateTime(dt.year(),dt.month()+1,1,3,3,3) - TimeSpan(1, 1, 1, 1) ).day();
          rtc.adjust(DateTime(dt.year(), dt.month(), min(lastDay, dt.day()+10), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnRIGHT: 
        {
          DateTime dt = rtc.now();
          byte lastDay = DateTime( DateTime(dt.year(),dt.month()+1,1,3,3,3) - TimeSpan(1, 1, 1, 1) ).day();
          rtc.adjust(DateTime(dt.year(), dt.month(), min(lastDay, dt.day()+1), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnDOWN: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), max(1, dt.day()-10), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnLEFT: 
        {
          DateTime dt = rtc.now();
          rtc.adjust(DateTime(dt.year(), dt.month(), max(1, dt.day()-1), dt.hour(), dt.minute(), rtc.now().second()));
          break;
        }
      case btnSELECT: 
        {
          currMenu = 260;
          break;
        }
      }
      break;
    }
  case 300: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" SENSORS     ", true);                  
          break;
        }
      case btnUP: 
        {
          currMenu = 200;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 100;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 0;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 310;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 310;
          break;
        }
      }
      break;
    }
  case 310: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" BAROMETER   ", false);
          lcd.setCursor(0,1);
          lcd.print("ALTITUDE:");
          if ( abs(altitudeBMP) < 10000 ) lcd.print(" ");          
          if ( abs(altitudeBMP) <  1000 ) lcd.print(" ");
          if ( abs(altitudeBMP) <   100 ) lcd.print(" ");
          if ( abs(altitudeBMP) <    10 ) lcd.print(" ");
          if ( altitudeBMP >= 0 ) lcd.print(" ");
          lcd.print(altitudeBMP);
          lcd.print("m");
          break;
        }
      case btnUP: 
        {
          currMenu = 350;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 320;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 300;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 311;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 311;
          break;
        }
      }
      break;
    }
  case 311: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" BAROMETER   ", false);
          lcd.setCursor(0,1);
          lcd.print("SET ALT: ");
          if ( abs(altitudeBMP) < 10000 ) lcd.print(" ");          
          if ( abs(altitudeBMP) <  1000 ) lcd.print(" ");
          if ( abs(altitudeBMP) <   100 ) lcd.print(" ");
          if ( abs(altitudeBMP) <    10 ) lcd.print(" ");
          if ( altitudeBMP >= 0 ) lcd.print(" ");
          lcd.print(altitudeBMP);
          lcd.print("m");
          break;
        }
      case btnUP: 
        {
          altitudeBMP = min (10000, altitudeBMP + 10);
          break;
        }
      case btnRIGHT: 
        {
          altitudeBMP = min (10000, altitudeBMP + 1);
          break;
        }
      case btnDOWN: 
        {
          altitudeBMP = max (-100, altitudeBMP - 10);
          break;
        }
      case btnLEFT: 
        {
          altitudeBMP = max (-100, altitudeBMP - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 310;
          break;
        }
      }
      break;
    }
  case 320: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" BAROMETER   ", false);
          lcd.setCursor(0,1);
          lcd.print("OFFSET:   ");
          if ( abs(pressureOffset) <   100 ) lcd.print(" ");
          if ( abs(pressureOffset) <    10 ) lcd.print(" ");
          if ( pressureOffset >= 0 ) lcd.print(" ");
          lcd.print(pressureOffset);
          lcd.print("mb");
          break;
        }
      case btnUP: 
        {
          currMenu = 310;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 330;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 300;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 321;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 321;
          break;
        }
      }
      break;
    }
  case 321: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" BAROMETER   ", false);
          lcd.setCursor(0,1);
          lcd.print("SET OFFS.:");
          if ( abs(pressureOffset) <   100 ) lcd.print(" ");
          if ( abs(pressureOffset) <    10 ) lcd.print(" ");
          if ( pressureOffset >= 0 ) lcd.print(" ");
          lcd.print(pressureOffset);
          lcd.print("mb");
          break;
        }
      case btnUP: 
        {
          pressureOffset = min(100, pressureOffset + 10);
          break;
        }
      case btnRIGHT: 
        {
          pressureOffset = min(100, pressureOffset + 1);
          break;
        }
      case btnDOWN: 
        {
          pressureOffset = max(-100, pressureOffset - 10);
          break;
        }
      case btnLEFT: 
        {
          pressureOffset = max(-100, pressureOffset - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 320;
          break;
        }
      }
      break;
    }
  case 330: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" TEMPERATURE ", false);
          lcd.setCursor(0,1);
          lcd.print("SENSOR:   ");
          lcd.print(tempSensorName[tempSensor]);
          break;
        }
      case btnUP: 
        {
          currMenu = 320;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 340;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 300;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 331;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 331;
          break;
        }
      }
      break;
    }
  case 331: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" TEMPERATURE ", false);
          lcd.setCursor(0,1);
          lcd.print("SET SENS.:");
          lcd.print(tempSensorName[tempSensor]);
          break;
        }
      case btnUP: 
        {
          tempSensor = min(NoOfTEMPSENSORS, tempSensor + 1);
          break;
        }
      case btnRIGHT: 
        {
          tempSensor = min(NoOfTEMPSENSORS, tempSensor + 1);
          break;
        }
      case btnDOWN: 
        {
          tempSensor = max(0, tempSensor - 1);
          break;
        }
      case btnLEFT: 
        {
          tempSensor = max(0, tempSensor - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 330;
          break;
        }
      }
      break;
    }
  case 340: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" TEMP. OFFSET", false);
          lcd.setCursor(0,1);
          lcd.print(tempSensorName[tempSensor]);
          lcd.print(":    ");
          //if ( abs(temperatureOffset[tempSensor]) <   100 ) lcd.print(" ");
          if ( abs(temperatureOffset[tempSensor]) <    10 ) lcd.print(" ");
          if ( temperatureOffset[tempSensor] >= 0 ) lcd.print(" ");
          lcd.print(temperatureOffset[tempSensor]);
          lcd.print((char) 223);
          lcd.print("C");
          break;
        }
      case btnUP: 
        {
          currMenu = 330;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 350;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 300;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 341;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 341;
          break;
        }
      }
      break;
    }
  case 341: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" TEMP. OFFSET", false);
          lcd.setCursor(0,1);
          lcd.print("SET ");          
          lcd.print(tempSensorName[tempSensor]);
          lcd.print(":");
          //if ( abs(temperatureOffset[tempSensor]) <   100 ) lcd.print(" ");
          if ( abs(temperatureOffset[tempSensor]) <    10 ) lcd.print(" ");
          if ( temperatureOffset[tempSensor] >= 0 ) lcd.print(" ");
          lcd.print(temperatureOffset[tempSensor]);
          lcd.print((char) 223);
          lcd.print("C");
          break;
        }
      case btnUP: 
        {
          temperatureOffset[tempSensor] = min(99, temperatureOffset[tempSensor] + 10);
          break;
        }
      case btnRIGHT: 
        {
          temperatureOffset[tempSensor] = min(99, temperatureOffset[tempSensor] + 1);
          break;
        }
      case btnDOWN: 
        {
          temperatureOffset[tempSensor] = max(-99, temperatureOffset[tempSensor] - 10);
          break;
        }
      case btnLEFT: 
        {
          temperatureOffset[tempSensor] = max(-99, temperatureOffset[tempSensor] - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 340;
          break;
        }
      }
      break;
    }
  case 350: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" HUMIDITY    ", false);
          lcd.setCursor(0,1);
          lcd.print("OFFSET:    ");
          if ( abs(humidityOffset) <   100 ) lcd.print(" ");
          if ( abs(humidityOffset) <    10 ) lcd.print(" ");
          if ( humidityOffset >= 0 ) lcd.print(" ");
          lcd.print(humidityOffset);
          lcd.print("%");
          break;
        }
      case btnUP: 
        {
          currMenu = 340;
          break;
        }
      case btnDOWN: 
        {
          currMenu = 310;
          break;
        }
      case btnLEFT: 
        {
          currMenu = 300;
          break;
        }
      case btnRIGHT: 
        {
          currMenu = 351;
          break;
        }
      case btnSELECT: 
        {
          currMenu = 351;
          break;
        }
      }
      break;
    }
  case 351: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" HUMIDITY    ", false);
          lcd.setCursor(0,1);
          lcd.print("SET OFFSET:");
          if ( abs(humidityOffset) <   100 ) lcd.print(" ");
          if ( abs(humidityOffset) <    10 ) lcd.print(" ");
          if ( humidityOffset >= 0 ) lcd.print(" ");
          lcd.print(humidityOffset);
          lcd.print("%");
          break;
        }
      case btnUP: 
        {
          humidityOffset = min(100, humidityOffset + 10);
          break;
        }
      case btnRIGHT: 
        {
          humidityOffset = min(100, humidityOffset + 1);
          break;
        }
      case btnDOWN: 
        {
          humidityOffset = max(-100, humidityOffset - 10);
          break;
        }
      case btnLEFT: 
        {
          humidityOffset = max(-100, humidityOffset - 1);
          break;
        }
      case btnSELECT: 
        {
          currMenu = 350;
          break;
        }
      }
      break;
    }
  default: 
    {
      switch ( buttonState ) {
      case btnNONE: 
        {
          outputMenu(" UNDEFINED   ", false);                                    
          lcd.setCursor(0,1);
          lcd.print("SELECT to MAIN  ");
          break;
        }
      case btnUP: 
        {
          break;
        }
      case btnDOWN: 
        {
          break;
        }
      case btnLEFT: 
        {
          break;
        }
      case btnRIGHT: 
        {
          break;
        }
      case btnSELECT: 
        {
          currMenu = 0;
          break;
        }
      }
      break;
    }
  }
}
//===[ GENERAL FUNCTIONS END ]=======================================================================
