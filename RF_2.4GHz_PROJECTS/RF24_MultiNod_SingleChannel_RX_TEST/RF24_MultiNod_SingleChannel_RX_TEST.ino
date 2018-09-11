/* ===================================
DATA STRUCTURE TO SEND AND RECEIVE DATA (32bytes in total)
 *  2 uint_16 nod ID
 * 12 char(4) nod_desc
 *  9 char(4) sensor_desc
 *  4 int_32  value
 *  1 byte  decimals 
 *  4 char  UoM
======================================= */

//### SETUP ######################
  #define MAX_NO_OF_SENSORS     25
  #define LOW_BATT_LEVEL_mV     2500  
  #define HIGH_TEMP_LEVEL_C     40  
  #define SCREEN_UPDATE_DELAY   10000
  #define OLED_I2C_ADDRESS      0x3C
  #define CE                    7   // CE pin of nRF24
  #define CSN                   8   // CSN pin of nRF24
  const byte pipeAddress[6]   = "00001";
  #define INTERRUPT_PIN         3
//################################

  #include <nRF24L01.h>
  #include <RF24.h>
  #include "SSD1306Ascii.h"
  #include "SSD1306AsciiAvrI2c.h"
  
  RF24 radio( CE, CSN ); // CE, CSN
  
  

  struct StructReceiveData{
    uint16_t  nodID;
    char      nodDesc[12];
    char      sensDesc[9];
    int32_t   sensValue;
    uint8_t   sensValueDecimals;
    char      sensValueUoM[4];
  } structRXdata;

  struct StructStoreData{
    uint16_t  nodID;
    char      nodDesc[12];
    char      sensDesc[9];
    int32_t   sensValue;
    uint8_t   sensValueDecimals;
    char      sensValueUoM[4];
    uint32_t  lastRXMillis;
    bool      isOldRX;
  } structStoreData[MAX_NO_OF_SENSORS + 1];

  SSD1306AsciiAvrI2c oled;
  
  unsigned long lastCountMillis = 0;
  unsigned long lastRXMillis = 0;
  
  volatile uint8_t sensID2Disp = 1;
  volatile boolean updateDisplay = false;
  
  const unsigned long debouncing_time = 50000; //Debouncing Time in Microseconds
  volatile unsigned long lastInterruptMicros;
  bool buttonPressed = false;

void debounceButtonInterrupt() {
  //noInterrupts();
  if((long)(micros() - lastInterruptMicros) >= debouncing_time)
  {
    performInterruptGoal();
    lastInterruptMicros = micros();
  }
  //interrupts();
}

void performInterruptGoal() {
  buttonPressed = true;
}

bool checkRadio(){
  if (radio.available()) {
    radio.read( &structRXdata, sizeof( structRXdata ) );
    lastRXMillis = millis();
    return true;
  }
  else{
    return false;
  }
}

void sortRXdata(){
  
  int sensID = -1;
  for ( uint16_t sensRunID=1; sensRunID <= MAX_NO_OF_SENSORS; sensRunID++ ) {
    if ( (structStoreData[sensRunID].nodID == 0) || 
         ((structRXdata.nodID == structStoreData[sensRunID].nodID) && 
          ( strcmp(structRXdata.nodDesc , structStoreData[sensRunID].nodDesc) == 0 ) && 
          ( strcmp(structRXdata.sensDesc , structStoreData[sensRunID].sensDesc) == 0 ) ) )
    {
      sensID = sensRunID;
      break;
    }
    if ( millis() - structStoreData[sensRunID].lastRXMillis > 3600000 ) {
      structStoreData[sensRunID].isOldRX = true;
    }
    if ( structStoreData[sensRunID].nodID == 0 ) {
      break;
    }
    
  }
  
  if ( sensID == -1 ){
    for ( uint16_t sensRunID = 0; sensRunID <= MAX_NO_OF_SENSORS; sensRunID++ ) {
      if ( structStoreData[sensRunID].isOldRX == true) {
        sensID = sensRunID;
        break;
      }
    }
  }
  structStoreData[sensID].nodID = structRXdata.nodID;
  strncpy(structStoreData[sensID].nodDesc, structRXdata.nodDesc, sizeof(structRXdata.nodDesc) );  
  strncpy(structStoreData[sensID].sensDesc, structRXdata.sensDesc, sizeof(structRXdata.sensDesc) );  
  structStoreData[sensID].sensValue = structRXdata.sensValue;
  structStoreData[sensID].sensValueDecimals = structRXdata.sensValueDecimals;
  strncpy(structStoreData[sensID].sensValueUoM, structRXdata.sensValueUoM, sizeof(structRXdata.sensValueUoM) );  
  structStoreData[sensID].lastRXMillis = lastRXMillis;
  structStoreData[sensID].isOldRX = false;
  lastRXMillis = 0;

  //sensID2Disp = sensID;
}


void outSensData( uint8_t sensID ){
  
  double result;
  result = structStoreData[sensID].sensValue / (round( pow(10,structStoreData[sensID].sensValueDecimals) )*1.0);
  
  oled.clear();
  oled.println("> YYYY-MM-DD  HH:MM < ");
  oled.print  (sensID); oled.print  ( " : " ); oled.print  ( structStoreData[sensID].nodID ); oled.print  ( " : " ); oled.println( structStoreData[sensID].nodDesc );
  if ( strcmp(structStoreData[sensID].sensDesc, "NOD") == 0 ) {
    oled.println(" >>> NOD HEALTH <<<");
    uint16_t batt = int(result);
    uint16_t temp = structStoreData[sensID].sensValue % round(pow(10,structStoreData[sensID].sensValueDecimals));
    if ( batt <= LOW_BATT_LEVEL_mV ) {
      oled.print(" ! BATT !");
    }
    if (temp >= HIGH_TEMP_LEVEL_C) {
      oled.print("! HOT CPU !");
    }
    oled.println("");  
    oled.print  ("Battery:   "); oled.print  ( batt );  oled.println( "mV" );
    oled.print  ("CPU Temp.: "); oled.print( temp ); oled.println( "*C" );
  }
  else {
    oled.print  (" >>> "); oled.print  ( structStoreData[sensID].sensDesc ); oled.println(" <<<");
    oled.println("");
    oled.print  (result); oled.print  (" ");oled.println( structStoreData[sensID].sensValueUoM );
    //oled.print  (structStoreData[sensID].sensValue); oled.print( " " ); oled.println( structStoreData[sensID].sensValueDecimals );
    //oled.print  ( int(result) ); oled.print("."); oled.print( (result-int(result))*pow(10,structStoreData[sensID].sensValueDecimals) ); oled.println( structStoreData[sensID].sensValueUoM );
    oled.println("");
  }
  oled.println("");
  oled.print  ("Age:       "); oled.print  ( (millis() - structStoreData[sensID].lastRXMillis) / 1000); oled.println( "s" );

}

void setup() {
  //Serial.begin(9600);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), debounceButtonInterrupt, LOW);

  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.println("=== STARTING ===");
  radio.begin();
  radio.openReadingPipe(0, pipeAddress);
  //radio.setPALevel(RF24_PA_MIN);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening();
  oled.println("=== RADIO STARTING ===");

  //Serial.println("END SETUP");
  interrupts();
}

void loop() {
  //Serial.println("LOOP");
  if ( ( buttonPressed ) || ( millis() - lastCountMillis > SCREEN_UPDATE_DELAY ) )
  { 
    //Serial.println("update display");
    buttonPressed = false;
    outSensData( sensID2Disp );
    sensID2Disp++;
    if ( (sensID2Disp > MAX_NO_OF_SENSORS) || (structStoreData[sensID2Disp].nodID == 0) ) sensID2Disp = 1 ;
    lastCountMillis = millis();
  }

  //Serial.println( "Check Radio" );
  if ( checkRadio() )
  {
    //Serial.println( "Sort RadioData" );
    sortRXdata();
  }

  delay(10);
}

long readVcc(){
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(10); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV ; 1126400 = 1.1*1024*1000
  return result;
}

double readInternalTemp(){
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}
