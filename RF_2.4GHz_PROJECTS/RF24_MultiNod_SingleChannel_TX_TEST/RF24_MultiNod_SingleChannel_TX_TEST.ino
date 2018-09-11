/* ===================================
DATA STRUCTURE TO SEND AND RECEIVE DATA (32bytes in total)
 *  2 uint_16  nod ID
 * 12 char(12) nod_desc
 *  9 char(9)  sensor_desc
 *  4 int_32   value
 *  1 byte     decimals 
 *  4 char     UoM
======================================= */



//### SETUP BEHAVIOUR ######################
  //#define SERIAL_DEBUG
  #define NOD_LED
//### SETUP NOD BASICS #####################
  const byte pipeAddress[6] = "00001";
  #define NOD_ID                  99
  #define NOD_DESC            "TestNod99"
//..........................................
  #define SLEEP_ITERATIONS   1   // SLEEP_ITERATIONS x 8s = sleep time, e.g.: 15=2min, 75=10min, 450=1hour
  #define NO_OF_SENSORS      4 
  #define pinLIGHT_SENS     A3
  #define pinLIGHT_SENS_VCC A0
  #define LIGHT_SENS_DELAY   5
  #define CE                 7   // CE pin of nRF24
  #define CSN                8   // CSN pin of nRF24
  #define BME280_ADDRESS    0x76
//..........................................
#ifdef NOD_LED
  #define pinLED 4
  #define LED_ON HIGH
  #define LED_OFF LOW
#endif
//################################

//#include <SPI.h> // no need, included in RF24
  #include <nRF24L01.h>
  #include <RF24.h>
  #include "LowPower.h"
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>

  struct StructTransmitData{
    uint16_t  nodID;
    char      nodDesc[12];
    char      sensDesc[9];
    int32_t   sensValue;
    uint8_t   sensValueDecimals;
    char      sensValueUoM[4];
  } structTXdata;

  bool isBMEOK = false;
  Adafruit_BME280 bme; // I2C

  RF24 radio( CE, CSN ); // CE, CSN

void updateSensors(){
#if defined( SERIAL_DEBUG )
  Serial.println( "Updating Sensors" );
  delay(1000);
#endif

  if (isBMEOK)  bme.takeForcedMeasurement(); // has no effect in normal mode  
  
#if defined( SERIAL_DEBUG )
  Serial.println( "FINISHED Updating Sensors" );
  delay(1000);
#endif

}

void readSensor( uint8_t sensorID ){
#if defined( SERIAL_DEBUG )
  Serial.print  ( "Reading Sensor: #" );
  Serial.println( sensorID );
  delay(1000);
#endif

  if (sensorID == 1) {
      digitalWrite(pinLIGHT_SENS_VCC, HIGH);
      strncpy(structTXdata.sensDesc, "PhotoRes", sizeof("PhotoRes") );
      structTXdata.sensValueDecimals = 0;
      strncpy(structTXdata.sensValueUoM, "R", sizeof("%") );
      delay( LIGHT_SENS_DELAY ); //not sure this delay is needed, 3 commands have been executed since the power pin was set to high
      structTXdata.sensValue = analogRead(pinLIGHT_SENS);
      digitalWrite(pinLIGHT_SENS_VCC, LOW);
#if defined( SERIAL_DEBUG )
      Serial.print  ( "FINISHED Reading Sensor 1" );
      delay(1000);
#endif
  }
  else if (sensorID == 2) {     //Sensor Temperature
      strncpy(structTXdata.sensDesc, "Tempr.", sizeof("Tempr.") );
      float flTemp = -99.99;
      if ( isBMEOK ) flTemp = bme.readTemperature();
      structTXdata.sensValue = int( flTemp * 100 );
      structTXdata.sensValueDecimals = 2;
      strncpy(structTXdata.sensValueUoM, "*C", sizeof("*C") );
  }
  else if (sensorID == 3) {     //Sensor Humidity
      strncpy(structTXdata.sensDesc, "Humid.", sizeof("Humid.") );
      float flHumid = -99.99;
      if ( isBMEOK ) flHumid = bme.readHumidity();
      structTXdata.sensValue = int( flHumid * 100 );
      structTXdata.sensValueDecimals = 2;
      strncpy(structTXdata.sensValueUoM, "%", sizeof("%") );
  }
  else if (sensorID == 4) {     //Sensor Pressure (Absolute)
      strncpy(structTXdata.sensDesc, "AtmPress", sizeof("AtmPress") );
      float flAbsPress = -999.99;
      if ( isBMEOK ) flAbsPress = bme.readPressure() / 100.0F;
      //float flRelPress = bme.seaLevelForAltitude(NOD_ALTITUDE_M, flAbsPress);
      structTXdata.sensValue = int( flAbsPress );
      structTXdata.sensValueDecimals = 0;
      strncpy(structTXdata.sensValueUoM, "mb", sizeof("mb") );
  }    
  else {      //Battery Voltage & Internal Temperature, keep 0 sensor dedicated for this
#if defined( SERIAL_DEBUG )
      Serial.println( "STARTING Sensor 0 or DEFAULT" );
      delay(1000);
#endif      
      strncpy(structTXdata.sensDesc, "NOD", sizeof("NOD") );
      
      int32_t sensVal = readVcc();
      structTXdata.sensValue = 100 * sensVal;
      structTXdata.sensValue += int( readTemp() );
      structTXdata.sensValueDecimals = 2;
      strncpy(structTXdata.sensValueUoM, "mVC", sizeof("mVC") );
#if defined( SERIAL_DEBUG )
      Serial.println( "FINISHED Reading Sensor 0 or DEFAULT" );
      delay(1000);
#endif      
//      break;
  }   
}

void setupBME(){
  if (! bme.begin( BME280_ADDRESS )) { //BME280_ADDRESS
#if defined(SERIAL_DEBUG)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
#endif
  }
  else {
    isBMEOK = true;
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
  }
}

void setupRF24(){
  radio.begin();
  radio.openWritingPipe(pipeAddress);
  //radio.setPALevel(RF24_PA_MIN);
  radio.setPALevel(RF24_PA_MAX);
  //radio.setPayloadSize(32); // max 32
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.stopListening();
#if defined( SERIAL_DEBUG )
  Serial.println( "RADIO ON" );
  Serial.println( "-------------------" );  
#endif
}
    
void setup() {
#if defined(SERIAL_DEBUG)
  Serial.begin(9600);
  Serial.println("");
  Serial.println("##### !!! BEGIN !!! #####");
#endif

#if defined( NOD_LED )
  pinMode(pinLED, OUTPUT);
#endif
  pinMode(pinLIGHT_SENS,INPUT);
  pinMode(pinLIGHT_SENS_VCC,OUTPUT);

  structTXdata.nodID = NOD_ID;
  strncpy(structTXdata.nodDesc, NOD_DESC, sizeof(NOD_DESC) );
#if defined(SERIAL_DEBUG)
  Serial.print  ( "Nod ID   : " ); Serial.println( structTXdata.nodID );
  Serial.print  ( "Nod Desc.: " ); Serial.println( structTXdata.nodDesc );
  Serial.println( "-------------------" );
#endif

  setupRF24();
  setupBME();

// SHOW END OF SETUP ONE LED AND/OR SERIAL ============================
#if defined( NOD_LED )
  for (uint8_t b=0; b<5; b++) {
    digitalWrite(pinLED, LED_ON);
    delay(300);
    digitalWrite(pinLED, LED_OFF);
    delay(300);
  }
#endif

#if defined( SERIAL_DEBUG )
  Serial.println( "END SETUP" );
  Serial.println( "===================" );
  delay(1000);  
#endif

}

void loop(){
#if defined( SERIAL_DEBUG )
  Serial.println( "LOOPING" );
  delay(1000);
#endif      

  
  updateSensors();
  for (uint8_t b = 0; b<=NO_OF_SENSORS; b++) { // START LOOPING THROUGH SENSORS
#if defined( SERIAL_DEBUG )
    Serial.print( "reading " ); Serial.println( b );
    delay(1000);
#endif      
    readSensor( b );    
#if defined(SERIAL_DEBUG)
    Serial.println( "PREPARING DATA:");    
    Serial.print  ( "Nod ID:       " ); Serial.println(structTXdata.nodID);
    Serial.print  ( "Nod Desc.:    " ); Serial.println(structTXdata.nodDesc);
    Serial.print  ( "Sens.Desc.:   " ); Serial.println(structTXdata.sensDesc);    
    Serial.print  ( "Sens.Value:   " ); Serial.println(structTXdata.sensValue);
    Serial.print  ( "Sens.Decim.:  " ); Serial.println(structTXdata.sensValueDecimals);    
    Serial.print  ( "Sens.Val.UoM: " ); Serial.println(structTXdata.sensValueUoM);    
    Serial.println( "-------------------" );
    Serial.println("Checking Radio Channel is free");
    delay(1000);
#endif
    radio.startListening();
    if ( !radio.available() ){
#if defined( NOD_LED )
      digitalWrite(pinLED, LED_ON);
#endif
      radio.stopListening();
#if defined(SERIAL_DEBUG)
      Serial.println("Stop listening and Start sending");
      delay(1000);
#endif
      //if ( !true ){
      if ( !radio.write( &structTXdata, sizeof( structTXdata ) ) ){
#if defined( NOD_LED )
        for (uint8_t bLed=0; bLed<3; bLed++) {
          digitalWrite(pinLED, LED_OFF);
          delay(100);
          digitalWrite(pinLED, LED_ON);
          delay(100);
        }
#endif
#if defined(SERIAL_DEBUG)
        Serial.println("TX ERROR ###");
        Serial.println( "-------------------" );        
        delay(1000);
#endif
      }
      else {
#if defined(SERIAL_DEBUG)
        Serial.println("TX OK !!!");
        Serial.println( "-------------------" );
        delay(1000);
#endif
      }
#if defined( NOD_LED )
      digitalWrite(pinLED, LED_OFF);
#endif
    }
  } // END LOOPING THROUGH SENSORS
  
#if defined(SERIAL_DEBUG)
  Serial.println( "Preparing to sleep" );
  Serial.println( "===================" );    
  delay(1000);
#endif

#ifndef SLEEP_ITERATIONS
  #define SLEEP_ITERATIONS 1
#endif
  radio.powerDown();
  for (byte sleepCounter = 0; sleepCounter < SLEEP_ITERATIONS; sleepCounter++)
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  radio.powerUp();

#if defined(SERIAL_DEBUG)
  Serial.println( "Woke Up" );
  delay(1000);
#endif

}

uint16_t readVcc(){
  uint16_t result;
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

float readTemp(){
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
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset of 324.31 could be wrong. It is just an indication.
  t = (wADC - 324.31 ) / 1.22;

  // The returned temperature is in degrees Celsius.
  return (t);
}

/*
char *ftoa(char *a, double f, int precision)
{
  long p[] = {0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000};

  char *ret = a;
  long heiltal = (long)f;
  itoa(heiltal, a, 10);
  while (*a != '\0') a++;
  *a++ = '.';
  long desimal = abs((long)((f - heiltal) * p[precision]));
  itoa(desimal, a, 10);
  return ret;
}
*/
