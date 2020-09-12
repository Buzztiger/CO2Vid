// CO2Vid CO2 CoV Monitor and Logging
#include <Arduino.h>

#include <sys/time.h>
#include <TimeLib.h>

//#include <time.h>

#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include "Adafruit_SGP30.h"

#include "BluetoothSerial.h"

// SD card
#include "FS.h"
#include "SD.h"
#include <SPI.h>

// Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <Adafruit_NeoPixel.h>

#include <FastLED.h>


#define LED_PIN   21
#define LED_COUNT 1


#define DATA_PIN 21
#define NUM_LEDS 1
CRGB leds[NUM_LEDS];

// TODO
// Bluetooth interface for reconfiguration

// Display
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    27 // Reset pin # (or -1 if sharing Arduino reset pin)

//Adafruit_NeoPixel strip(1, LED_PIN, NEO_GRB + NEO_KHZ800);
//Adafruit_NeoPixel strip(1, LED_PIN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

File myFile;
String buffer;

bool sd_card_storage = false;
bool alert = false;               // Above 1000 ppm?

// Sensor Calibration TODO  -----------------------------------

/* When activated for the first time a
period of minimum 7 days is needed so that the algorithm can find its initial parameter set for ASC. The sensor has to be exposed
to fresh air for at least 1 hour every day. 
*/ 

// Count uptime in minutes
// Count fresh air time/day  < 500 ppm

int  calibration_timer = 0;
bool calibrated        = false;
bool calibrating       = false;
bool fresh_air         = true;
bool startup           = true;

int fresh_air_timer    = 0;
int fresh_air_time     = 0;  // Seconds under 500ppm
int RED   = 0;
int GREEN = 0;
int BLUE  = 0;
int days = 0;
int calibration_days = 0;

// 1h under 500ppm in 24h
// 7 days in total

#define SD_CS 5

// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

int new_pressure       = 1024; // in mbar
int new_altitude       = 2;    // in meters
int new_interval       = 2;    // in seconds

float current_temp    = 20.0f;
float current_hum     = 50.0f;
int current_CO2       = 0;
int current_TVOC      = 0;
int current_H2        = 0;
int current_Ethanol   = 0;

uint16_t TVOC_base    = 0;
uint16_t eCO2_base    = 0;

const int array_length  = 128;  // Every 10 Seconds covers the last two minutes TODO check size in mem??
const int plot_length   = 128;

float batt_cor  = 0.14f;  // TODO calibrate this properly
float batt_volt = 0;

#define fan_pin 1          // Fan PIN -> transistor 5V line to fan
const int plot_minimum = 300;   //

String data[array_length];       // Every 10 Seconds covers the last two minutes TODO check size in mem??
int array_counter = 0;           // 

int plot[128];            // 2m plot    
int plot_1h[128];         // 1h plot  7200  points (@2s intervall) -> 56 points  = 1px
int plot_6h[128];         // 6h plot  43200 points (@2s intervall) -> 337 points = 1px
int counter_6h = 0;       //337
unsigned long sum_6h  = 0;
int counter_1h = 0;       //56
unsigned long  sum_1h = 0;
int plot_mode  = 0;       // 0=live   1=1h   2=6h
float scaling_factor = 0.05;

long time_offset  = 1598918400;  //  1/9/2020 00:00

String BTStringH = "";
String BTStringL = "";
String payload   = "";

SCD30 airSensor;      // CO2 Sensor
Adafruit_SGP30 sgp;   // TVOC Sensor

/* return absolute humidity [mg/m^3] with approximation formula
* @param temperature [°C]
* @param humidity [%RH]
*/
uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

// Append array to the SD card
void appendArray(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  for (byte i = 0; i < array_length; i = i + 1) {                
      if(file.print(data[i])) {
        //Serial.println("Message appended");
      } else {        
        Serial.println("Append failed");
      }
  }
  file.close();
}

void set_time(String epoch_time2){
  int epoch_time     = 1599218222;
  timeval epoch      = {epoch_time, 0};
  const timeval *tv  = &epoch;
  timezone utc       = {0,0};
  const timezone *tz = &utc;
  settimeofday(tv, tz);
}

float getBatteryVoltage(){     // Battery Voltage
  float b_volt = (analogRead(A13)/4095.0f)*2.0f*3.30f*1.1f-batt_cor;
  return b_volt;
}

void setLED(int CO2){
  RED   = 0;
  GREEN = 0;
  BLUE  = 0;
  if( CO2 < 400 ){  
    CO2 = 400;
  }

  if( CO2 < 700 ){                  // GREEN
    //GREEN = 255;
    RED   = 0;
    BLUE  = 0;
    GREEN = 255;
  }
  if( (CO2 >= 700) && (CO2 < 1000)){ // ORANGE
    
    GREEN = 255;
    RED   = 165;
    BLUE  = 0;
  }
  if(CO2 >= 1000){                    // RED
    GREEN = 0;
    RED   = 255;
    BLUE  = 0;
  }
  //strip.setPixelColor(0, strip.Color(RED, GREEN, BLUE));
  //Serial.println(GREEN);
  //Serial.println(RED);
  //Serial.println(BLUE);
  leds[0] = CRGB(RED,GREEN,BLUE);
  FastLED.show();
}

void setup() {

  // LED
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(64);

  // SD Card
  // Initialize SD card
  /*
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }
  */

  // DISPLAY
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Init plot arrays
  for (byte i = 0; i < array_length; i = i + 1) {                
      plot[i]    = plot_minimum;
      plot_1h[i] = plot_minimum;
      plot_6h[i] = plot_minimum;
  }  
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen. TODO change this to CO2VID Logo
  display.display();
  display.clearDisplay();
  display.setTextColor(1);
  
  int epoch_time = 1599218222;
  setTime(epoch_time);
  Wire.begin();
  SerialBT.begin("CO2Vid-Mon");
  Serial.begin(9600);
  
  // CO2 Sensor
  airSensor.begin();
  airSensor.setMeasurementInterval(new_interval);   // Default 10 seconds
  airSensor.setAltitudeCompensation(new_altitude);  // Set altitude of the sensor in m
  airSensor.setAmbientPressure(new_pressure);       // Current ambient pressure in mBar: 700 to 1200
  
  // TVOC Sensor
  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }
  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

}

void loop()
{ 

  // Incoming command ----------------------------------------------------------------
  if (SerialBT.available()){
    Serial.println("BT Data");

    //String cmnd = SerialBT.readString();
    String cmnd  = SerialBT.readStringUntil('\n');
    cmnd.trim();
    String cmnd1   = cmnd.substring(0,1); // Get First Character
    String payload = cmnd.substring(1);   // Get Payload, rest of the string
    Serial.println(cmnd1);
    Serial.println(payload);

    // Request stored data
      // RingBuffer             (R)
      // SD card data           (S)
    // Set/Get Ambient Pressure        (P) P S/G 1024 in mbar
    // Set/Get Altitude Compensation   (A) C S/G 2 in meters
    // Set/Get Measurement Intervall   (I) I S/G xxxx in seconds
    // Set/Get Time                    (T) T S/G 1599050020 epoch  
    // Switch  Fan                     (F) F0 off, F1 on
    // Get/Set plot_mode               (G) 0,1,2  live,1h,6h
    // Get Battery Voltage             (B) , returns B:X.XX (e.g. B:3.7)

    // DATA    ========================
    if (cmnd1.equals("R")) {
      Serial.println("ringbuffer");
      for (byte i = 0; i < array_length; i = i + 1) {        
        SerialBT.println(data[i]);
        delay(500);    // Do we need a delay here?
      }
    }
    if (cmnd1.equals("B")) {
      float batt_volt = getBatteryVoltage();
      String batt = "B:" + String(batt_volt);
      Serial.println(batt);
      SerialBT.println(batt);
    }

    if (cmnd1.equals("T")) {
      Serial.print("synctime: ");
      Serial.println(payload);
      set_time(payload);
    }


    /*
    if (cmnd1.equals("S")) {      // Read from SD Card and send via BT
      Serial.println("sd_card");            
      myFile = SD.open("/data.txt");

      if (!myFile) {
        Serial.print("The text file cannot be opened");
        while(1);
      }

      while (myFile.available()) {
        buffer = myFile.readStringUntil('\n');
        Serial.println(buffer); //Printing for debugging purpose         
      }   
    } */

    // SETTINGS ========================
    if (cmnd1.equals("P")) {
      Serial.print("pressure: ");
      Serial.println(payload);
      airSensor.setAmbientPressure(payload.toInt());
    }
    if (cmnd1.equals("A")) {
      Serial.print("altitude: ");
      Serial.println(payload);
      airSensor.setAltitudeCompensation(payload.toInt());
    }
    if (cmnd1.equals("I")) {
      Serial.print("intervall: ");
      Serial.println(payload);
      airSensor.setMeasurementInterval(payload.toInt());
    }
    
    if (cmnd1.equals("F")) {
      Serial.println(payload);
      Serial.print("fan: ");
      String cmnd3 = payload.substring(0,1);
      if (cmnd3.equals("0")) {
        Serial.println("Fan Off");
      }
      if (cmnd3.equals("1")) {
        Serial.println("Fan On");
      }
    }

    if (cmnd1.equals("G")) {
      Serial.print("intervall: ");
      Serial.println(payload);
      String cmnd3 = payload.substring(0,1);
      if (cmnd3.equals("0")) {
        Serial.println("Live Plot");
        plot_mode = 0;
      }
      if (cmnd3.equals("1")) {
        Serial.println("1h Plot");
        plot_mode = 1;
      }
      if (cmnd3.equals("6")) {
        Serial.println("6h Plot");
        plot_mode = 6;
      }      
    }    
  }
  
  // Air Sensors
  if (airSensor.dataAvailable())
  {
    current_CO2     = airSensor.getCO2();
    current_temp    = airSensor.getTemperature();
    current_hum     = airSensor.getHumidity();
    unsigned long t_stamp = (unsigned long) now();
    t_stamp               = t_stamp - time_offset;
    
    if (current_CO2 > 999){
      alert = true;     
       
    }else{
      alert = false;
    }
    setLED(current_CO2);

    /*
    if( current_CO2 < 700 ){                  // GREEN
      strip.setPixelColor(0,255,0,0);
      strip.show();
      delay(1000);
    }

    if( (current_CO2 > 700) && (current_CO2 < 1000)){ // ORANGE
      strip.setPixelColor(0,0,0,255);
      strip.show();
      delay(1000);
    }
    if(current_CO2 > 1000){                    // RED
      strip.setPixelColor(0,0,255,0);
      strip.show();
      delay(1000);
    }
    */

    // Battery Voltage
    //Serial.printf("raw adc value No. %d: %d\n", i, analogRead(A13));
    
    // (2.05 / 3.3) = 0.62 * 4098 = 2540
    //  V / 3.3 * 4098 = 2540

    //float batt_volt = (analogRead(A13)/4095)*2*3.3*1.1;
    //float batt_volt = (1481.0f/4095.0f)*2.0f*3.30f*1.1f;

    // (ADC.read(battery)/4095)*2*3.3*1.1;
    //Serial.println(analogRead(A13));
    //Serial.println(batt_volt);

    // Real time plot array
    memcpy(plot, &plot[1], sizeof(plot) - sizeof(int));    // Shift datapoints one down    
    plot[127] = current_CO2;                               // Add new datapoint

    counter_1h = counter_1h + 1;
    sum_1h     =  sum_1h + current_CO2;
    
    counter_6h = counter_6h + 1;
    sum_6h     =  sum_6h + current_CO2;
   
    // 1h plot    
    if( counter_1h >= 56 ){                                           
      memcpy(plot_1h, &plot_1h[1], sizeof(plot_1h) - sizeof(int));    // Shift datapoints one down
      plot_1h[127] = sum_1h/56;                                       // Add new datapoint
      counter_1h   = 0;
      sum_1h       = 0;
    }

    // 6h plot
    if( counter_6h >= 337 ){                                          
      memcpy(plot_6h, &plot_6h[1], sizeof(plot_6h) - sizeof(int));    // Shift datapoints one down
      plot_6h[127] = sum_6h/337;                                      // Add new datapoint
      counter_6h   = 0;
      sum_6h       = 0;
    }

  // TVOC Sensor
  // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
  //float temperature = 22.1; // [°C]
  //float humidity    = 45.2; // [%RH]
  
  sgp.setHumidity(getAbsoluteHumidity(current_temp, current_hum));

  //sgp.getIAQBaseline(&eCO2_base, &TVOC_base);
  //sgp.setIAQBaseline(eCO2_baseline, TVOC_baseline);

    if (sgp.IAQmeasure()) {
      current_TVOC    = sgp.TVOC;
    }

    if (sgp.IAQmeasureRaw()) {
      current_H2      = sgp.rawH2;
      current_Ethanol = sgp.rawEthanol;
    }

    // Update Display ------------------------------------------------------------    
    display.clearDisplay();
    display.setTextSize(3);    
    if(alert){
      display.setCursor(7,40); // Adjust this location depending on above or below 1000 TODO
      display.print("!");
      display.setCursor(29,40); // Adjust this location depending on above or below 1000 TODO
      display.print(current_CO2);
      display.setCursor(105,55);
      display.setTextSize(1);
      display.print("ppm");
      scaling_factor = 0.007;
    }else{
      display.setCursor(39,40); // Adjust this location depending on above or below 1000 TODO
      display.print(current_CO2);
      display.setCursor(100,55);
      display.setTextSize(1);
      display.print("ppm");
      scaling_factor = 0.05;
    }
    // Graph two modes: OK <1000 ALERT > 1000ppm

    // OK Mode < 1000 ppm
    // Graph height 300-1000 = 700 ppm = 35 px    scaling_factor = 0.05

    // ALERT Mode > 1000 ppm
    // Graph height 300-5000 = 4700 ppm = 35 px   scaling_factor = 0.007
    // 1000 ppm at  5px

    //display.drawFastHLine(0, 35, 127,1);   // X-Axis
    //display.drawFastHLine(0, 35, 127,1); // 1000-Marker 
    //                        x   y   l
    //display.drawFastVLine(5,  35 - 10, 10,1);   // 500 ppm  500  - 300 = 200 *0.05 = 10
    //display.drawFastVLine(10, 35 - 35, 35,1);   //1000 ppm  1000 - 300 = 700 *0.05 = 35
    //display.drawFastVLine(15, 35 -  0,  0,1);   // 300 ppm             = 0
    
    // Plot
    if(plot_mode == 0) {       // Live Plot
      for (byte i = 0; i < plot_length; i = i + 1) {                      
        int l    = round( (plot[i]-plot_minimum) *scaling_factor);
        if (l>35){
          l = 35; }
        int ypos = 35 - l;
        if(l>0){
          display.drawFastVLine(i, ypos, l,1);
        }      
      }
    }
  
    if(plot_mode == 1) {       // 1h Plot
      Serial.println("1h Plot");
      for (byte i = 0; i < plot_length; i = i + 1) {                
        int l    = round((plot_1h[i]-plot_minimum)*scaling_factor);
        if (l>35){ l = 35; }
        int ypos = 35 - l;
        if(l>0){
          display.drawFastVLine(i, ypos, l,1);
        }      
      }
    }

    if(plot_mode == 6) {       // 6h Plot
      Serial.println("6h Plot");
      for (byte i = 0; i < plot_length; i = i + 1) {                
        int l    = round((plot_6h[i]-plot_minimum)*scaling_factor);
        if (l>35){ l = 35; }
        int ypos = 35 - l;
        if(l>0){
          display.drawFastVLine(i, ypos, l,1);
        }      
      }
    }    
   
    display.display();

    // Data string format
    // L,epochtime,co2,temp,hum,tvoc,h2,ethanol
    
    BTStringH = "H:" + String(t_stamp) + ":" + String(current_CO2) + ":" + String(current_temp) + ":" + String(current_hum) + ":" + String(current_TVOC) + ":" + String(current_H2) + ":" + String(current_Ethanol) + "\r\n";
    BTStringL = "L:" + String(t_stamp) + ":" + String(current_CO2) + ":" + String(current_temp) + ":" + String(current_hum) + ":" + String(current_TVOC) + ":" + String(current_H2) + ":" + String(current_Ethanol); // + "\r\n";

    Serial.println(BTStringL);             // Send via serial connection Debug
    SerialBT.println(BTStringL);           // Send via BT TODO only if connection is available...
        
   /*
    if(sd_card_storage){
      data[array_counter++] = BTStringH;     // Store to ringbuffer array
      if( array_counter >= array_length ){  // Store Ringbuffer to SD Card
          display.println("saving to SD Card");
          array_counter = 0;
          appendArray(SD, "/data.txt", BTStringH.c_str());
      }
    }
   */

  // Calibration Timers
  if(calibrating){
    if ( (current_CO2 < 500) && (!fresh_air)) {
      fresh_air_timer = millis(); // Start timer
      Serial.println("Fresh Air!"); 
      fresh_air = true;
    }
    if ( (current_CO2 >= 500) && (fresh_air)) {
      fresh_air_time = fresh_air_time + (millis() - fresh_air_timer)/1000;  // Add seconds spend in fresh air to timer
      fresh_air_timer = 0;
      Serial.print("Fresh Air STOPPED: "); 
      Serial.println(fresh_air_time + " sec" );
      fresh_air = false;
    }

    if(days < 8){
      
      if(millis()/1000 > 43200*(days+1)){     // 12hrs passed
        sgp.getIAQBaseline(&eCO2_base, &TVOC_base);
        // Store baseline values

      }
      if(millis()/1000 > 86400*(days+1)){     // New day
        days = days + 1;
        // 1h under 500ppm?
        if(fresh_air_time/1000 > 3600){
          fresh_air_time  = 0;
          calibration_days = calibration_days + 1;
        }
        // Storage TVOC Baseline in EEPROM
        
      }

    } 
       
  }

  }
  

 }

// Archive --------------------------------------------------------------------------------------------------------------------
/*
    // (ADC.read(battery)/4095)*2*3.3*1.1;
    //Serial.println(analogRead(A13));

    //Serial.printf("raw adc value No. %d: %d\n", i, analogRead(A13));
    
    // (2.05 / 3.3) = 0.62 * 4098 = 2540
    //  V / 3.3 * 4098 = 2540

    //float batt_volt = (analogRead(A13)/4095)*2*3.3*1.1;

 if (strcmp (topic,"sensors/scd30_2/set/pressure") == 0) {                               // Set pressure
      new_pressure = atoi(payload);      
      if(new_pressure > 1200) {
        new_pressure = 1200;
      }
      if(new_pressure < 700) {
        new_pressure = 700;
      }
      airSensor.setAmbientPressure(new_pressure);
      char buf[7];
      mqttClient.publish("sensors/scd30_2/pressure", 0, true, ftoa(new_pressure, buf, 2));

 }

 if (strcmp (topic,"sensors/scd30_2/set/interval") == 0) {                               // Set interval
      new_interval = atoi(payload);      
      if(new_interval < 2) {
        new_interval = 2;
      }
      if(new_interval > 1800) {
        new_interval = 1800;
      }
      airSensor.setMeasurementInterval(new_interval);
      char buf[7];
      mqttClient.publish("sensors/scd30_2/interval", 0, true, ftoa(new_interval, buf, 2));

 }

if (strcmp (topic,"sensors/scd30_2/set/altitude") == 0) {                               // Set altitude
      new_altitude = atoi(payload);      
      if(new_altitude < 0) {
        new_altitude = 0;
      }
      if(new_altitude > 5000) {
        new_altitude = 5000;
      }
      airSensor.setAltitudeCompensation(new_altitude);
      char buf[7];
      mqttClient.publish("sensors/scd30_2/altitude", 0, true, ftoa(new_altitude, buf, 2));

 }
*/
/*
// Write the sensor readings on the SD card
void logSDCard() {
  dataMessage = String(readingID) + "," + String(dayStamp) + "," + String(timeStamp) + "," + 
                String(temperature) + "\r\n";
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
}
*/

/*
char *ftoa( double f, char *a, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};
 
 char *ret = a;
 long heiltal = (long)f;
 itoa(heiltal, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long desimal = abs((long)((f - heiltal) * p[precision]));
 itoa(desimal, a, 10);
 return ret;
}

  while(true)
  {
    strip.setBrightness(8);
    strip.setPixelColor(0, strip.Color(255, 0, 0));    
    strip.show();
    delay(2000);
    strip.setBrightness(8);
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();
    delay(2000);
    strip.setBrightness(8);
    strip.setPixelColor(0, strip.Color(0, 0, 255));
    strip.show();
    delay(2000);
    Serial.println("LED");
  }

*/