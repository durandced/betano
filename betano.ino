// Load Wi-Fi library
#include <esp_timer.h>
#include "BluetoothSerial.h"
#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS

/* Communication with the outside world */
BluetoothSerial ESP_BT;

SFE_UBLOX_GPS myGPS;
long gps_check_time = 0;
char gpsdata[2048] = {0};
long lastTime =  0;

/* Led indicator */
String outputState = "off";
const int output = 13;

/* Timer */
static void gaeta_timer_callback(void* arg)
{
    int64_t time_since_boot = esp_timer_get_time();
    ESP_LOGI(TAG, "One-shot timer called, time since boot: %lld us", time_since_boot);
}


void setup() {
  // Start serual and bluetooth
  Serial.begin(115200);
  ESP_BT.begin("SV-650_GPS");

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz


  if (myGPS.begin(Wire, 0x42) == false) {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring."));
  }
  //This will pipe all NMEA sentences to the serial port so we can see them
  myGPS.setNMEAOutputPort(ESP_BT);
  myGPS.setNavigationFrequency(20); //Set output to 10 times a second
  //myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX sentences (turn off NMEA noise)
  //myGPS.enableDebugging(); //Enable debug messages over Serial (default)

  boolean response = true;
  //These key values are hard coded. You can obtain them from the ZED-F9P interface description doc
  //or from u-center's Messages->CFG->VALSET window. Keys must be 32-bit.
  response &= myGPS.setVal(0x10930006, 0); //Enable high precision NMEA
  response &= myGPS.setVal(0x20110011, 3); //Set 3d fix
  response &= myGPS.setVal(0x20110021, 4); //Set automotive mode
  response &= myGPS.setVal(0x20220005, 3); //Set odometer car mode
  response &= myGPS.setVal(0x20140011, 3); //Set RTK correction mode
  response &= myGPS.setVal(0x30210001, 50); //Set measurement rate to 100ms (10Hz update rate)
  if (response == true)
  {
    Serial.println(F("RTCM messages enabled"));
  }
  else
  {
    Serial.println(F("RTCM failed to enable. Are you sure you have an ZED-F9P? Freezing."));
  }

  byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  Serial.print("Current update rate:");
  Serial.println(rate);
  Serial.print(F("Version: "));
  byte versionHigh = myGPS.getProtocolVersionHigh();
  Serial.print(versionHigh);
  Serial.print(".");
  byte versionLow = myGPS.getProtocolVersionLow();
  Serial.print(versionLow);

  // Power the LED on
  pinMode(output, OUTPUT);
  digitalWrite(output, HIGH);

  // Start the timer
  esp_timer_create_args_t timer_args;
  timer_args.callback = &gaeta_timer_callback;
  timer_args.name = "gaeta";
  esp_timer_handle_t gaeta;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &gaeta));
  ESP_ERROR_CHECK(esp_timer_start_once(gaeta, 5000000));
}

void loop(){
  int64_t current = esp_timer_get_time();
  if (current - gps_check_time > 250){
    myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.
    gps_check_time = current;
    /* ESP_BT.printf("%s", gpsdata); */
    /* gpsdata[0] = 0; */
  }

  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer

    long latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    long accuracy = myGPS.getPositionAccuracy();
    Serial.print(F(" 3D Positional Accuracy: "));
    Serial.print(accuracy);
    Serial.println(F("mm"));
  }

}

//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  ESP_BT.printf("%c", incoming);
}
