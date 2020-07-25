// Load Wi-Fi library
#include <esp_timer.h>
#include "BluetoothSerial.h"
#include <Wire.h> //Needed for I2C to GPS
#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include <SparkFunLSM9DS1.h>
#include "SparkFun_Ublox_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_Ublox_GPS

// Mag address must be 0x1E, would be 0x1C if SDO_M is LOW
#define LSM9DS1_M   0x1E
// Accel/gyro address must be 0x6B, would be 0x6A if SDO_AG is LOW
#define LSM9DS1_AG  0x6B

/* Communication with the outside world */
BluetoothSerial ESP_BT;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

/* Led indicator */
String outputState = "off";
const int output = 13;

/* IMU */
LSM9DS1 imu;  // Create an LSM9DS1 object
float tyre_temp= 0;
float ambient_temp= 0;
int temp_error= 0;

// Global variables to keep track of update rates
unsigned long startTime;
unsigned int accelReadCounter = 0;
unsigned int gyroReadCounter = 0;
unsigned int magReadCounter = 0;
unsigned int tempReadCounter = 0;

void round_i2c() {
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Serial.printf("%i -",address);
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}

void setupGyro()
{
// [enabled] turns the gyro on or off.
  imu.settings.gyro.enabled = true;  // Enable the gyro
// [scale] sets the full-scale range of the gyroscope.
// scale can be set to either 245, 500, or 2000
  imu.settings.gyro.scale = 245; // Set scale to +/-245dps
// [sampleRate] sets the output data rate (ODR) of the gyro
// sampleRate can be set between 1-6
// 1 = 14.9    4 = 238
// 2 = 59.5    5 = 476
// 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 3; // 59.5Hz ODR
// [bandwidth] can set the cutoff frequency of the gyro.
// Allowed values: 0-3. Actual value of cutoff frequency
// depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 0;
// [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
// [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
// [HPFCutoff] sets the HPF cutoff frequency (if enabled)
// Allowable values are 0-9. Value depends on ODR.
// (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz
// [flipX], [flipY], and [flipZ] are booleans that can
// automatically switch the positive/negative orientation
// of the three gyro axes.
  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z
}

void setupAccel()
{
// [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
// [enableX], [enableY], and [enableZ] can turn on or off
// select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z
// [scale] sets the full-scale range of the accelerometer.
// accel scale can be 2, 4, 8, or 16
  imu.settings.accel.scale = 8; // Set accel scale to +/-8g.
// [sampleRate] sets the output data rate (ODR) of the
// accelerometer. ONLY APPLICABLE WHEN THE GYROSCOPE IS
// DISABLED! Otherwise accel sample rate = gyro sample rate.
// accel sample rate can be 1-6
// 1 = 10 Hz    4 = 238 Hz
// 2 = 50 Hz    5 = 476 Hz
// 3 = 119 Hz   6 = 952 Hz
  imu.settings.accel.sampleRate = 1; // Set accel to 10Hz.
// [bandwidth] sets the anti-aliasing filter bandwidth.
// Accel cutoff freqeuncy can be any value between -1 - 3.
// -1 = bandwidth determined by sample rate
// 0 = 408 Hz   2 = 105 Hz
// 1 = 211 Hz   3 = 50 Hz
  imu.settings.accel.bandwidth = 0; // BW = 408Hz
// [highResEnable] enables or disables high resolution
// mode for the acclerometer.
  imu.settings.accel.highResEnable = false; // Disable HR
// [highResBandwidth] sets the LP cutoff frequency of
// the accelerometer if it's in high-res mode.
// can be any value between 0-3
// LP cutoff is set to a factor of sample rate
// 0 = ODR/50    2 = ODR/9
// 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;
}

void setupMag()
{
// [enabled] turns the magnetometer on or off.
  imu.settings.mag.enabled = true; // Enable magnetometer
// [scale] sets the full-scale range of the magnetometer
// mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
// [sampleRate] sets the output data rate (ODR) of the
// magnetometer.
// mag data rate can be 0-7:
// 0 = 0.625 Hz  4 = 10 Hz
// 1 = 1.25 Hz   5 = 20 Hz
// 2 = 2.5 Hz    6 = 40 Hz
// 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 5; // Set OD rate to 20Hz
// [tempCompensationEnable] enables or disables
// temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
// [XYPerformance] sets the x and y-axis performance of the
// magnetometer to either:
// 0 = Low power mode      2 = high performance
// 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
// [ZPerformance] does the same thing, but only for the z
  imu.settings.mag.ZPerformance = 3; // Ultra-high perform.
// [lowPowerEnable] enables or disables low power mode in
// the magnetometer.
  imu.settings.mag.lowPowerEnable = false;
// [operatingMode] sets the operating mode of the
// magnetometer. operatingMode can be 0-2:
// 0 = continuous conversion
// 1 = single-conversion
// 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
}

void setupTemperature()
{
// [enabled] turns the temperature sensor on or off.
  imu.settings.temp.enabled = true;
}

uint16_t initLSM9DS1()
{
  setupGyro(); // Set up gyroscope parameters
  setupAccel(); // Set up accelerometer parameters
  setupMag(); // Set up magnetometer parameters
  setupTemperature(); // Set up temp sensor parameter

  return imu.begin(LSM9DS1_AG, LSM9DS1_M, Wire);
}

/* Timer */
long old_time = 0;
long count=0;
static void gaeta_timer_callback(void* arg)
{
  int64_t time_since_boot = esp_timer_get_time();
  ESP_LOGI(TAG, "One-shot timer called, time since boot: %lld us", time_since_boot);
}

// printSensorReadings prints the latest IMU readings
// along with a calculated update rate.
void printSensorReadings()
{
  float runTime = (float)(millis() - startTime) / 1000.0;
  float accelRate = (float)accelReadCounter / runTime;
  float gyroRate = (float)gyroReadCounter / runTime;
  float magRate = (float)magReadCounter / runTime;
  float tempRate = (float)tempReadCounter / runTime;
  Serial.print("A: ");
  Serial.print(imu.calcAccel(imu.ax));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay));
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az));
  Serial.print(" g \t| ");
  Serial.print(accelRate);
  Serial.println(" Hz");
  Serial.print("G: ");
  Serial.print(imu.calcGyro(imu.gx));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy));
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz));
  Serial.print(" dps \t| ");
  Serial.print(gyroRate);
  Serial.println(" Hz");
  Serial.print("M: ");
  Serial.print(imu.calcMag(imu.mx));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my));
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz));
  Serial.print(" Gs \t| ");
  Serial.print(magRate);
  Serial.println(" Hz");
  Serial.print("T: ");
  Serial.print(imu.temperature);
  Serial.print(" \t\t\t| ");
  Serial.print(tempRate);
  Serial.println(" Hz");
  Serial.println();
}

String checksum(String s)   // Checksum calculation
{
  byte cs=0x00;
  char *buf = (char*)s.c_str();
  for (unsigned int i = 1; i < s.length()-1; i++) cs = cs ^ buf[i];   // XOR
  return String((cs <= 0x0f ? "0" : "") + String(cs, HEX));
}

void setup() {
  // Start serual and bluetooth
  Serial.begin(115200);
  ESP_BT.begin("SV-650_BTelemetry");
  Wire.begin();
  Wire.setClock(100000); //Increase I2C clock speed to 400kHz

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
  round_i2c();
  Serial.println("Initializing the LSM9DS1");
  uint16_t status = initLSM9DS1();
  Serial.print("LSM9DS1 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x683D");
  Serial.println();
}

void loop(){
  if (imu.accelAvailable())
  {
    imu.readAccel();
    accelReadCounter++;
  }
  // imu.gyroAvailable() returns 1 if new gyroscope
  // data is ready to be read. 0 otherwise.
  if (imu.gyroAvailable())
  {
    imu.readGyro();
    gyroReadCounter++;
  }
  // imu.magAvailable() returns 1 if new magnetometer
  // data is ready to be read. 0 otherwise.
  if (imu.magAvailable())
  {
    imu.readMag();
    magReadCounter++;
  }
  // imu.tempAvailable() returns 1 if new temperature sensor
  // data is ready to be read. 0 otherwise.
  if (imu.tempAvailable())
  {
    imu.readTemp();
    tempReadCounter++;
  }

  int64_t current = esp_timer_get_time();
  if (current - old_time > 100000) {
    //$RC3,[time],[count],[xacc],[yacc],[zacc],[gyrox],[gyroy],[gyroz],[rpm/d1],[d2],[a1],[a2],[a3],[a4],[a5],[a6],[a7],[a8],[a9],[a10],[a11],[a12],[a13],[a14],[a15]*[checksum]
    float t_temp = mlx.readObjectTempC();
    if (t_temp > 0 && t_temp < 150) {
      ambient_temp = mlx.readAmbientTempC();
      tyre_temp = t_temp;
      temp_error = 0;
    }
    else
    {
      temp_error = 1;
    }

    String str = String("$RC3,,") + String(count++) + ",";
    //str += String(tempTC,1) + ",";   //[a2],   (thermocouple)
    str+= String(imu.calcAccel(imu.ax)) + ",";
    str+= String(imu.calcAccel(imu.ay)) + ",";
    str+= String(imu.calcAccel(imu.az)) + ",";
    str+= String(imu.calcGyro(imu.gx)) + ",";
    str+= String(imu.calcGyro(imu.gy)) + ",";
    str+= String(imu.calcGyro(imu.gz)) + ",,,"; // gyroz,d1,d2
    str+= String(tyre_temp) + ","; // a1
    str+= String(ambient_temp) + ","; // a2
    str+= String(imu.calcMag(imu.mx)) + ","; // a3
    str+= String(imu.calcMag(imu.my)) + ","; // a4
    str+= String(imu.calcMag(imu.mz)) + ","; // a5
    str+= ","; // a6
    str+= ","; //a7
    str+= ","; //a8
    str+= ","; //a9
    str+= String(temp_error) + ","; //a10
    str+= ","; //a11
    str+= ","; //a12
    str+= ","; //a13
    str+= ","; //a14
    str+= "*"; //a15
    Serial.println(str + checksum(str));
    ESP_BT.println(str + checksum(str));

    old_time = current;
  }
}
