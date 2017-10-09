#include <Wire.h>
#include "BMP085.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

HMC5883L compass;
BMP085 pressure_m;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
#define LED_PIN 13
bool blinkState = false;
double dt = 0;
long counter = 0;
double t0 = -1;
double millis_t = 0;

void setup(){
    //Включаем последовательный порт на максимальную скорость
    Serial.begin(230400);
    //Включаем протокол Wire
    Wire.begin();
    TWBR = 24;
    Serial.println("Initializing I2C devices...");    
    //инициализируем акселерометр с гироскопом
    accelgyro.initialize();
    accelgyro.setMasterClockSpeed(13);
    accelgyro.setI2CMasterModeEnabled(true);
    //инициализируем компас
    compass = HMC5883L();
    setupHMC5883L(); 

    //проверяем содеинение
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    //калибруем датчик давления
    pressure_m.bmp085Calibration();
}

void loop()
{
  millis_t = millis();
  //Считаем температуру (её знает датчик давления)
  float temperature = pressure_m.bmp085GetTemperature(); //MUST be called first
  //Считаем давление
  float pressure = pressure_m.bmp085GetPressure();
  //Вычислим высоту над уровнем моря
  float altitude = pressure_m.calcAltitude(pressure);

  //Вычислим dt (интервал между замерами в миллисекундах)
  if (t0 < 0)
  {
    t0 = millis_t;
  }
  else
  {
    dt = millis_t - t0;
    t0 = millis_t;
  }

  //Интервал между замерами
  Serial.print("dt:"); Serial.print(dt / 1000, 3);
  //Сколько времени прошло с запуска?
  Serial.print(" tm:"); Serial.print(millis_t);
  //Температура, в градусах цельсия
  Serial.print(" t:"); Serial.print(temperature, 2); 
  //Давление, в паскалях
  Serial.print(" p:"); Serial.print(pressure, 0); 
  //Высота над уровнем моря, в метрах
  Serial.print(" alt:"); Serial.print(altitude, 2); 
  
  //Считаем значения с магнитометра
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  //Считаем значения с акселерометра и гироскопа
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //Данные акселерометра:
  Serial.print(" ax:"); Serial.print(ax);
  Serial.print(" ay:"); Serial.print(ay);
  Serial.print(" az:"); Serial.print(az);
  //Данные гироскопа
  Serial.print(" wx:"); Serial.print(gx);
  Serial.print(" wy:"); Serial.print(gy);
  Serial.print(" wz:"); Serial.print(gz);
  //Данные магнитометра    
  Serial.print(" cx:"); Serial.print(scaled.XAxis, 3);
  Serial.print(" cy:"); Serial.print(scaled.YAxis, 3);
  Serial.print(" cz:"); Serial.print(scaled.ZAxis, 3);
  Serial.println();
}

//Настройка модуля HMC5883L (компаса)
void setupHMC5883L(){
  //Setup the HMC5883L, and check for errors
  int error;  
  error = compass.SetScale(1.3); //Set the scale of the compass.
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so

  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) Serial.println(compass.GetErrorText(error)); //check if there is an error, and print if so
}
