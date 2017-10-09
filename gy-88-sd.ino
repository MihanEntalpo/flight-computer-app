#include <Wire.h>
#include "BMP085.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include <SPI.h>
#include <Wire.h>
#include "BMP085.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include <SPI.h>
#include <SD.h>

#define TRUE 1
#define FALSE 0

File myFile;

HMC5883L compass;
BMP085 pressure_m;
MPU6050 accelgyro;

#define EXEC_PIN 3
#define RESET_ALT_PIN 6
#define ARMED_PIN 9

int16_t ax, ay, az;
int16_t gx, gy, gz;
bool blinkState = false;
double dt = 0;
long counter = 0;
double t0 = -1;
double millis_t = 0;
double io_t = 0;
double io_dt = 40;
double fired_millis_t = 0;

#define FIRE_TIME 4000

float base_altitude = 0;

#define RUN_ALT_NUM 16

float run_alt[RUN_ALT_NUM];
int run_alt_index = 0;
int last_run_alt_index = 0;
int run_alt_count = 0;
float run_alt_summ = 0;

bool is_prepared = FALSE;
bool is_armed = FALSE;
bool is_fired = FALSE;
bool is_led_armed = FALSE;
bool is_led_fired = FALSE;
bool is_done = FALSE;

float arming_altitude = 10;
float fire_minus_altitude = 1;
float max_altitude = 0;

char filename[] = "GY88_000.TXT";

void reset_running_alt();
void reset_alt(float current_alt);

void generateFileName()
{
  for (int i = 0; i< 1000; i++)
  {
    filename[5] = i/100 + '0';
    filename[6] = (i%100)/10 + '0';
    filename[7] = (i%10) + '0';
    if (SD.exists(filename)) continue;
    Serial.print("File: ");
    Serial.println(filename);
    break;  
  }
}

float debug_get_alt_summ()
{
  float summ = 0;
  for (int i=0; i< run_alt_count; i++)
  {
    summ += run_alt[i];
  }
  return summ;
}

void setup(){
    //Настраиваем режимы работы для цифровых контактов
    pinMode(EXEC_PIN, OUTPUT);
    pinMode(RESET_ALT_PIN, INPUT);
    pinMode(ARMED_PIN, OUTPUT);
    //Сбрасываем значения для скользящего среднего по высоте
    reset_running_alt();
    
    //Включаем последовательный порт на максимальную скорость
    Serial.begin(230400);
    //while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    //}    

    Serial.print("Initializing SD card...");
  
    if (!SD.begin(4)) {
      Serial.println("initialization failed!");
      return;
    }
    Serial.println("initialization done.");

    generateFileName();
    myFile = SD.open(filename, O_WRITE | O_CREAT);
    
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
  float altitude_b = altitude - base_altitude;
  
  //Проведём вычисления для скользящего среднего по высоте
  run_alt[run_alt_index] = altitude_b;
  //run_alt_summ += altitude_b;
  if (run_alt_count == RUN_ALT_NUM)
  {
    last_run_alt_index = (run_alt_index == 0) ? RUN_ALT_NUM - 1 : run_alt_index - 1;
    //run_alt_summ -= run_alt[last_run_alt_index];
  }
  else
  {
    run_alt_count +=1;
  }
  run_alt_index = (run_alt_index + 1 == RUN_ALT_NUM) ? 0 : run_alt_index + 1;

  //Временно вычисляем медленным методом
  run_alt_summ = debug_get_alt_summ();

  float run_altitude = run_alt_summ / (float) run_alt_count;

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

  if (millis_t > 10000)
  {
    //digitalWrite(EXEC_PIN, HIGH);
  }

  if (run_altitude > max_altitude)
  {
    max_altitude = run_altitude;
  }

  if (millis_t - io_t > io_dt)
  {
    io_t = millis_t;
    if (digitalRead(RESET_ALT_PIN) == HIGH)
    {
      reset_alt(altitude);
    }

    if (!is_done)
    {
      if (!is_fired)
      {
        if (is_prepared)
        {
          if (!is_armed)
          {
            if (max_altitude > arming_altitude)
            {
              is_armed = TRUE;
            }
          }
          else
          {
            if (max_altitude - run_altitude > fire_minus_altitude)
            {
              is_armed = FALSE;
              is_fired = TRUE;
              fired_millis_t = millis_t;
            }
          }
        }
      }
      else
      {
        if (millis_t - fired_millis_t > FIRE_TIME)
        {
          is_fired = FALSE;
          is_done = TRUE;
          is_prepared = FALSE;
        }
      }
    }

    if (is_armed != is_led_armed)
    {
      is_led_armed = is_armed;
      digitalWrite(ARMED_PIN, is_armed ? HIGH : LOW);  
    }

    if (is_fired != is_led_fired)
    {
      is_led_fired = is_fired;
      digitalWrite(EXEC_PIN, is_fired ? HIGH : LOW);
    }
    
  }

  
  //Интервал между замерами
  Serial.print("dt:"); Serial.print(dt / 1000, 3);
  myFile.print("dt:"); myFile.print(dt / 1000, 3);
  //Сколько времени прошло с запуска?
  Serial.print(" tm:"); Serial.print(millis_t);
  myFile.print(" tm:"); myFile.print(millis_t);
  //Температура, в градусах цельсия
  Serial.print(" t:"); Serial.print(temperature, 2); 
  myFile.print(" t:"); myFile.print(temperature, 2); 
  //Давление, в паскалях
  Serial.print(" p:"); Serial.print(pressure, 0); 
  myFile.print(" p:"); myFile.print(pressure, 0); 
  //Высота над уровнем моря, в метрах
  Serial.print(" alt:"); Serial.print(altitude, 2); 
  myFile.print(" alt:"); myFile.print(altitude, 2); 
  //Высота над базовым уровнем, в метрах
  Serial.print(" altb:"); Serial.print(altitude_b, 2); 
  myFile.print(" altb:"); myFile.print(altitude_b, 2);
  //Высота над базовым уровнем, в метрах, скользящее среднее
  Serial.print(" altr:"); Serial.print(run_altitude, 2); 
  myFile.print(" altr:"); myFile.print(run_altitude, 2);
  //Высота над базовым уровнем, в метрах, максимальная
  Serial.print(" Malt:"); Serial.print(max_altitude, 2); 
  myFile.print(" Malt:"); myFile.print(max_altitude, 2);

  //Serial.print(" raltS:"); Serial.print(run_alt_summ, 2); 
  //myFile.print(" raltS:"); myFile.print(run_alt_summ, 2);

  //float debug_run_alt_summ = debug_get_alt_summ();

  //Serial.print(" dRaltS:"); Serial.print(debug_run_alt_summ, 2); 
  //myFile.print(" dRaltS:"); myFile.print(debug_run_alt_summ, 2);

  //Serial.print(" daltR:"); Serial.print(debug_run_alt_summ/(float)run_alt_count, 2); 
  //myFile.print(" daltR:"); myFile.print(debug_run_alt_summ/(float)run_alt_count, 2);

  //Статусы:
  Serial.print(" P"); Serial.print(is_prepared ? "1" : "0"); 
  myFile.print(" P"); myFile.print(is_prepared ? "1" : "0");
  Serial.print(" A"); Serial.print(is_armed ? "1" : "0"); 
  myFile.print(" A"); myFile.print(is_armed ? "1" : "0");
  Serial.print(" F"); Serial.print(is_fired ? "1" : "0"); 
  myFile.print(" F"); myFile.print(is_fired ? "1" : "0");
  Serial.print(" D"); Serial.print(is_done ? "1" : "0"); 
  myFile.print(" D"); myFile.print(is_done ? "1" : "0");

  //Serial.print(" rai"); Serial.print(run_alt_index); 
  //myFile.print(" rai"); myFile.print(run_alt_index);

  //Serial.print(" lrai"); Serial.print(last_run_alt_index); 
  //myFile.print(" lrai"); myFile.print(last_run_alt_index);


  //Считаем значения с магнитометра
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  //Считаем значения с акселерометра и гироскопа
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //Данные акселерометра:
  Serial.print(" ax:"); Serial.print(ax);
  Serial.print(" ay:"); Serial.print(ay);
  Serial.print(" az:"); Serial.print(az);
  myFile.print(" ax:"); myFile.print(ax);
  myFile.print(" ay:"); myFile.print(ay);
  myFile.print(" az:"); myFile.print(az);
  //Данные гироскопа
  Serial.print(" wx:"); Serial.print(gx);
  Serial.print(" wy:"); Serial.print(gy);
  Serial.print(" wz:"); Serial.print(gz);
  myFile.print(" wx:"); myFile.print(gx);
  myFile.print(" wy:"); myFile.print(gy);
  myFile.print(" wz:"); myFile.print(gz);
  //Данные магнитометра    
  Serial.print(" cx:"); Serial.print(scaled.XAxis, 3);
  Serial.print(" cy:"); Serial.print(scaled.YAxis, 3);
  Serial.print(" cz:"); Serial.print(scaled.ZAxis, 3);
  Serial.println();
  myFile.print(" cx:"); myFile.print(scaled.XAxis, 3);
  myFile.print(" cy:"); myFile.print(scaled.YAxis, 3);
  myFile.print(" cz:"); myFile.print(scaled.ZAxis, 3);
  myFile.println();
  
  myFile.flush();
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

/**
 * Сброс высоты и перевод в состояние готовности - выполняется по нажатию кнопки
 */
void reset_alt(float current_alt)
{
  base_altitude = current_alt;
  is_prepared = TRUE;
  is_armed = FALSE;
  max_altitude = 0;  
  reset_running_alt();
}

/**
 * Сброс скользящего среднего для высоты
 */
void reset_running_alt()
{
  run_alt_index = 0;
  run_alt_count = 0;
  run_alt_summ = 0;
  last_run_alt_index = 0;
  for (int i =0; i<RUN_ALT_NUM; i++)
    {
      run_alt[i]=0;
    }
}


#define TRUE 1
#define FALSE 0

File myFile;

HMC5883L compass;
BMP085 pressure_m;
MPU6050 accelgyro;

#define EXEC_PIN 3
#define RESET_ALT_PIN 6
#define ARMED_PIN 9

int16_t ax, ay, az;
int16_t gx, gy, gz;
bool blinkState = false;
double dt = 0;
long counter = 0;
double t0 = -1;
double millis_t = 0;
double io_t = 0;
double io_dt = 40;
double fired_millis_t = 0;

#define FIRE_TIME 4000

float base_altitude = 0;

#define RUN_ALT_NUM 16

float run_alt[RUN_ALT_NUM];
int run_alt_index = 0;
int last_run_alt_index = 0;
int run_alt_count = 0;
float run_alt_summ = 0;

bool is_prepared = FALSE;
bool is_armed = FALSE;
bool is_fired = FALSE;
bool is_led_armed = FALSE;
bool is_led_fired = FALSE;
bool is_done = FALSE;

float arming_altitude = 10;
float fire_minus_altitude = 1;
float max_altitude = 0;

char filename[] = "GY88_000.TXT";

void reset_running_alt();
void reset_alt(float current_alt);

void generateFileName()
{
  for (int i = 0; i< 1000; i++)
  {
    filename[5] = i/100 + '0';
    filename[6] = (i%100)/10 + '0';
    filename[7] = (i%10) + '0';
    if (SD.exists(filename)) continue;
    Serial.print("File: ");
    Serial.println(filename);
    break;  
  }
}

float debug_get_alt_summ()
{
  float summ = 0;
  for (int i=0; i< run_alt_count; i++)
  {
    summ += run_alt[i];
  }
  return summ;
}

void setup(){
    //Настраиваем режимы работы для цифровых контактов
    pinMode(EXEC_PIN, OUTPUT);
    pinMode(RESET_ALT_PIN, INPUT);
    pinMode(ARMED_PIN, OUTPUT);
    //Сбрасываем значения для скользящего среднего по высоте
    reset_running_alt();
    
    //Включаем последовательный порт на максимальную скорость
    Serial.begin(230400);
    //while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    //}    

    Serial.print("Initializing SD card...");
  
    if (!SD.begin(4)) {
      Serial.println("initialization failed!");
      return;
    }
    Serial.println("initialization done.");

    generateFileName();
    myFile = SD.open(filename, O_WRITE | O_CREAT);
    
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
  float altitude_b = altitude - base_altitude;
  
  //Проведём вычисления для скользящего среднего по высоте
  run_alt[run_alt_index] = altitude_b;
  //run_alt_summ += altitude_b;
  if (run_alt_count == RUN_ALT_NUM)
  {
    last_run_alt_index = (run_alt_index == 0) ? RUN_ALT_NUM - 1 : run_alt_index - 1;
    //run_alt_summ -= run_alt[last_run_alt_index];
  }
  else
  {
    run_alt_count +=1;
  }
  run_alt_index = (run_alt_index + 1 == RUN_ALT_NUM) ? 0 : run_alt_index + 1;

  //Временно вычисляем медленным методом
  run_alt_summ = debug_get_alt_summ();

  float run_altitude = run_alt_summ / (float) run_alt_count;

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

  if (millis_t > 10000)
  {
    //digitalWrite(EXEC_PIN, HIGH);
  }

  if (run_altitude > max_altitude)
  {
    max_altitude = run_altitude;
  }

  if (millis_t - io_t > io_dt)
  {
    io_t = millis_t;
    if (digitalRead(RESET_ALT_PIN) == HIGH)
    {
      reset_alt(altitude);
    }

    if (!is_done)
    {
      if (!is_fired)
      {
        if (is_prepared)
        {
          if (!is_armed)
          {
            if (max_altitude > arming_altitude)
            {
              is_armed = TRUE;
            }
          }
          else
          {
            if (max_altitude - run_altitude > fire_minus_altitude)
            {
              is_armed = FALSE;
              is_fired = TRUE;
              fired_millis_t = millis_t;
            }
          }
        }
      }
      else
      {
        if (millis_t - fired_millis_t > FIRE_TIME)
        {
          is_fired = FALSE;
          is_done = TRUE;
          is_prepared = FALSE;
        }
      }
    }

    if (is_armed != is_led_armed)
    {
      is_led_armed = is_armed;
      digitalWrite(ARMED_PIN, is_armed ? HIGH : LOW);  
    }

    if (is_fired != is_led_fired)
    {
      is_led_fired = is_fired;
      digitalWrite(EXEC_PIN, is_fired ? HIGH : LOW);
    }
    
  }

  
  //Интервал между замерами
  Serial.print("dt:"); Serial.print(dt / 1000, 3);
  myFile.print("dt:"); myFile.print(dt / 1000, 3);
  //Сколько времени прошло с запуска?
  Serial.print(" tm:"); Serial.print(millis_t);
  myFile.print(" tm:"); myFile.print(millis_t);
  //Температура, в градусах цельсия
  Serial.print(" t:"); Serial.print(temperature, 2); 
  myFile.print(" t:"); myFile.print(temperature, 2); 
  //Давление, в паскалях
  Serial.print(" p:"); Serial.print(pressure, 0); 
  myFile.print(" p:"); myFile.print(pressure, 0); 
  //Высота над уровнем моря, в метрах
  Serial.print(" alt:"); Serial.print(altitude, 2); 
  myFile.print(" alt:"); myFile.print(altitude, 2); 
  //Высота над базовым уровнем, в метрах
  Serial.print(" altb:"); Serial.print(altitude_b, 2); 
  myFile.print(" altb:"); myFile.print(altitude_b, 2);
  //Высота над базовым уровнем, в метрах, скользящее среднее
  Serial.print(" altr:"); Serial.print(run_altitude, 2); 
  myFile.print(" altr:"); myFile.print(run_altitude, 2);
  //Высота над базовым уровнем, в метрах, максимальная
  Serial.print(" Malt:"); Serial.print(max_altitude, 2); 
  myFile.print(" Malt:"); myFile.print(max_altitude, 2);

  //Serial.print(" raltS:"); Serial.print(run_alt_summ, 2); 
  //myFile.print(" raltS:"); myFile.print(run_alt_summ, 2);

  //float debug_run_alt_summ = debug_get_alt_summ();

  //Serial.print(" dRaltS:"); Serial.print(debug_run_alt_summ, 2); 
  //myFile.print(" dRaltS:"); myFile.print(debug_run_alt_summ, 2);

  //Serial.print(" daltR:"); Serial.print(debug_run_alt_summ/(float)run_alt_count, 2); 
  //myFile.print(" daltR:"); myFile.print(debug_run_alt_summ/(float)run_alt_count, 2);

  //Статусы:
  Serial.print(" P"); Serial.print(is_prepared ? "1" : "0"); 
  myFile.print(" P"); myFile.print(is_prepared ? "1" : "0");
  Serial.print(" A"); Serial.print(is_armed ? "1" : "0"); 
  myFile.print(" A"); myFile.print(is_armed ? "1" : "0");
  Serial.print(" F"); Serial.print(is_fired ? "1" : "0"); 
  myFile.print(" F"); myFile.print(is_fired ? "1" : "0");
  Serial.print(" D"); Serial.print(is_done ? "1" : "0"); 
  myFile.print(" D"); myFile.print(is_done ? "1" : "0");

  //Serial.print(" rai"); Serial.print(run_alt_index); 
  //myFile.print(" rai"); myFile.print(run_alt_index);

  //Serial.print(" lrai"); Serial.print(last_run_alt_index); 
  //myFile.print(" lrai"); myFile.print(last_run_alt_index);


  //Считаем значения с магнитометра
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  //Считаем значения с акселерометра и гироскопа
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  //Данные акселерометра:
  Serial.print(" ax:"); Serial.print(ax);
  Serial.print(" ay:"); Serial.print(ay);
  Serial.print(" az:"); Serial.print(az);
  myFile.print(" ax:"); myFile.print(ax);
  myFile.print(" ay:"); myFile.print(ay);
  myFile.print(" az:"); myFile.print(az);
  //Данные гироскопа
  Serial.print(" wx:"); Serial.print(gx);
  Serial.print(" wy:"); Serial.print(gy);
  Serial.print(" wz:"); Serial.print(gz);
  myFile.print(" wx:"); myFile.print(gx);
  myFile.print(" wy:"); myFile.print(gy);
  myFile.print(" wz:"); myFile.print(gz);
  //Данные магнитометра    
  Serial.print(" cx:"); Serial.print(scaled.XAxis, 3);
  Serial.print(" cy:"); Serial.print(scaled.YAxis, 3);
  Serial.print(" cz:"); Serial.print(scaled.ZAxis, 3);
  Serial.println();
  myFile.print(" cx:"); myFile.print(scaled.XAxis, 3);
  myFile.print(" cy:"); myFile.print(scaled.YAxis, 3);
  myFile.print(" cz:"); myFile.print(scaled.ZAxis, 3);
  myFile.println();
  
  myFile.flush();
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

/**
 * Сброс высоты и перевод в состояние готовности - выполняется по нажатию кнопки
 */
void reset_alt(float current_alt)
{
  base_altitude = current_alt;
  is_prepared = TRUE;
  is_armed = FALSE;
  max_altitude = 0;  
  reset_running_alt();
}

/**
 * Сброс скользящего среднего для высоты
 */
void reset_running_alt()
{
  run_alt_index = 0;
  run_alt_count = 0;
  run_alt_summ = 0;
  last_run_alt_index = 0;
  for (int i =0; i<RUN_ALT_NUM; i++)
    {
      run_alt[i]=0;
    }
}

