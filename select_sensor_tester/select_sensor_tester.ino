#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Timer.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define DEBUG_PRINT_(x) Serial.print(x)
//#define DEBUG_PRINT_(x)

#define LED_PIN 2
#define BUTTON_PIN 4
#define PIN_S1 7
#define PIN_S2 8
#define sampFreq 100
/*
   Linha alterada em i2cdevlib (Replace your file by the file in the folder of this sketch):
    DEBUG_PRINTLN(F("Setting sample rate to 100Hz..."));
    setRate(9); // 1khz / (1 + 9) = 100 Hz
*/
#define PSDMP 42
#define ST '$'
#define ET '\n'

MPU6050 mpu1(0x68);
MPU6050 mpu2(0x68);

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  Serial.begin(115200);
  
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(200000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  DEBUG_PRINT_("Aguardando sinal do botao.\n");
}

void loop() {
 if (digitalRead(BUTTON_PIN)) {
    delay(100);
    if (digitalRead(BUTTON_PIN)) {
      DEBUG_PRINT_("Aguardando liberacao do botao.\n");
      while (digitalRead(BUTTON_PIN)); //Espera soltar o botao
      DEBUG_PRINT_("Liberado.\n\n");

      
    }
 }
}
