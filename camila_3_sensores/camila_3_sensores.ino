/* ------------------------------------------------------------------------------
   FEDERAL UNIVERSITY OF UBERLANDIA
   Faculty of Electrical Engineering
   Biomedical Engineering Lab
   Uberlândia, Brazil
   ------------------------------------------------------------------------------
   Programa: Análise de marcha
   Lê acelerômetro, giroscópio e o quartenion, os salva em um cartão sd simultaneamente envia vio bluetooth
   Esse sistema deve ser colocado sobre o pé para se realizar a análise da marcha
   Autor: Ana

*/
/* ================================================================================================ *
  | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
  |                                                                                                  |
  | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][g[1]RO X][      ][g[1]RO Y][      ] |
  |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
  |                                                                                                  |
  | [gyRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
  |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
   ================================================================================================ */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Timer.h"
#include "SoftwareSerial.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif


#define DEBUG_PRINT_(x) Serial.print(x)
//#define DEBUG_PRINT_(x)

#define LED_PIN 2
#define BUTTON_PIN 4
#define PIN_S1 5
#define PIN_S2 6
#define PIN_S3 7
#define PIN_S4 8
#define PIN_S5 9
#define sampFreq 100
/*
   Linha alterada em i2cdevlib (Replace your file by the file in the folder of this sketch):
    DEBUG_PRINTLN(F("Setting sample rate to 100Hz..."));
    setRate(9); // 1khz / (1 + 9) = 100 Hz
*/
#define PSDMP 42
#define ST '$'
#define ET '\n'

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x68);
SoftwareSerial BTSerial(10, 11); // RX | TX

Timer t;
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//  VARIABLES
//------------------------------------------------------------------------------
//Data acquisition variables
//Sampling period in us
const double sampPeriod = (1.0 / sampFreq) * 1000000;
//Serial variables
String serialOp; // Variable for receiving commands from serial
//DMP variables
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[42]; // FIFO storage fifoBuffer
int numbPackets;
// orientation/motion vars
float q[4]; // [w, x, y, z] quaternion container
float a[3]; // [x, y, z] acceleration container
float g[3]; // [x, y, z] gyroscope container

bool is_alive = false;
int timer_id;

bool running_coleta = false;
bool led_state = LOW;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_S4, OUTPUT);
  pinMode(PIN_S5, OUTPUT);

  Serial.begin(115200);
  BTSerial.begin(9600);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(200000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  inicializar_sensores();
  DEBUG_PRINT_("Aguardando sinal do botao.\n");
}

void loop()
{
  t.update();
  if (digitalRead(BUTTON_PIN)) {
    delay(100);
    if (digitalRead(BUTTON_PIN)) {
      DEBUG_PRINT_("Aguardando liberacao do botao.\n");
      while (digitalRead(BUTTON_PIN)); //Espera soltar o botao
      DEBUG_PRINT_("Liberado.\n\n");

      if (running_coleta) {
        // finalizes the aquisition
        // Interrompe o processo e fecha o arquivo
        DEBUG_PRINT_("Aquisicao finalizada.\n");
        t.stop(timer_id);
        digitalWrite(LED_PIN, LOW);
      } else {
        //starts the aquisition
        DEBUG_PRINT_("Aquisicao iniciada.\n");
        select_sensor(1);
        DEBUG_PRINT_("Testando conexao com o sensor - " + String(mpu1.testConnection()) + "\n");
        mpu1.resetFIFO();
        select_sensor(2);
        DEBUG_PRINT_("Testando conexao com o sensor - " + String(mpu2.testConnection()) + "\n");
        mpu2.resetFIFO();
        delay(5);
        digitalWrite(LED_PIN, HIGH);
        timer_id = t.every(sampPeriod / 1000, takeReading);
      }
      running_coleta = !running_coleta;

    }
  }

}
/**
    Methodo para leitura de dados, escrita e envio. É chamada pelo timer.
*/
void takeReading() {
  BTSerial.write(ST); //byte Start Transmission

  ler_sensor_1();  //Le sensor
  send_packet_via_bt();  //enviar dados via bluetooth
  DEBUG_PRINT_("1:\t");
  mostrar_dados();

  ler_sensor_2();  //Le sensor
  send_packet_via_bt(); //enviar dados via bluetooth
  DEBUG_PRINT_("2:\t");
  mostrar_dados();

  DEBUG_PRINT_("\n");
  BTSerial.write(ET); //byte End Transmission
  digitalWrite(LED_PIN, led_state);
  led_state = !led_state;
}

void inicializar_sensores() {
  //////////////
  //MPU WRIST //
  //////////////
  select_sensor(1);
  //Iniciando o sensor
  DEBUG_PRINT_("Testando o sensor.....\n");
  mpu1.initialize();
  if (mpu1.testConnection()) {
    DEBUG_PRINT_("Iniciando o sensor.....\n");
    //Initializes the IMU
    mpu1.initialize();
    //Initializes the DMP
    uint8_t ret = mpu1.dmpInitialize();
    delay(50);
    if (ret == 0) {
      mpu1.setDMPEnabled(true);
      mpu1.setXAccelOffset(-5282);
      mpu1.setYAccelOffset(718);
      mpu1.setZAccelOffset(1297);
      mpu1.setXGyroOffset(78);
      mpu1.setYGyroOffset(-54);
      mpu1.setZGyroOffset(36);
      DEBUG_PRINT_("Sensor Iniciado.\n");
      DEBUG_PRINT_("Testando conexao - " + String(mpu1.testConnection()) + "\n");
    }
    else
    {
      DEBUG_PRINT_("Erro na inicializacao do sensor !\n");
    }
  } else {
    DEBUG_PRINT_("Erro.....\n");
  }

  //////////////
  //MPU ELBOW //
  //////////////
  select_sensor(2);
  //Iniciando o sensor
  DEBUG_PRINT_("Testando o sensor.....\n");
  mpu2.initialize();
  if (mpu2.testConnection()) {
    DEBUG_PRINT_("Iniciando o sensor.....\n");
    //Initializes the IMU
    mpu2.initialize();
    //Initializes the DMP
    uint8_t ret = mpu2.dmpInitialize();
    delay(50);
    if (ret == 0) {
      mpu2.setDMPEnabled(true); /*Not Calibrated yet*/

      mpu2.setXAccelOffset(2370);
      mpu2.setYAccelOffset(-4200);
      mpu2.setZAccelOffset(1130);
      mpu2.setXGyroOffset(-52);
      mpu2.setYGyroOffset(15);
      mpu2.setZGyroOffset(26);
      DEBUG_PRINT_("Sensor Iniciado.\n");
      DEBUG_PRINT_("Testando conexao - " + String(mpu2.testConnection()) + "\n");
    }
    else
    {
      DEBUG_PRINT_("Erro na inicializacao do sensor !\n");
    }
  } else {
    DEBUG_PRINT_("Erro.....\n");
  }
}



void ler_sensor_1() {
  select_sensor(1);
  numbPackets = floor(mpu1.getFIFOCount() / PSDMP);
  DEBUG_PRINT_(numbPackets); DEBUG_PRINT_(" - ");
  if (numbPackets >= 24) {
    mpu1.resetFIFO();
    DEBUG_PRINT_("FIFO sensor 0x68 overflow!\n");
  }
  for (int i = 0; i < numbPackets; i++) {
    mpu1.getFIFOBytes(fifoBuffer, PSDMP);
  }
  numbPackets = 0;
}

void ler_sensor_2() {
  select_sensor(2);
  numbPackets = floor(mpu2.getFIFOCount() / PSDMP);
  DEBUG_PRINT_(numbPackets); DEBUG_PRINT_(" - ");
  if (numbPackets >= 24) {
    mpu2.resetFIFO();
    DEBUG_PRINT_("FIFO sensor 0x69 overflow!\n");
  }
  for (int i = 0; i < numbPackets; i++) {
    mpu2.getFIFOBytes(fifoBuffer, PSDMP);
  }
  numbPackets = 0;
}

void mostrar_dados() {
  //Quaternion
  q[0] = (float) ((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0f;
  q[1] = (float) ((fifoBuffer[4] << 8) | fifoBuffer[5]) / 16384.0f;
  q[2] = (float) ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0f;
  q[3] = (float) ((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0f;

  //Aceleracao
  a[0] = (float) ((fifoBuffer[28] << 8) | fifoBuffer[29]) / 8192.0f;
  a[1] = (float) ((fifoBuffer[32] << 8) | fifoBuffer[33]) / 8192.0f;
  a[2] = (float) ((fifoBuffer[36] << 8) | fifoBuffer[37]) / 8192.0f;

  //Giroscopio
  g[0] = (float) ((fifoBuffer[16] << 8) | fifoBuffer[17]) / 131.0f;
  g[1] = (float) ((fifoBuffer[20] << 8) | fifoBuffer[21]) / 131.0f;
  g[2] = (float) ((fifoBuffer[24] << 8) | fifoBuffer[25]) / 131.0f;
  //Quaternions
  DEBUG_PRINT_(q[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[2]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[3]);
  DEBUG_PRINT_("\t-\t");
  //accel in G
  DEBUG_PRINT_(a[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(a[2]);
  DEBUG_PRINT_("\t-\t");
  //g[1]ro in degrees/s
  DEBUG_PRINT_(g[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(g[2]);
  DEBUG_PRINT_("\t");
}

void send_packet_via_bt() {
  //Assembling packet and sending
  BTSerial.write(fifoBuffer[0]); //qw_msb
  BTSerial.write(fifoBuffer[1]); //qw_lsb
  BTSerial.write(fifoBuffer[4]); //qx_msb
  BTSerial.write(fifoBuffer[5]); //qx_lsb
  BTSerial.write(fifoBuffer[8]); //qy_msb
  BTSerial.write(fifoBuffer[9]); //qy_lsb
  BTSerial.write(fifoBuffer[12]); //qz_msb
  BTSerial.write(fifoBuffer[13]); //qz_lsb

  /*
    BTSerial.write(fifoBuffer[28]); //ax_msb
    BTSerial.write(fifoBuffer[29]); //ax_lsb
    BTSerial.write(fifoBuffer[32]); //ay_msb
    BTSerial.write(fifoBuffer[33]); //ay_lsb
    BTSerial.write(fifoBuffer[36]); //az_msb
    BTSerial.write(fifoBuffer[37]); //az_lsb

    BTSerial.write(fifoBuffer[16]); //gx_msb
    BTSerial.write(fifoBuffer[17]); //gx_lsb
    BTSerial.write(fifoBuffer[20]); //gy_msb
    BTSerial.write(fifoBuffer[21]); //gy_lsb
    BTSerial.write(fifoBuffer[24]); //gz_msb
    BTSerial.write(fifoBuffer[25]); //gz_lsb
  */
}

void select_sensor(int sensor_id) {
  switch (sensor_id) {
    case 1:
      digitalWrite(PIN_S1, LOW);
      digitalWrite(PIN_S2, HIGH);
      digitalWrite(PIN_S3, HIGH);
      digitalWrite(PIN_S4, HIGH);
      digitalWrite(PIN_S5, HIGH);
      break;
    case 2:
      digitalWrite(PIN_S1, HIGH);
      digitalWrite(PIN_S2, LOW);
      digitalWrite(PIN_S3, HIGH);
      digitalWrite(PIN_S4, HIGH);
      digitalWrite(PIN_S5, HIGH);
      break;
    case 3:
      digitalWrite(PIN_S1, HIGH);
      digitalWrite(PIN_S2, HIGH);
      digitalWrite(PIN_S3, LOW);
      digitalWrite(PIN_S4, HIGH);
      digitalWrite(PIN_S5, HIGH);
      break;
    case 4:
      digitalWrite(PIN_S1, HIGH);
      digitalWrite(PIN_S2, HIGH);
      digitalWrite(PIN_S3, HIGH);
      digitalWrite(PIN_S4, LOW);
      digitalWrite(PIN_S5, HIGH);
      break;
    case 5:
      digitalWrite(PIN_S1, HIGH);
      digitalWrite(PIN_S2, HIGH);
      digitalWrite(PIN_S3, HIGH);
      digitalWrite(PIN_S4, HIGH);
      digitalWrite(PIN_S5, LOW);
      break;
  }
  delayMicroseconds(10);
}

