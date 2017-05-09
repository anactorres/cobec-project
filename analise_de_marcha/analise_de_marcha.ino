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

#include "SdFat.h"
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

// Pino ligado ao CS do modulo
#define chipSelect 10
#define LED_PIN 2
#define BUTTON 3
#define sampFreq 100
#define PSDMP 42
#define ST '$'
#define ET '\n'

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

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
float q[4];           // [w, x, y, z]         quaternion container
int16_t ax, ay, az;
int16_t gx, gy, gz;
bool is_alive = false;
int timer_id;

SdFat sdCard;
SdFile meuArquivo;

bool running_coleta = false;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  inicializar_sensor();
  // Inicializa o modulo SD
  if (!sdCard.begin(chipSelect, SPI_HALF_SPEED))sdCard.initErrorHalt();
  // Abre o arquivo coleta.txt
  if (!meuArquivo.open("coleta.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sdCard.errorHalt("Erro na abertura do arquivo coleta.txt!");
  }
}

void loop()
{
  t.update();
  if (!digitalRead(BUTTON)) {
    delay(250);
    if (!digitalRead(BUTTON)) {
      if (running_coleta) {
        // finalizes the aquisition
        // Interrompe o processo e fecha o arquivo
        DEBUG_PRINT_("Processo de gravacao interrompido. Retire o SD!\n");
        t.stop(timer_id);
        meuArquivo.close();
        digitalWrite(LED_PIN, LOW);
      } else {
        //starts the aquisition
        DEBUG_PRINT_("Testando conexao com o sensor - " + String(mpu.testConnection()) + "\n");
        mpu.resetFIFO();
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
  //Le sensor
  ler_sensor();
  //escrever dados no cartao sd
  escrever_no_cartao_sd();
  //enviar dados via bluetooth
  send_packet_via_bt();
}

void inicializar_sensor() {
  //Iniciando o sensor
  DEBUG_PRINT_("testando o sensor.....\n");
  mpu.initialize();
  if (mpu.testConnection())
  {
    DEBUG_PRINT_("Iniciando o sensor.....\n");
    //Initializes the IMU
    mpu.initialize();
    //Initializes the DMP
    uint8_t ret = mpu.dmpInitialize();
    delay(50);
    if (ret == 0) {
      mpu.setDMPEnabled(true); /*Not Calibrated yet*/
      mpu.setXAccelOffset(-520);
      mpu.setYAccelOffset(632);
      mpu.setZAccelOffset(914);
      mpu.setXGyroOffset(22);
      mpu.setYGyroOffset(-8);
      mpu.setZGyroOffset(26);
      DEBUG_PRINT_("Sensor Iniciado.\n");
      DEBUG_PRINT_("Testando conexao - " + String(mpu.testConnection()) + "\n");
    }
    else
    {
      DEBUG_PRINT_("Erro na inicializacao do sensor !\n");
    }
  }
  else
    DEBUG_PRINT_("Erro.....\n");
}
void ler_sensor() {
  numbPackets = floor(mpu.getFIFOCount() / PSDMP);
  DEBUG_PRINT_(numbPackets); DEBUG_PRINT_(" - ");
  for (int i = 0; i < numbPackets; i++) {
    mpu.getFIFOBytes(fifoBuffer, PSDMP);
  }
}

void mostrar_dados() {
  //Quaternion
  q[3] = ((fifoBuffer[12] << 8) | fifoBuffer[13]);
  q[0] = (float) ((fifoBuffer[0] << 8) | fifoBuffer[1]) / 16384.0f;
  q[1] = (float) ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0f;
  q[2] = (float) ((fifoBuffer[8] << 8) | fifoBuffer[9]) / 16384.0f;
  q[3] = (float) ((fifoBuffer[12] << 8) | fifoBuffer[13]) / 16384.0f;

  //Aceleracao
  ax = (fifoBuffer[28] << 8) | fifoBuffer[29];
  ay = (fifoBuffer[32] << 8) | fifoBuffer[33];
  az = (fifoBuffer[36] << 8) | fifoBuffer[37];

  //Giroscopio
  gx = (fifoBuffer[16] << 8) | fifoBuffer[17];
  gy = (fifoBuffer[20] << 8) | fifoBuffer[21];
  gz = (fifoBuffer[24] << 8) | fifoBuffer[25];

  DEBUG_PRINT_(q[0]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[1]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[2]);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(q[3]);
  DEBUG_PRINT_("\t");

  DEBUG_PRINT_(ax);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(ay);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(az);
  DEBUG_PRINT_("\t");

  DEBUG_PRINT_(gx);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(gy);
  DEBUG_PRINT_("\t");
  DEBUG_PRINT_(gz);
  DEBUG_PRINT_("\t\n");
}

void send_packet_via_bt() {
  Serial.write(ST); //byte Start Transmission

  //Assembling packet and sending
  Serial.write(fifoBuffer[0]); //qw_msb
  Serial.write(fifoBuffer[1]); //qw_lsb
  Serial.write(fifoBuffer[4]); //qx_msb
  Serial.write(fifoBuffer[5]); //qx_lsb
  Serial.write(fifoBuffer[8]); //qy_msb
  Serial.write(fifoBuffer[9]); //qy_lsb
  Serial.write(fifoBuffer[12]); //qz_msb
  Serial.write(fifoBuffer[13]); //qz_lsb

  Serial.write(fifoBuffer[28]); //ax_msb
  Serial.write(fifoBuffer[29]); //ax_lsb
  Serial.write(fifoBuffer[32]); //ay_msb
  Serial.write(fifoBuffer[33]); //ay_lsb
  Serial.write(fifoBuffer[36]); //az_msb
  Serial.write(fifoBuffer[37]); //az_lsb

  Serial.write(fifoBuffer[16]); //gx_msb
  Serial.write(fifoBuffer[17]); //gx_lsb
  Serial.write(fifoBuffer[20]); //gy_msb
  Serial.write(fifoBuffer[21]); //gy_lsb
  Serial.write(fifoBuffer[24]); //gz_msb
  Serial.write(fifoBuffer[25]); //gz_lsb

  Serial.write(ET); //byte End Transmission
}

void escrever_no_cartao_sd() {
  meuArquivo.write(ST); //byte Start Transmission

  //Assembling packet and sending
  meuArquivo.write(fifoBuffer[0]); //qw_msb
  meuArquivo.write(fifoBuffer[1]); //qw_lsb
  meuArquivo.write(fifoBuffer[4]); //qx_msb
  meuArquivo.write(fifoBuffer[5]); //qx_lsb
  meuArquivo.write(fifoBuffer[8]); //qy_msb
  meuArquivo.write(fifoBuffer[9]); //qy_lsb
  meuArquivo.write(fifoBuffer[12]); //qz_msb
  meuArquivo.write(fifoBuffer[13]); //qz_lsb

  meuArquivo.write(fifoBuffer[28]); //ax_msb
  meuArquivo.write(fifoBuffer[29]); //ax_lsb
  meuArquivo.write(fifoBuffer[32]); //ay_msb
  meuArquivo.write(fifoBuffer[33]); //ay_lsb
  meuArquivo.write(fifoBuffer[36]); //az_msb
  meuArquivo.write(fifoBuffer[37]); //az_lsb

  meuArquivo.write(fifoBuffer[16]); //gx_msb
  meuArquivo.write(fifoBuffer[17]); //gx_lsb
  meuArquivo.write(fifoBuffer[20]); //gy_msb
  meuArquivo.write(fifoBuffer[21]); //gy_lsb
  meuArquivo.write(fifoBuffer[24]); //gz_msb
  meuArquivo.write(fifoBuffer[25]); //gz_lsb

  meuArquivo.write(ET); //byte End Transmission
}

