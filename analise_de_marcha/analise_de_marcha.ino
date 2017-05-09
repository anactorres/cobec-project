/* ------------------------------------------------------------------------------
   FEDERAL UNIVERSITY OF UBERLANDIA
   Faculty of Electrical Engineering
   Biomedical Engineering Lab
   Uberlândia, Bra[2]il
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
 | [g[1]RO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

#include "SdFat.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Timer.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//#define USING_SD_MODULE

#define DEBUG_PRINT_(x) Serial.print(x)
//#define DEBUG_PRINT_(x)

// Pino ligado ao CS do modulo
#define chipSelect 10
#define LED_PIN 2
#define BUTTON_PIN 4
#define sampFreq 100
#define PSDMP 42
#define ST '$'
#define ET '\n'

// class default I2C address is 0x68
// specific I2C addresses ma[1] be passed as a parameter here
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
float a[3];
float g[3];
bool is_alive = false;
int timer_id;
#ifdef USING_SD_MODULE
SdFat sdCard;
SdFile meuArquivo;
#endif /*USING_SD_MODULE*/

bool running_coleta = false;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  inicializar_sensor();
#ifdef USING_SD_MODULE
  // Inicializa o modulo SD
  if (!sdCard.begin(chipSelect, SPI_HALF_SPEED))sdCard.initErrorHalt();
  // Abre o arquivo coleta.txt
  if (!meuArquivo.open("coleta.txt", O_RDWR | O_CREAT | O_AT_END)) {
    sdCard.errorHalt("Erro na abertura do arquivo coleta.txt!");
  }
#endif /*USING_SD_MODULE*/
  DEBUG_PRINT_("Aguardando sinal do botao.\n");
}

void loop()
{
  t.update();
  if (digitalRead(BUTTON_PIN)) {
    delay(250);
    if (digitalRead(BUTTON_PIN)) {
      DEBUG_PRINT_("Aguardando liberacao do botao.\n");
      while (digitalRead(BUTTON_PIN)); //Espera soltar o botao
      DEBUG_PRINT_("Liberado.\n\n");
      
      if (running_coleta) {
        // finalizes the aquisition
        // Interrompe o processo e fecha o arquivo
        DEBUG_PRINT_("Aquisicao finalizada.\n");
        t.stop(timer_id);
#ifdef USING_SD_MODULE
        meuArquivo.close();
        DEBUG_PRINT_("Processo de gravacao interrompido. Retire o SD!\n");
#endif /*USING_SD_MODULE*/
        digitalWrite(LED_PIN, LOW);
      } else {
        //starts the aquisition
        DEBUG_PRINT_("Aquisicao iniciada.\n");
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
#ifdef USING_SD_MODULE
  //escrever dados no cartao sd
  escrever_no_cartao_sd();
#endif /*USING_SD_MODULE*/
  //enviar dados via bluetooth
  //send_packet_via_bt();
  mostrar_dados();
}

void inicializar_sensor() {
  //Iniciando o sensor
  DEBUG_PRINT_("Testando o sensor.....\n");
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
      mpu.setXAccelOffset(-1759);
      mpu.setYAccelOffset(1051);
      mpu.setZAccelOffset(1510);
      mpu.setXGyroOffset(-117);
      mpu.setYGyroOffset(-27);
      mpu.setZGyroOffset(71);
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
#ifdef USING_SD_MODULE
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
#endif /*USING_SD_MODULE*/
}

