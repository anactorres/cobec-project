// Programa: Análise de marcha
// Lê acelerômetro, giroscópio e o quartenion, os salva em um cartão sd simultaneamente envia vio bluetooth
// Esse sistema deve ser colocado sobre o pé para se realizar a análise da marcha
// Autor: Ana 

#include "SdFat.h" 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

SdFat sdCard;
SdFile meuArquivo;
 
// Pino ligado ao CS do modulo
#define chipSelect 10
#define PIN_LED 2
#define BUTTON 3
#define 

bool running_coleta = false;

void setup()
{
  pinMode(LED_PIN,OUTPUT);
  pinMode(BUTTON,OUTPUT);
  Serial.begin(115200);
  // Inicializa o modulo SD
  if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();
  // Abre o arquivo coleta.txt
  if (!meuArquivo.open("coleta.txt", O_RDWR | O_CREAT | O_AT_END))
  {
    sdCard.errorHalt("Erro na abertura do arquivo coleta.txt!");
  }
}


   
 
void loop()
{

  if(!digitalRead(BUTTON)){
    delay(250);
    if(!digitalRead(BUTTON)){
      if(running_coleta){
        // finalizes the aquisition
        // Interrompe o processo e fecha o arquivo
        Serial.println("Processo de gravacao interrompido. Retire o SD!");
        meuArquivo.close();
      } else {
        //starts the aquisition
        
      }
      running_coleta = !running_coleta;
    }
  }
  
  if(running_coleta){
l
  }
  // Grava dados do potenciometro em LER_POT.TXT
  meuArquivo.print("Leitura Potenciometro: ");
  meuArquivo.println(valor);
  
}
/**
 *  Methodo para leitura de dados, escrita e envio. É chamada pelo timer.
 */
void takeReading(){
  //Le sensor

  //escrever dados no cartao sd
  
  //enviar dados via seria
 
}


