#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // RX | TX

double bauds[5] = {9600, 19200, 38400, 57600, 115200};
int i = 0;
void setup()
{
  Serial.begin(bauds[0]);
  Serial.print("Use both NL & CR as line endings\n");
  Serial.print("AT : Check the connection.\n");
  Serial.print("AT+NAME : See default name \n");
  Serial.print("AT+ADDR : see default address\n");
  Serial.print("AT+VERSION : See version\n");
  Serial.print("AT+UART : See baudrate\n");
  Serial.print("AT+ROLE: See role of bt module(1=master/0=slave)\n");
  Serial.print("AT+RESET : Reset and exit AT mode\n");
  Serial.print("AT+ORGL : Restore factory settings\n");
  Serial.print("AT+PSWD: see default password\n");
  Serial.print("AT+BAUDX Options: \n");
  Serial.print("1 set to 1200bps");
  Serial.print("2 set to 2400bps\n");
  Serial.print("3 set to 4800bps\n");
  Serial.print("4 set to 9600bps (Default)\n");
  Serial.print("5 set to 19200bps\n");
  Serial.print("6 set to 38400bps\n");
  Serial.print("7 set to 57600bps\n");
  Serial.print("8 set to 115200bps\n");
  Serial.print("so sending AT+BAUD4 will set the baud rate to 9600.\n");
  Serial.print("For example, AT+NAMEPROTOTYPE will set the name to PROTOTYPE.\n");
  Serial.print("AT : check the connection\n");
  Serial.print("AT+NAME: Change name. No space between name and command.\n");
  Serial.print("AT+BAUD: change baud rate, x is baud rate code, no space between command and code.\n");
  Serial.print("AT+PIN: change pin, xxxx is the pin, again, no space.\n");
  Serial.print("AT+VERSION\n");
  Serial.println("Enter AT commands:");

  Serial.println("********************AUTOCONFIG*********************");
  Serial.println("Starting config");
  BTSerial.begin(bauds[i]);
  delay(200);
  //BTSerial.print("AT+ORGL\r\n");
  /*
  delay(1000);

  // Should respond with OK
  BTSerial.print("AT\r\n");
  waitForResponse();

  // Should respond with its version
  BTSerial.print("AT+VERSION\r\n");
  waitForResponse();

  // Set pin to 0000
  BTSerial.print("AT+PSWD1234\r\n");
  waitForResponse();

  // Set the name to ROBOT_NAME
  String rnc = String("AT+NAME") + String("MODULO_DAFEELT1") + String("\r\n");
  BTSerial.print(rnc);
  waitForResponse();

  // Set baudrate to 57600
  BTSerial.print("AT+UART115200,0,0\r\n");
  waitForResponse();
    // Set baudrate to 57600
  BTSerial.print("AT+UART=115200,0,0\r\n");
  waitForResponse();

  // Set baudrate to 57600
  BTSerial.print("AT+BAUD8\r\n");
  waitForResponse();

    // Set baudrate to 57600
  BTSerial.print("AT+RESET\r\n");
  waitForResponse();

*/
  //Serial.println("Done!");
}



void waitForResponse() {
  delay(1000);
  while (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }
  Serial.write("\n");
}

void loop()
{

  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available()) {
    Serial.write(BTSerial.read());
  }

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available()) {
    char leitura = Serial.read();
    if (leitura == '$') {
      i = i + 1;
      i = i % 5;
      Serial.println(bauds[i]);
      BTSerial.begin(bauds[i]);
    }
    BTSerial.write(leitura);
  }
}
