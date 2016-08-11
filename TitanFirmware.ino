#include <Wire.h>
#include <SPI.h>  
#include <Pixy.h>
#include <SeeedGrayOLED.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>

#include "common.h"



void MotorSpeedSetAB(unsigned char MotorSpeedA , unsigned char MotorSpeedB)  {

  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);

  if (sensor_data[SENSOR_FRONT] <= 10 || sensor_data[SENSOR_LEFTFRONT] <= 10 || sensor_data[SENSOR_RIGHTFRONT] <= 10)
  {
    //MotorSpeedA = 0;
    //MotorSpeedB = 0;
  }
  
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);        // set pwm header 
  Wire.write(MotorSpeedA);              // send pwma 
  Wire.write(MotorSpeedB);              // send pwmb    
  Wire.endTransmission();    // stop transmitting
  delay(10);
}


void MotorDirectionSet(unsigned char Direction)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);        // Direction control header
  Wire.write(Direction);              // send direction control information
  Wire.write(Nothing);              // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting 
  delay(10);
}

void setMotors(unsigned char Direction,unsigned char MotorSpeedA,unsigned char MotorSpeedB)  {  //you can adjust the driection and speed together
  MotorDirectionSet(Direction);
  //delay(10);
  MotorSpeedSetAB(MotorSpeedA,MotorSpeedB);  
  //delay(10);
}

void InitializeCommandBuffer()
{
  for (cmd_buffer_ptr = 0; cmd_buffer_ptr < CMD_BUFFER_SIZE;cmd_buffer_ptr++)
  {
    cmd_buffer[cmd_buffer_ptr] = 0;
  }
  cmd_buffer_ptr = 0;
}

void BlueToothSetup()
{
  BlueToothSerial.print("$$$");
  delay(400);
  BlueToothSerial.print("S~,5\r");
  delay(400);
  BlueToothSerial.print("SN,LINK\r");
  delay(400);
  BlueToothSerial.print("ST,253\r");
  delay(400);
  BlueToothSerial.print("SO,%\r");
  delay(400);
  BlueToothSerial.print("T,1\r");
  delay(400);
  BlueToothSerial.print("SM,0\r");
  delay(400);
  BlueToothSerial.print("SA,2\r");
  delay(400);
  BlueToothSerial.print("---\r");
}
     

void GetBlueToothData()
{
  int rd = BlueToothSerial.available();
  if (rd > 0) 
  {
    byte tmp_buffer[80];
    int n = BlueToothSerial.readBytes((byte *)tmp_buffer, rd);
    for (int i =0;i<n;i++)
    {
      if (tmp_buffer[i] != '\n')
      {
        cmd_buffer[cmd_buffer_ptr++] = tmp_buffer[i];
      }
      else
      {
        ProcessBuffer();
      }
    }
  }
}

float getFloatBuffer(int start_pos)
{
  float2bytes f2b;
  f2b.b[0] = cmd_buffer[start_pos+3];
  f2b.b[1] = cmd_buffer[start_pos+2];
  f2b.b[2] = cmd_buffer[start_pos+1];
  f2b.b[3] = cmd_buffer[start_pos];
  return f2b.f;

}

void ProcessBuffer()
{
  byte cmd_byte = cmd_buffer[0];
  
  switch (cmd_byte)
  {
    case 'S':
      BlueToothSerial.print("L=");
      BlueToothSerial.print(sensor_data[SENSOR_LEFTFRONT],2);
      BlueToothSerial.print("\r\n");

      BlueToothSerial.print("F=");
      BlueToothSerial.print(sensor_data[SENSOR_FRONT],2);
      BlueToothSerial.print("\r\n");

      BlueToothSerial.print("R=");
      BlueToothSerial.print(sensor_data[SENSOR_RIGHTFRONT],2);
      BlueToothSerial.print("\r\n");

      BlueToothSerial.print("S=");
      BlueToothSerial.print(sensor_data[SENSOR_SONAR_FRONT],2);
      BlueToothSerial.print("\r\n");
    case '%':
      //Serial.println(F("BlueTooth Cmd"));
      
      break;
    case 'M':
      motorLeftPower = getFloatBuffer(1);
      motorRightPower = getFloatBuffer(5);

      
      if (motorLeftPower < -100.0f)
        motorLeftPower = -100.0f;
      if (motorRightPower < -100.0f)
        motorRightPower = -100.0f;

      if (motorLeftPower > 100.0f)
        motorLeftPower = 100.0f;
      if (motorRightPower > 100.0f)
        motorRightPower = 100.0f;

      updateMotors();
      break;
    
  }
  InitializeCommandBuffer();
}

float digitalToVoltage(int val)
{
  return ((float)val / 1023.0F) * 5.0;
}

float voltageToDistance40cm(float voltage)
{
  float result = 29.37 * exp(-0.783 * voltage);
  
  if ( voltage > 2.9)
    result = 4;

  if ( voltage < 0.38)
    result = 40;
  return result;
}

void updateMotors()
{
  unsigned char mask = 0x00;
  
  if (motorLeftPower <= 0.0F)
  {
    mask |= 0x04;
  }
  else
  {
    mask |= 0x08;
  }
  if (motorRightPower <= 0.0F)
  {
    mask |= 0x02;
  }
  else
  {
    mask |= 0x01;
  }

  if (motorLeftPower == 0.0F && motorRightPower == 0.0F) 
    mask = 0;
  
  MotorDirectionSet(mask);

  unsigned char powerLeft = (unsigned char)(abs(motorLeftPower));
  unsigned char powerRight = (unsigned char)(abs(motorRightPower));

  MotorSpeedSetAB(powerRight,powerLeft);
}

void sort(int a[], int size) {
    for(int i=0; i<(size-1); i++) {
        bool flag = true;
        for(int o=0; o<(size-(i+1)); o++) {
            if(a[o] > a[o+1]) {
                int t = a[o];
                a[o] = a[o+1];
                a[o+1] = t;
                flag = false;
            }
        }
        if (flag) break;
    }
}

float getDistance80cm(int irPin)
{
  int ir_val[NB_SAMPLE];

  for (int i=0; i<NB_SAMPLE; i++){
      // Read analog value
      ir_val[i] = analogRead(irPin);
  }
  
  // Sort it 
  sort(ir_val,NB_SAMPLE);
  return 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0, -1.2045);
}

float getDistanceSonar(int sonarPin)
{
  int ir_val[NB_SAMPLE];

  for (int i=0; i<NB_SAMPLE; i++){
      // Read analog value
      ir_val[i] = analogRead(sonarPin);
  }
  
  // Sort it 
  sort(ir_val,NB_SAMPLE);
  float voltage =  digitalToVoltage(ir_val[NB_SAMPLE/2]); //((float)ir_val[NB_SAMPLE] / 1023.0) * 5.0; //map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0;
  float distInches = voltage / 0.0098;
  return distInches  * 2.54;
  //return 27.728 * pow(map(ir_val[NB_SAMPLE / 2], 0, 1023, 0, 5000)/1000.0, -1.2045);
}

void updateDistance()
{
  int valLeftRear = analogRead(PIN_SENSOR_LEFT_REAR);
  int valRear = analogRead(PIN_SENSOR_REAR);
  int valRightRear = analogRead(PIN_SENSOR_RIGHT_REAR);
  
  //int valLeftFront = analogRead(PIN_SENSOR_LEFT_FRONT);
  //int valFront = analogRead(PIN_SENSOR_FRONT);
  //int valRightFront = analogRead(PIN_SENSOR_RIGHT_FRONT);

  
  
  sensor_data[SENSOR_LEFTREAR] = voltageToDistance40cm(digitalToVoltage(valLeftRear));
  sensor_data[SENSOR_REAR] = voltageToDistance40cm(digitalToVoltage(valRear));
  sensor_data[SENSOR_RIGHTREAR] = voltageToDistance40cm(digitalToVoltage(valRightRear));

  sensor_data[SENSOR_LEFTFRONT] = getDistance80cm(PIN_SENSOR_LEFT_FRONT) ;
  sensor_data[SENSOR_FRONT] = getDistance80cm(PIN_SENSOR_FRONT);
  sensor_data[SENSOR_RIGHTFRONT] = getDistance80cm(PIN_SENSOR_RIGHT_FRONT);

  sensor_data[SENSOR_SONAR_FRONT] = getDistanceSonar(PIN_SONAR_FRONT);
}

void updateGPS()
{
  GPS.read();
  //if (c) BlueToothSerial.print(c);
  if (GPS.newNMEAreceived()) 
  {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
        return; 
  }
  
}

void updateDisplay()
{
  if (timerDisplay > millis()) timerDisplay = millis();
  if (millis() - timerDisplay < REFRESH_RATE)
  {
    return;
  }
  timerDisplay = millis();
  char buffer[50];
  //SeeedGrayOled.clearDisplay(); 
  SeeedGrayOled.setGrayLevel(15); //Set Grayscale level. Any number between 0 - 15.
  SeeedGrayOled.setTextXY(0,0);  //set Cursor to ith line, 0th column
  
  sprintf(buffer, "LF: %.2f", sensor_data[SENSOR_LEFTFRONT]);
  SeeedGrayOled.putString(buffer);
  
  SeeedGrayOled.setTextXY(1,0);  //set Cursor to ith line, 0th column
  sprintf(buffer, "F: %.2f", sensor_data[SENSOR_FRONT]);
  SeeedGrayOled.putString(buffer);

  SeeedGrayOled.setTextXY(2,0);  //set Cursor to ith line, 0th column
  sprintf(buffer, "RF: %.2f", sensor_data[SENSOR_RIGHTFRONT]);
  SeeedGrayOled.putString(buffer);

  /*SeeedGrayOled.setTextXY(3,0);  //set Cursor to ith line, 0th column
  sprintf(buffer, "Count: %i", totalPixyBlocks);
  SeeedGrayOled.putString(buffer);

  if (totalPixyBlocks > 0)
  {
    SeeedGrayOled.setTextXY(4,0);  //set Cursor to ith line, 0th column
    SeeedGrayOled.putString("              ");
    sprintf(buffer, "%i,%i", pixy.blocks[0].x, pixy.blocks[0].y);
    SeeedGrayOled.putString(buffer);

    SeeedGrayOled.setTextXY(5,0);  //set Cursor to ith line, 0th column
    SeeedGrayOled.putString("              ");
    sprintf(buffer, "%i,%i", pixy.blocks[0].width, pixy.blocks[0].height);
    SeeedGrayOled.putString(buffer);
  }*/

  SeeedGrayOled.setTextXY(3,0);  //set Cursor to ith line, 0th column
  SeeedGrayOled.putString("              ");
  sprintf(buffer, "F=%i Q=%i", (int)GPS.fix, (int)GPS.fixquality);
  SeeedGrayOled.putString(buffer);

  SeeedGrayOled.setTextXY(4,0);  //set Cursor to ith line, 0th column
  SeeedGrayOled.putString("              ");
  sprintf(buffer, "%i:%i:%i", (int)GPS.hour, (int)GPS.minute, (int)GPS.seconds);
  SeeedGrayOled.putString(buffer);

  SeeedGrayOled.setTextXY(5,0);  //set Cursor to ith line, 0th column
  SeeedGrayOled.putString("              ");
  sprintf(buffer, "Sat: %i", GPS.satellites);
  SeeedGrayOled.putString(buffer);

  SeeedGrayOled.setTextXY(6,0);  //set Cursor to ith line, 0th column
  SeeedGrayOled.putString("              ");
  sprintf(buffer, "X=%.2f", euler.x());
  SeeedGrayOled.putString(buffer);

  SeeedGrayOled.setTextXY(7,0);  //set Cursor to ith line, 0th column
  SeeedGrayOled.putString("              ");
  sprintf(buffer, "Y=%.2f", euler.y());
  SeeedGrayOled.putString(buffer);

  SeeedGrayOled.setTextXY(8,0);  //set Cursor to ith line, 0th column
  SeeedGrayOled.putString("              ");
  sprintf(buffer, "Z=%.2f", euler.z());
  SeeedGrayOled.putString(buffer);
}

void InitializeIRSensors()
{
  pinMode (PIN_SENSOR_LEFT_REAR, INPUT);
  pinMode (PIN_SENSOR_REAR, INPUT);
  pinMode (PIN_SENSOR_RIGHT_REAR, INPUT);
  
  pinMode (PIN_SENSOR_LEFT_FRONT, INPUT);
  pinMode (PIN_SENSOR_FRONT, INPUT);
  pinMode (PIN_SENSOR_RIGHT_FRONT, INPUT);
  
}

void InitializeSonar()
{
  pinMode (PIN_SONAR_FRONT, INPUT);
}

void InitializeGPS()
{
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}

void InitializePixy()
{
  pixy.init();
  delay(10);
  pixy.setServos(PIXY_RCS_CENTER_POS, PIXY_RCS_CENTER_POS);
}

void InitializeDisplay()
{
  SeeedGrayOled.init();             //initialize SEEED OLED display
  SeeedGrayOled.clearDisplay();     //Clear Display.
  SeeedGrayOled.setNormalDisplay(); //Set Normal Display Mode
  SeeedGrayOled.setVerticalMode();  // Set to vertical mode for displaying text
}

void InitializeSDCard()
{
  if (!SD.begin(PIN_CS_SDCARD)) {
    //Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
}

void InitializeIMU()
{
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  delay(1000);
 
  bno.setExtCrystalUse(true);
}

void updateIMU()
{
  euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
}

void setup()  {
  timerDisplay = millis();
  Wire.begin(); // join i2c bus (address optional for master)
  delayMicroseconds(10000); //wait for motor driver to initialization

  InitializeCommandBuffer();
  USBSerial.begin(115200);
  BlueToothSerial.begin(115200);
  InitializeSDCard();
  MotorSpeedSetAB(0,0);
  InitializeDisplay();  

  //InitializePixy();
  InitializeIMU();
  InitializeGPS();
  InitializeIRSensors();
  InitializeSonar();
  
}

void writeDataSD(char *dataString)
{
  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  }
}

void loop()  {
  updateDistance();
  GetBlueToothData();
  updateDistance();
  //totalPixyBlocks = pixy.getBlocks();
  updateGPS();
  updateIMU();
  
  //updateDisplay();
  
}
