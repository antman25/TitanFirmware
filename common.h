
#define USBSerial Serial
#define BlueToothSerial Serial1
#define GPSSerial Serial3

#define PIN_CS_SDCARD             10

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
#define I2CMotorDriverAdd         0x0f   // Set the address of the I2CMotorDriver

#define SENSOR_LEFTFRONT          0
#define SENSOR_FRONT              1
#define SENSOR_RIGHTFRONT         2
#define SENSOR_LEFTREAR           3
#define SENSOR_REAR               4
#define SENSOR_RIGHTREAR          5
#define SENSOR_SONAR_LEFT         6
#define SENSOR_SONAR_FRONT        7
#define SENSOR_SONAR_RIGHT        8

#define TOTAL_SENSOR              9

#define PIN_SENSOR_LEFT_REAR      A0
#define PIN_SENSOR_REAR           A1
#define PIN_SENSOR_RIGHT_REAR     A2

#define PIN_SENSOR_LEFT_FRONT     A3
#define PIN_SENSOR_FRONT          A6
#define PIN_SENSOR_RIGHT_FRONT    A7

#define PIN_SONAR_FRONT           A0


#define CMD_BUFFER_SIZE         512

#define NB_SAMPLE               25
#define REFRESH_RATE            3000

size_t cmd_buffer_ptr = 0;
uint8_t cmd_buffer[CMD_BUFFER_SIZE];
char *cmd_buffer_test;
float sensor_data[TOTAL_SENSOR];

float motorLeftPower;
float motorRightPower;

Pixy pixy;
Adafruit_BNO055 bno = Adafruit_BNO055();
Adafruit_GPS GPS(&GPSSerial);

uint16_t totalPixyBlocks;

uint32_t timerDisplay = millis();

union float2bytes { byte b[sizeof(float)]; float f; };

imu::Vector<3> euler;
imu::Vector<3> gyro;
