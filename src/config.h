/*
    motor1  motor2
      \\      //
       \\    //
left  [ ESP32 ] right
       //    \\
      //      \\
    motor4  motor3
*/

//motor
#define MOTOR1_PIN 13
#define MOTOR2_PIN 33
#define MOTOR3_PIN 34
#define MOTOR4_PIN 14

//I2C 
#define SDA_PIN 21
#define SCL_PIN 26
#define MPU9250_ADRESS 0x68

//UART
#define SERIAL_BAUD_RATE 115200

