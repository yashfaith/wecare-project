// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO
//#define OFFSET_AX 800
//#define OFFSET_AY 1100
//#define OFFSET_AZ 216
//#define OFFSET_GX -400
//#define OFFSET_GY -300
//#define OFFSET_GZ 80

#define G 9.80665
#define CONS 16384
#define DT 1
#define HPF 0.98
#define LPF 0.02
#define LED_PIN 13

bool blinkState = false;
double OFFSET_AX = 0;
double OFFSET_AY = 0;
double OFFSET_AZ = 0;
double OFFSET_GX = 0;
double OFFSET_GY = 0;
double OFFSET_GZ = 0;
double anglerealx = 0;
double anglerealy = 0;
double anglerealz = 0;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    double sumAx = 0;
    double sumAy = 0;
    double sumAz = 0;
    double sumGx = 0;
    double sumGy = 0;
    double sumGz = 0;
    
    // use the code below to change accel/gyro offset values
    
    for (int i = 0; i < 20; i++) 
    {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      sumAx += ax;
      sumAy += ay;
      sumAz += az;
      
      sumGx += gx;
      sumGy += gy;
      sumGz += gz;
    }
    
    OFFSET_AX = sumAx / 20;
    OFFSET_AY = sumAy / 20;
    OFFSET_AZ = (sumAz / 20) - 16384;

    OFFSET_GX = sumGx / 20;
    OFFSET_GY = sumGy / 20;
    OFFSET_GZ = sumGz / 20;

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
    //percepatan dalam satuan bit
    double acx= ax-OFFSET_AX;
    double acy= ay-OFFSET_AY;
    double acz= -(az-OFFSET_AZ);

    //kecepatan dalam satuan bit
    double gyx= gx-OFFSET_GX;
    double gyy= gy-OFFSET_GY;
    double gyz= gz-OFFSET_GZ;

    //percepatan dalam satuan m/s^2
    double accelx= acx * G / CONS;
    double accely= acy * G / CONS;
    double accelz= acz * G / CONS;

    //posisi sudut dalam satuan rad atau derajat ya(?)
    double anglex0= atan2(acx,(sqrt(acy*acy + acz*acz))) * 180 / M_PI;
    double angley0= atan2(acy,(sqrt(acx*acx + acz*acz))) * 180 / M_PI;
    double anglez0= atan2(acz,(sqrt(acy*acy + acx*acx))) * 180 / M_PI;

    //kecepatan sudut dalam satuan rad/s atau derajat (lupa(?))
    double wx= gyx * 250 / 32768000;
    double wy= gyy * 250 / 32768000;
    double wz= gyz * 250 / 32768000;

    //posisi sudut setelah difilter
    anglerealx = LPF * (anglerealx + wx * DT) + HPF * anglex0 ;
    anglerealy = LPF * (anglerealy + wy * DT) + HPF * angley0 ;
    anglerealz = LPF * (anglerealz + wz * DT) + HPF * anglez0 ;

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values 
//        Serial.print("ax:\t");
//        Serial.print(accelx); Serial.print("\t");
//        Serial.print("ay:\t");
//        Serial.print(accely); Serial.print("\t");
//        Serial.print("az:\t");
//        Serial.print(accelz); Serial.print("\t");
        
        Serial.print("wx:\t");
        Serial.print(wx); Serial.print("\t");
        Serial.print("wy:\t");
        Serial.print(wy); Serial.print("\t");
        Serial.print("wz:\t");
        Serial.print(wz); Serial.print("\t");
        
//        Serial.print(anglex0); Serial.print("\t");
//        Serial.print(angley0); Serial.print("\t");
//        Serial.print(anglez0); Serial.print("\t");
//        Serial.print(anglerealx); Serial.print("\t");
//        Serial.print(anglerealy); Serial.print("\t");
//        Serial.print(anglerealz); Serial.print("\t");
        Serial.println();
                
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(DT);
}
