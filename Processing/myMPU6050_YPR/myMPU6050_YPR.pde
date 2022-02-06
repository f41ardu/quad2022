/**
 Visualize an MPU6050 using quaternion (the teapot packete 
 coming from the serial port. 
 
 INSTRUCTIONS: 
 This program requires MPU6050_DMP6.ino program with teapot packet enabled 
 running on your Arduino and the Arduino connected to your PC.
 
 This program is  based on the FREE_IMO demo package fron Fabio Varesano
 Copyright (C) 2011 Fabio Varesano - http://www.varesano.net/
 
 Copyright (c) 2022 f41ardu - quad2022 edition
 Copyright (C) 2016 f41_ardu 
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 */

import processing.serial.*;

Serial myPort;  // Create object from Serial class
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
String[] msg=new String[60];
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;
int startTime = 0;
float freq = 0; 
// it migh be a good idea to make use of the toxi lib
float [] q = new float [4];
float [] hq = null;
float [] Euler = new float [3]; // psi, theta, phi
float [] ypr = new float [3]; // yaw, pitch, roll
float delta = 0; 
PImage img, img2;

PFont font;
final int VIEW_SIZE_X = 1000, VIEW_SIZE_Y = 800;

void setup() 
{
  size(1000, 800, P3D);
  String serialPort = "/dev/ttyUSB1";
  myPort = new Serial(this, serialPort, 115200);  
  myPort.bufferUntil('\n');
      myPort.write('r');
// The font and the images must be located in the sketch's "data" directory to load successfully
  img = loadImage("GY-521_Front.JPG");
  img.resize(200, 100);
  img2 = loadImage("GY-521_Back.JPG");
  img2.resize(200, 100);
  imageMode(CENTER);
  font = loadFont("CourierNew36.vlw"); 
  delay(100);
  myPort.clear();
  myPort.write("1");
  q[0] =1.0;
  q[1] =0.0;
  q[2] =0.0;
  q[3] =0.0;
  delay(4000);
}

void draw() {
  background(0);
//  fill(#ffffff);
   
  if (millis() - interval > 500) {
    // resend single character to trigger DMP init/start
    // in case the MPU is halted/reset while applet is running
    myPort.write('r');
    interval = millis();
  }

  if (hq != null) { // use home quaternion
    quaternionToYPR(quatProd(hq, q), Euler);
    text("Disable home position by pressing \"n\"", 20, VIEW_SIZE_Y - 30);
  } else {
    quaternionToYPR(q, Euler);
    text("Point Sensor's X axis to your monitor then press \"h\"", 20, VIEW_SIZE_Y - 30);
  }
   
  
  textFont(font, 20);
  textAlign(LEFT, TOP);
  text("Q:\n" + nf(q[0], 1, 2) + "\n" + nf(q[1], 1, 2) + "\n" + nf(q[2], 1, 2) + "\n" + nf(q[3], 1, 2), 20, 20);
  text("Euler Angles: Yaw (psi)  : " + nf(degrees(Euler[0]), 3, 2)  + "\nDelta Euler: " + nf(degrees(Euler[2] - delta), 3, 2 ) + "\n′°/s Euler: " + nf(degrees(Euler[2] - delta)/(1e-3*(millis()-startTime)), 0, 5 )   
        + "\nPitch (theta): " + nf(degrees(Euler[1]), 3, 2) + "\nRoll (phi)  : " + nf(degrees(Euler[2]), 3, 2) +"\nFreq (Hz)  :" +freq, 200, 20);

  drawCube();
}

void keyPressed() {
  if (key == 'h') {
    println("pressed h");
    // set hq the home quaternion as the quatnion conjugate coming from the sensor fusion
    hq = quatConjugate(q);
  } else if (key == 'n') {
    println("pressed n");
    hq = null;
  } else if (key == 'r' ){
    delta = Euler[2];
    startTime = millis(); 
  }
}