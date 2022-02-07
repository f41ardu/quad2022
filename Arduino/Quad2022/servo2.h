/*
Universal Servo Libary 
Works with almost any *tiny's

Version 1.2 ThR
22.12.2021
Version 1.1 ThR
18.02.2015

 *   This is a universal servo library for the Attiny/Arduino.

  See xxxxx code project for latest, bugs and info http://XXXX
  
  This library was inspired by my interest in objectoriented coding 
        and to understand to build my my own classes.  
  
  It provides a universal servo libary for atmel based microcontrollers
        based on Arduino commands supported on Attinys to avoid direct hw access 
        to Attiny's timers. 
        
        See http://highlowtech.org/?p=1695 for the excellent documentation of 
        Attiny support. 
          
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  (the file gpl.txt is included with the library's zip package)
*/


#ifndef tservo2_H
#define tservo2_H


#include <Arduino.h>
#include <inttypes.h>

#define refreshTime 20000
#define minPulse 450
#define maxPulse 2600 

class tservo
{
  public: 
   tservo();
   tservo(unsigned); // val in °
   tservo(unsigned,bool); // val from RC (true) 
   void write(unsigned);
   unsigned read(); 
// Operator Overload 
    tservo& operator=(unsigned);
    tservo& operator+(tservo& obj); 
    tservo& operator-(tservo& obj);
    tservo& operator*(tservo& obj);
    operator unsigned(); 
     
   private: 
     unsigned long _pulseWidth;
     unsigned _pin; 
     unsigned _val;
     bool _pulse;
     uint8_t attach(unsigned);
     
     void detach();
     static void refresh();
     class tservo *next;
     static tservo* first;
     
   
}; 
#endif  // tservo2_H
