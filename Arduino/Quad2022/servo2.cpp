#include "servo2.h"

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

tservo *tservo::first;

tservo::tservo() : _pin(0),_val(0),_pulse(0),next(0)
{}

tservo::tservo(unsigned a) // val in ° 
{ 
  _pulse=false; 
  attach(a);
}

tservo::tservo(unsigned a, bool pulse) // val in pulses from RC
{
  _pulse=pulse;
  attach(a);
}


uint8_t tservo::attach(unsigned pin)
{
  _pin=pin;
  if(!_pulse) { // val in °
    _pulseWidth = map(90,0,180,minServoPulse,maxServoPulse);
  }   else {    // pulse from RC
    _pulseWidth = int(maxServoPulse-minServoPulse/2);
  }   
  next = first;
  first = this;
  pinMode(_pin,OUTPUT);
  digitalWrite(_pin, LOW);
  return 1; 
  }

void tservo::detach() 
{
 for ( tservo **p = &first; *p != 0; p = &((*p)->next) ) {
  if ( *p == this) {
      *p = this->next;
      this->next = 0;
      return;
      }
    }
}

void tservo::write(unsigned val) {
  _val = val;
  if (!_pulse) { // val in °
    _pulseWidth = map(_val,0,180,minServoPulse,maxServoPulse);
    } else { // val in pulse from RC     
    _pulseWidth = val;
  }
  refresh();  
}
  
void tservo::refresh()
{
    uint8_t count = 0, i = 0;
    uint16_t base = 0;
    tservo *p;
    static unsigned long lastRefresh = 0;
    unsigned long m = micros(); // micros() instead of millis 

    // if we haven't wrapped millis, and 20000 micros have not passed, then don't do anything
    if ( m >= lastRefresh && m < lastRefresh + refreshTime) return; 
    lastRefresh = m;

    for ( p = first; p != 0; p = p->next ) if ( p->_pulseWidth) count++;
    if ( count == 0) return;

    // gather all servos in an array
    tservo *s[count];
    for ( p = first; p != 0; p = p->next ) if ( p->_pulseWidth) s[i++] = p;

    // bubblesort by pulse time, ascending order
    for(;;) {
  uint8_t moved = 0;
  for ( i = 1; i < count; i++) {
      if ( s[i]->_pulseWidth < s[i-1]->_pulseWidth) {
    tservo *t = s[i];
    s[i] = s[i-1];
    s[i-1] = t;
    moved = 1;
      }
  }
  if ( !moved) break;
    }

    // turn on all the pins
    // Note the timing error here... when you have many SoftwareServos going, the
    // ones at the front will get a pulse that is a few microseconds too long.
    // Figure about 4uS/SoftwareServo after them. This could be compensated, but I feel
    // it is within the margin of error of software SoftwareServos that could catch
    // an extra interrupt handler at any time.
    for ( i = 0; i < count; i++) digitalWrite( s[i]->_pin, HIGH);

    uint8_t start = micros();
    uint8_t now = start;
    uint8_t last = now;

    // Now wait for each pin's time in turn..
    for ( i = 0; i < count; i++) {
  uint16_t go = start + s[i]->_pulseWidth;
    // loop until we reach or pass 'go' time
        for (;;) {
           now = micros();
     if ( now < last)  base += 256;
     last = now;
           if ( base+now > go) {
     digitalWrite( s[i]->_pin,LOW);
           break;
      }
  }
   }
}

unsigned tservo::read() {
  return(_val);
}
 
// An operator aequivalent for = 
tservo& tservo::operator=(unsigned value) {
       write(value);
       return *this;
}
// An operator aequivalent for +
tservo& tservo::operator+(tservo& obj) {
        write(_val+obj.read());
        return *this;
}
// An operator aequivalent for -
tservo& tservo::operator-(tservo& obj) {
        write(_val-obj.read());
        return *this;
}
// An operator aequivalent for * 
tservo& tservo::operator*(tservo& obj) {
        write(_val*obj.read());
        return *this;
}
//An operator aequivalent for read()
tservo::operator unsigned() {
         return read();
}
