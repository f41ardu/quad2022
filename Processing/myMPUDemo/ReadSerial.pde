// quad2022 edition
void serialEvent(Serial myPort) {
  interval = millis();   

  while ( myPort.available() > 0) {
    String myString = myPort.readStringUntil('\n');
    //println(myString);
    msg = trim(split(myString, ','));
    // println(msg);
    for (i=0; i < msg.length; i++ ) {
      value[i]=radians(float(msg[i]));
      // print(value[i]); 
    } 
  }     
}
