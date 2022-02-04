
void serialEvent(Serial myport) {
  interval = millis();
  while (myport.available() > 0) {
    int ch = myport.read();

    if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
    synced = 1;
    // print ((char)ch);

    if ((serialCount == 1 && ch != 2)
      || (serialCount == 12 && ch != '\r')
      || (serialCount == 13 && ch != '\n')) {
      serialCount = 0;
      synced = 0;
      return;
    }

    if (serialCount > 0 || ch == '$') {
      teapotPacket[serialCount++] = (char)ch;
      if (serialCount == 14) {
        serialCount = 0; // restart packet byte position

        // get quaternion from data packet
        q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
        q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
        for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
      }
    }
  }
}

/*
// quad2022 edition
void serialEvent(Serial myPort) {
  interval = millis();   
   
//  while ( myPort.available() > 0) {
    String myString = myPort.readStringUntil('\n');
    //println(myString);
    msg = trim(split(myString, ','));
    //println(msg);
    //freq = float(msg[4]);
/*    for (int i=0; i < msg.length-1; i++ ) {
      //value[i]=radians(float(msg[i]));
      q[i] = float(msg[i]);
      // print(value[i]); 
  } 
*/
/*
    q[0] = float(msg[0]);
    q[1] = float(msg[1]);
    q[2] = float(msg[2]);
    q[3] = float(msg[3]);

    
//  }     
}
*/
