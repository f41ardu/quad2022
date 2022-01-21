

void setupHMCRegister() {

 //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

 
void readHCM() {
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    mx = Wire.read()<<8 | Wire.read(); //X lsb
    mz = Wire.read()<<8 | Wire.read(); //Z lsb
    my = Wire.read()<<8 | Wire.read(); //Y lsb
  } 
}
