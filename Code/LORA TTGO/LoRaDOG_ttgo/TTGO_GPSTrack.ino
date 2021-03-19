bool getGPSCoordinates () {

  //Declarations
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    
    while (SoftSerial.available()){
      
      char c = SoftSerial.read();
      if (gps.encode(c)) newData = true;
      
    }
    
  }

  if (newData){

    success = true;

    // Getting the data
    gps.f_get_position(&flat, &flon, &age);

    // Serial
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  
  }

  // More statistics
  gps.stats(&chars, &sentences, &failed);

  // Print those in serial
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);

  // Just in case GPS fails
  if (chars == 0) Serial.println("** No characters received from GPS: check wiring **");

  return success;

}  
