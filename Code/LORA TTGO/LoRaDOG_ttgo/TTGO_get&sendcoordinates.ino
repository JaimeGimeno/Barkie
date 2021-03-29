void getsendcoordinates() {
  
  // Get the GPS coordinates
  success = false;
  success = getGPSCoordinates();
  while (!success) {
    success = getGPSCoordinates();
  }

  //Create the String response (sentString)
  sentString = "Coordinates ";
  sentString.concat(flat * 1000000);
  sentString.concat(" ");
  sentString.concat(flon * 1000000);
  
  // Send the response with the coordinates
  Serial.print("Sending String: ");
  Serial.println(sentString);

  send_packet(sentString);

}
