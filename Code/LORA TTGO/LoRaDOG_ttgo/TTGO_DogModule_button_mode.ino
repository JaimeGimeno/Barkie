// Requests to have pressed the "request" button in order to receive coordinates

void button_mode() {

   // Wait for request
  String msg = receive_packet();

  // Requesting coordinates
  if (msg == "Coordinates") {
    
    getsendcoordinates();

  // Requesting battery status
  } else if (msg == "Battery") {
    
    //battery_action();

  // Uncaught error
  } else {
    
    Serial.print("¿Qué me mandas loco? -> ");
    Serial.println(msg);
    
  }
  
}
