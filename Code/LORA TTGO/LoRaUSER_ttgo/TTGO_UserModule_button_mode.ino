// Requests to have pressed the "request" button in order to receive coordinates

void button_mode() {

   if (SerialBT.available()) { // SerialBT.available()
    STATE = SerialBT.read();

    if (STATE == '1') { // STATE == '1'

      // Send request to dog
      Serial.print("Sending 'request'");
      send_packet("Coordinates");

      // Wait for response      
      msg = receive_packet();
      // Send coordinates to app
      SerialBT.print(msg);

      STATE = 0;
    }
  
  }
  
}
