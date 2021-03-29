// Will send coordinates repeteadly
// That will create the "live location" app

void live_mode(int freq) {

  // Wait for response      
  msg = receive_packet();
  // Send coordinates to app
  SerialBT.print(msg);
  
}
