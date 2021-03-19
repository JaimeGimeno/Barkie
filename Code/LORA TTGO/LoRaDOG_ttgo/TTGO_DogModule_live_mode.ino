// Will send coordinates repeteadly
// That will create the "live location" app

void live_mode(int freq) {

  delay(freq/2);
  getsendcoordinates();
  delay(freq/2);
  
}
