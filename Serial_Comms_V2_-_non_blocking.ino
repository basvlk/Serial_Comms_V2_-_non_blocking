

//PROGRAM CONTROL
const byte ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
const byte  BusyLedPin = 12 ; //optial pin to indicate reading is in progress
unsigned long previousMillis = 0;  // will store last time at the end of the previous loop
unsigned long currentMillis = 0;  // will store the current time
unsigned long CommsTimeout = 5000;    // When the program is expecting X bytes to arrive, this is how long it will wait before discarding

//DIAGNOSTIC TOOLS
byte Diagnostic = 1;                // switches on all kinds of diagnostic feedback from various locations in the program
unsigned long Slowdown = 1000;                  // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
byte LoopIteration = 0;             // to track loop iterations

// SERIAL Comms- required for the core functionality of the Serial communication
const int MaxInputSize = 15;     // the maximum length of input data the sketch accepts. Keep it to the minimum required to save memory
byte ReadInBuffer[MaxInputSize]; // Buffer to read data in to
byte ReadInBufferValid = 0;     // flag to be set when full valid data is read
byte PrevBytesInBuffer = 0;     // previous number of unread Bytes waiting in the serial buffer
byte BytesInBuffer = 0;             // current number of unread Bytes waiting in the serial buffer
int NextReadIndex = 0;
unsigned long ReadStartMillis = 0;
byte ReadingBulkData = 0; //When reading is in progress - can take several loops for long inputs
byte DiscardedBytes = 0;            // number of Bytes discarded (read out of the serial buffer but not used) since the last start of a read operation. Indicates something is wrong
byte ReadRuns = 0; // if not all data is read in in one single main loop iteration, 'ReadRuns' keeps track of how many times were required to read all data
byte Mode = 0; // Mode for the program - only updated when data is validated. Mode 0-98 are OnceModes, 99 is to set 'Diagnostic', 100-> are ContModes
byte TempMode = 0; // temporary storage for 'Mode' until data is validated
int DataLength = 0;

//**********************************************************
//*************      S E T U P       ***********************
//**********************************************************
void setup() {
  pinMode(ArduinoLedPin, OUTPUT);
  pinMode(BusyLedPin, OUTPUT);
  Serial.begin(115200);
  // Serial.setTimeout(2000); // Optional, wait longer for all data to arrive
  Serial.println("]0"); // start value for host program: 0 bytes in buffer
  Serial.println("Hello.");
  memset(ReadInBuffer, 0, MaxInputSize); // Initialise ReadInBuffer, with all zeroes. Valid flag is set to 0

}
//**********************************************************
//  *************         M A I N       **********************
//**********************************************************
void loop()
{
  //  Start of loop housekeeping, and feedback
  currentMillis = millis();
  ++LoopIteration;
  PrevBytesInBuffer = BytesInBuffer;
  BytesInBuffer = Serial.available();
  LoopBlink(LoopIteration);
  FeedbackToHost();
  // End of op housekeeping, and feedback

  //Core program
  SerialInitialRead();
  if (ReadingBulkData) {
    digitalWrite(BusyLedPin, HIGH);
    ReadSerialBulkData();
  }
  //     EmptySerialBuffer();
  //     PostReadChecks();
  //  }
  ArrayToSerial(ReadInBuffer, MaxInputSize);

}

//**********************************************************
//*************        F U N C T I O N S       *************
//**********************************************************

void SerialReadPostChecks()
{
  //  all OK, properly read:
  // ReadingBulkData = 0; digitalWrite(BusyLedPin, LOW);
  //DiscardedBytes = 0;
  // Mode = TempMode;
  // ReadInBufferValid = 1;
  //
  //
  //read Bytes are not 'DataLength': make data invalid, etc
}

void ReadSerialBulkData() {
  ReadRuns++;
  Serial.print(F("[ Start BulkReadLoop. NextReadIndex: "));
  Serial.println( NextReadIndex);
  while (((Serial.available() > 0) && (NextReadIndex < DataLength)) &&  (NextReadIndex <= MaxInputSize)) {
    ReadInBuffer[NextReadIndex] = Serial.read();
    NextReadIndex++;
  }
  if (NextReadIndex > MaxInputSize) { // More data than fits in buffer
    Serial.print(F("[ ERROR: more data sent than input buffer can handle. Dumping: "));
    Serial.print(BytesInBuffer);
    Serial.println(F("[ Bytes of Data"));
    char dump[BytesInBuffer];
    Serial.readBytes(dump, (Serial.available()));
    ReadingBulkData = 0;
    digitalWrite(BusyLedPin, LOW);
    ReadInBufferValid = 0;
  }
  if (NextReadIndex == DataLength) { // valid data
    ReadingBulkData = 0; // reset 'Readin Data' flag, ready for new data
    digitalWrite(BusyLedPin, LOW);
    Mode = TempMode ;
    ReadInBufferValid = 1;
    if (Diagnostic) {
      Serial.println(F("[ Done Bulk reading, Buffer complete "));
    }
  }// end valid data
  Serial.print(F("[ End BulkReadLoop. NextReadIndex "));
  Serial.println( NextReadIndex);
}

void SerialInitialRead() {
  if (ReadingBulkData) {
    // ******* Existing Bulk Read cycle *****
    if ((currentMillis - ReadStartMillis) > CommsTimeout) { //Reading Cycle Timed Out
      Serial.print(F("[ ERROR: Timeout, waited "));
      Serial.print(CommsTimeout);
      Serial.print(F(" for incoming Bytes. Expected: "));
      Serial.print(DataLength);
      Serial.print(F(" Received: "));
      Serial.println((NextReadIndex)); //this is the next Index that would be read, so not the actual latest read index. but indexing starts 0 so NextReadIndex = #Bytes read)
      digitalWrite(BusyLedPin, LOW);
      ReadingBulkData = 0;
      ReadInBufferValid = 0;
    } // end of Reading Cycle Timed Out
  } //End Existing read cycle

  if (!ReadingBulkData) {
    //  ****** New reading cycle ******
    if (BytesInBuffer > 2)  {// there are 3 or more Bytes in Buffer
      if (Serial.read() == 255) {//And first is 255, ready to start reading
        ReadStartMillis = millis();
        DiscardedBytes = 0;
        TempMode = Serial.read(); // Second Byte = Mode, stored temporarily
        DataLength = int(Serial.read()); // Third Byte is DataLength: #Bytes to follow

        // SHORT MESSAGE
        if ((DataLength == 0)) {//short 3-Byte message only
          Mode = TempMode;
          if (Diagnostic) {
            Serial.print(F("[ INFO: the 3 initial Bytes have been read. After 255, Mode: "));
            Serial.print(TempMode);
            Serial.print(F(" Datalength: "));
            Serial.println(DataLength);
          }
        } // End first byte = 255

        //TOO LONG MESSAGE
        if ((DataLength > 0) && (DataLength > MaxInputSize)) {
          ReadingBulkData = 0;
          Serial.print(F("[ ERROR: intending to send DataLength =   "));
          Serial.print(DataLength);
          Serial.print(F(" bytes of data, but MaxInputSize =  "));
          Serial.print(MaxInputSize);
          Serial.println(F(". Dumping data. "));
          byte dump[DataLength];
          Serial.readBytes(dump, DataLength); // will read the bytes and dump until timeout, or DataLength bytes are  being read
        } // End DataLength > MaxInputSize

        // LONG MESSAGE
        if ((DataLength > 0) && (DataLength <= MaxInputSize)) { // DataLengh >0 and within limit
          ReadingBulkData = 1;
          digitalWrite(BusyLedPin, HIGH);
          NextReadIndex = 0;
          ReadRuns = 0;
          if (Diagnostic) {
            Serial.println(F("[ Read prechecks: going into bulk read mode"));
          }
        } // End long message
      } // End 'first Byte == 255

      else { // more than 3 Bytes, but the first isn't 255. It's now taken out of the buffer so in the next loop, the second Byte will be evaluated. Error feedback to host:
        ++DiscardedBytes;
        Serial.print(F("[ ERROR: first Byte is not '255', Bytes Discarded: "));
        Serial.println(DiscardedBytes);
      } // end 'first byte isnt 255'
    } // end '>2 Bytes'
  } // End of"New reading cycle"
} //End SerialInitialRead

void LoopBlink(int Loop) // Blink ArduinoLED to show program is running. Toggles On/Off every loop
{
  if (Loop % 2)
  {
    digitalWrite(ArduinoLedPin, HIGH);
  }
  else
  {
    digitalWrite(ArduinoLedPin, LOW);
  }
}// End blinkLed


void FeedbackToHost()
{
  if (PrevBytesInBuffer != BytesInBuffer) {
    Serial.print("]");
    Serial.println(BytesInBuffer);
  } //Only update BytesInBuffer value to host if there has been a change

  if (Diagnostic) {
    Serial.print(F("[ **** NEW LOOP: "));
    Serial.println(LoopIteration);
    Serial.print(F("[ currentMillis: "));
    Serial.println(currentMillis);
    Serial.print(F("[ ReadStartMillis: "));
    Serial.println(ReadStartMillis);
    Serial.print(F("[ Slowdown: "));
    Serial.println(Slowdown);
    Serial.print(F("[ Bytes in Buffer: "));
    Serial.println(BytesInBuffer);
    Serial.print(F("[Bytes Discarded: "));
    Serial.println(DiscardedBytes);
    Serial.print(F("[ TempMode: "));
    Serial.println(TempMode);
    Serial.print(F("[ DataLength: "));
    Serial.println(DataLength);
    Serial.print(F("[ Reading Bulk data in progress: "));
    Serial.println(ReadingBulkData);
    Serial.print(F("[ ReadRuns: "));
    Serial.println(ReadRuns);
    Serial.print(F("[ Mode: "));
    Serial.println(Mode);
    delay(Slowdown);
  }
}

// PrintArrayToSerial: A diagnostic printing out full arrays to the serial port
void ArrayToSerial(byte Array[], int N) {
  Serial.println("}"); // CLEARS array in Host
  for (int i = 0; i < N ; i++)
  {
    Serial.print("{ ");
    Serial.println(Array[i], DEC);
  }
} // END PrintArrayToSerial


/*



  void PostReadChecks(index, DataLength)
  if L
*/

