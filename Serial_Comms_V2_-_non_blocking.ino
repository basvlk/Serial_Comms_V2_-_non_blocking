//PROGRAM CONTROL
const byte ArduinoLedPin =  13  ;   // NOT the NeoPixel data pin!! the number of the Arduino LED pin - it's blinking helps seeing if the program runs
unsigned long previousMillis = 0;  // will store last time at the end of the previous loop
unsigned long currentMillis = 0;  // will store the current time
unsigned long CommsTimeout = 200;    // When the program is expecting X bytes to arrive, this is how long it will wait before discarding

//DIAGNOSTIC TOOLS
byte Diagnostic = 1;                // switches on all kinds of diagnostic feedback from various locations in the program
unsigned long Slowdown = 200;                  // Delay value (ms) added to each loop, only in 'Diagnostic' mode to allow inspecting the data coming back over serial
byte LoopIteration = 0;             // to track loop iterations

// SERIAL- required for the core functionality of the Serial communication
const int MaxInputSize =128; // the maximum length of input data the sketch accepts. Keep it to the minimum required to save memory
byte UpdateBytesInBufferToSerial = 1;                // siwtch to only write to serial if BytesInBuffer has changed
byte PrevBytesInBuffer = 0;     // previous number of unread Bytes waiting in the serial buffer
byte BytesInBuffer = 0;             // current number of unread Bytes waiting in the serial buffer
int CurrentReadIndex = 0;
byte ReadStartMillis = 0;
byte ReadingInProgress = 0; //When reading is in progress - can take several loops for long inputs
byte DiscardedBytes = 0;            // number of Bytes discarded (read out of the serial buffer but not used) since the last start of a read operation. Indicates something is wrong
byte ReadRuns = 0; // if not all data is read in in one single main loop iteration, 'ReadRuns' keeps track of how many times were required to read all data
byte Mode = 0; // Mode for the program - only updated when data is validated. Mode 0-98 are OnceModes, 99 is to set 'Diagnostic', 100-> are ContModes
byte TempMode = 0; // temporary storage for 'Mode' until data is validated
int DataLength = 0;
byte ReadInBuffer[MaxInputSize];



//**********************************************************
//*************      S E T U P       ***********************
//**********************************************************
void setup() {
  pinMode(ArduinoLedPin, OUTPUT);
  Serial.begin(115200);
  Serial.println("]0"); // start value for host program: 0 bytes in buffer
  Serial.println("Hello.");
  memset(ReadInBuffer,0,MaxInputSize);

}
//**********************************************************
//  *************         M A I N       **********************
//**********************************************************
void loop()
{
  //  Start of loop housekeeping
  currentMillis = millis();
  ++LoopIteration;
  LoopBlink(LoopIteration);
  PrevBytesInBuffer = BytesInBuffer;
  BytesInBuffer = Serial.available();
  // End of loop housekeeping

  //Core program
  FeedbackToHost();
  SerialReadPrechecks();
  
    if (ReadingInProgress) {ReadSerialData();}
  //     EmptySerialBuffer();
  //     PostReadChecks();
  //  }
}

void ReadSerialData()
{
  ReadRuns++;
  while (Serial.available()>0) {
    Serial.print(F("[ StartReadLoop CurrentReadIndex "));
  Serial.println( CurrentReadIndex);
    ReadInBuffer[CurrentReadIndex++]=Serial.read();
      Serial.print(F("[ StartReadLoop CurrentReadIndex "));
  Serial.println( CurrentReadIndex);
  }
  
}

void SerialReadPrechecks() {
  if (!ReadingInProgress) { // if we were NOT in reading mode,
    if (BytesInBuffer > 2) // there are 3 or more Bytes in Buffer
    {
      if (Serial.read() == 255) //And first is 255, ready to start reading
      {
        ReadingInProgress = 1;
        ReadStartMillis = 0;
        CurrentReadIndex = 0;
        ReadRuns = 0;
      }
      else { // more than 3 Bytes, but the first isn't 255. It's now taken out of the buffer so in the next loop, the second Byte will be evaluated. Error feedback to host:
        ++DiscardedBytes;
        Serial.print(F("[ ERROR: first Byte is not '255', Bytes Discarded: "));
        Serial.println(DiscardedBytes);
      }
    }

  }
  else  {// we're already in Read mode
    Serial.println("woo hoo we're in reading mode ");
    
  }

  //
  //                 void EmptySerialBuffer(LastSerialReadIndex);
  //    while (Serial.available);
  //    LastSerialReadIndex++
  //    ReadInBuffer[index] = Serial.read;
  //    DiscardedBytes = =; // At some stage, after successful read, need to reset 'discarded Bytes'
  //    if
}

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

  delay(1); /////REMOVE THIS LINE WHEN MORE CODE ADDED!!: somehow the serial communication deosn't work well if there is no slight delay

  if (Diagnostic) {                  //Diag
    Serial.print(F("[ **** NEW LOOP: "));    //Diag
    Serial.println(LoopIteration);        //Diag
    Serial.print(F("[ currentMillis: "));    //Diag
    Serial.println(currentMillis);        //Diag
    Serial.print(F("[ Slowdown: "));         //Diag
    Serial.println(Slowdown);             //Diag
    delay(Slowdown);                      //Diag
    Serial.print(F("[ Bytes in Buffer: "));         //Diag
    Serial.println(BytesInBuffer);             //Diag
    Serial.print(F("[ Reading in progress: "));         //Diag
    Serial.println(ReadingInProgress);             //Diag
  }
}



/*



  void PostReadChecks(index, DataLength)
    if L
*/

