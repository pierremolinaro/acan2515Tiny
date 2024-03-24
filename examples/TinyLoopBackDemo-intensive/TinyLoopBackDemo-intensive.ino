//——————————————————————————————————————————————————————————————————————————————
//  ACAN2515Tiny Tiny Demo in loopback mode, intensive test
//——————————————————————————————————————————————————————————————————————————————

#include <ACAN2515Tiny.h>

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 connections:
//    - standard SPI pins for SCK, MOSI and MISO
//    - a digital output for CS
//    - interrupt input pin for INT
//——————————————————————————————————————————————————————————————————————————————
// If you use CAN-BUS shield (http://wiki.seeedstudio.com/CAN-BUS_Shield_V2.0/) with Arduino Uno,
// use B connections for MISO, MOSI, SCK, #9 or #10 for CS (as you want),
// #2 or #3 for INT (as you want).
//——————————————————————————————————————————————————————————————————————————————

static const byte MCP2515_CS  = 10 ; // CS input of MCP2515 (adapt to your design)
static const byte MCP2515_INT =  3 ; // INT output of MCP2515 (adapt to your design)

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Driver object
//——————————————————————————————————————————————————————————————————————————————

ACAN2515Tiny can (MCP2515_CS, SPI, MCP2515_INT) ;

//——————————————————————————————————————————————————————————————————————————————
//  MCP2515 Quartz: adapt to your design
//——————————————————————————————————————————————————————————————————————————————

static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL ; // 16 MHz

//-----------------------------------------------------------------

static ACANBuffer8 gBuffer ;

//——————————————————————————————————————————————————————————————————————————————
//   SETUP
//——————————————————————————————————————————————————————————————————————————————

void setup () {
  gBuffer.initWithSize (25) ;
//--- Switch on builtin led
  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;
//--- Start serial
  Serial.begin (38400) ;
//--- Wait for serial (blink led at 10 Hz during waiting)
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }
//--- Begin SPI
  SPI.begin () ;
//--- Configure ACAN2515
  Serial.println ("Configure ACAN2515") ;
  ACAN2515TinySettings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515TinySettings::LoopBackMode ; // Select loopback mode
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode == 0) {
    Serial.print ("Bit Rate prescaler: ") ;
    Serial.println (settings.mBitRatePrescaler) ;
    Serial.print ("Propagation Segment: ") ;
    Serial.println (settings.mPropagationSegment) ;
    Serial.print ("Phase segment 1: ") ;
    Serial.println (settings.mPhaseSegment1) ;
    Serial.print ("Phase segment 2: ") ;
    Serial.println (settings.mPhaseSegment2) ;
    Serial.print ("SJW: ") ;
    Serial.println (settings.mSJW) ;
    Serial.print ("Triple Sampling: ") ;
    Serial.println (settings.mTripleSampling ? "yes" : "no") ;
    Serial.print ("Actual bit rate: ") ;
    Serial.print (settings.actualBitRate ()) ;
    Serial.println (" bit/s") ;
    Serial.print ("Exact bit rate ? ") ;
    Serial.println (settings.exactBitRate () ? "yes" : "no") ;
    Serial.print ("Sample point: ") ;
    Serial.print (settings.samplePointFromBitStart ()) ;
    Serial.println ("%") ;
  }else{
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }
}

//——————————————————————————————————————————————————————————————————————————————
//   pseudoRandomValue
//——————————————————————————————————————————————————————————————————————————————

static uint32_t pseudoRandomValue (void) {
  static uint32_t gSeed = 0 ;
  gSeed = 8253729UL * gSeed + 2396403UL ;
  return gSeed ;
}

//——————————————————————————————————————————————————————————————————————————————
//   LOOP
//——————————————————————————————————————————————————————————————————————————————

static uint32_t gBlinkLedDate = 2000 ;
static uint32_t gExtendedFrameCount = 0 ;
static uint32_t gStandardFrameCount = 0 ;
static uint32_t gRemoteFrameCount = 0 ;
static uint32_t gCanDataFrameCount = 0 ;
static uint32_t gReceiveCount = 0 ;

static bool gOk = true ;

//——————————————————————————————————————————————————————————————————————————————

void loop () {
  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 2000 ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
    Serial.print ("Counters: ") ;
    Serial.print (gStandardFrameCount) ;
    Serial.print (", ") ;
    Serial.print (gExtendedFrameCount) ;
    Serial.print (", ") ;
    Serial.print (gCanDataFrameCount) ;
    Serial.print (", ") ;
    Serial.print (gRemoteFrameCount) ;
    Serial.print (", ") ;
    Serial.println (gReceiveCount) ;
  }
//--- Send Message
  if (gOk && (!gBuffer.isFull ()) && can.sendBufferNotFull ()) {
    CANMessage frame ;
    const uint32_t r = pseudoRandomValue () ;
    frame.ext = (r & (1UL << 29)) != 0 ;
    frame.rtr = (r & (1UL << 30)) != 0 ;
    frame.id = r & 0x1FFFFFFFUL ;
    if (frame.ext) {
      gExtendedFrameCount += 1 ;
    }else{
      gStandardFrameCount += 1 ;
      frame.id &= 0x7FFU ;
    }
    frame.len = pseudoRandomValue () % 9 ;
    if (frame.rtr) {
      gRemoteFrameCount += 1 ;
    }else{
      gCanDataFrameCount += 1 ;
      for (uint32_t i=0 ; i<frame.len ; i++) {
        frame.data [i] = uint8_t (pseudoRandomValue ()) ;
      }
    }    
    gBuffer.append (frame) ;
    const bool sendStatusIsOk = can.tryToSend (frame) ;
    if (!sendStatusIsOk) {
      gOk = false ;
      Serial.println ("Send status error") ;
    }
  }
//--- Receive message
  CANMessage receivedFrame ;
  if (gOk && can.receive (receivedFrame)) {
    CANMessage storedFrame ;
    gOk = gBuffer.remove (storedFrame) ;
    if (!gOk) {
      Serial.println ("gBuffer is empty") ;
    }else{    
      gReceiveCount += 1 ;
      bool sameFrames = storedFrame.id == receivedFrame.id ;
      if (sameFrames) {
        sameFrames = storedFrame.ext == receivedFrame.ext ;
      }
      if (sameFrames) {
        sameFrames = storedFrame.rtr == receivedFrame.rtr ;
      }
      if (sameFrames) {
        sameFrames = storedFrame.len == receivedFrame.len ;
      }
      if (!storedFrame.rtr) {
        for (uint32_t i=0 ; (i<receivedFrame.len) && sameFrames ; i++) {
          sameFrames = storedFrame.data [i] == receivedFrame.data [i] ;
        }
      }
      if (!sameFrames) {
        gOk = false ;
        Serial.println ("Receive error") ;
        Serial.print ("  IDF: 0x") ;
        Serial.print (storedFrame.id, HEX) ;
        Serial.print (" :: 0x") ;
        Serial.println (receivedFrame.id, HEX) ;
        Serial.print ("  EXT: ") ;
        Serial.print (storedFrame.ext) ;
        Serial.print (" :: ") ;
        Serial.println (receivedFrame.ext) ;
        Serial.print ("  RTR: ") ;
        Serial.print (storedFrame.rtr) ;
        Serial.print (" :: ") ;
        Serial.println (receivedFrame.rtr) ;
        Serial.print ("  LENGTH: ") ;
        Serial.print (storedFrame.len) ;
        Serial.print (" :: ") ;
        Serial.println (receivedFrame.len) ;     
      }
    }
  }
}

//——————————————————————————————————————————————————————————————————————————————
