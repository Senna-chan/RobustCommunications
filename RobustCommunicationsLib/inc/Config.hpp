#ifndef _ROBUSTCOMMUNICATIONSCONFIG_H
#define _ROBUSTCOMMUNICATIONSCONFIG_H

#include <stdint.h>
namespace RobustCommunications{
/**
 * Size of all the information bytes used.
 */
static const uint16_t MaxDataBytes = 512;

static const uint16_t definitionSize = 300;

static const uint8_t maxFormatsDefined = 50;

static const char charPacketSeperator = '|';

#ifndef ROBUSTCOMMSDEBUG
    #define ROBUSTCOMMSDEBUG 0
#endif
#ifndef ROBUSTCOMMSTRACE
    #define ROBUSTCOMMSTRACE 0
#endif

#if ROBUSTCOMMSDEBUG == 1
#if ARDUINO
 #include <Arduino.h>
 #define DEBUGPRINTF(f_, ...) Serial.printf((f_), ##__VA_ARGS__) 
 #define TRACEPRINTF(f_, ...) Serial.printf((f_), ##__VA_ARGS__) 
#elif PIO_UNIT_TESTING
 #include <unity.h>
 #define DEBUGPRINTF TEST_PRINTF((f_), ##__VA_ARGS__) 
 #define TRACEPRINTF TEST_PRINTF((f_), ##__VA_ARGS__) 
#else
 #include <stdio.h>
 #define DEBUGPRINTF (f_, ...) printf((f_), ##__VA_ARGS__)
 #define TRACEPRINTF (f_, ...) printf((f_), ##__VA_ARGS__)
#endif
#else
 #define DEBUGPRINTF(f_, ...) //((f_), ##__VA_ARGS__)
#endif
#if ROBUSTCOMMSTRACE == 0
 #ifdef TRACEPRINTF
  #undef TRACEPRINTF
 #endif
 #define TRACEPRINTF(f_, ...) //((f_), ##__VA_ARGS__)
#endif
}
#endif