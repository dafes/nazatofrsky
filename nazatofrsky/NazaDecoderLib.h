/*
  DJI Naza (v1, v1 Lite, V2) data decoder library
  (c) Pawelsky 20140123
   Not for commercial use!
  If you want to use it in comercial projects contact:
  http://www.rcgroups.com/forums/member.php?u=393936

  Refer to naza_decoder_wiring.jpg diagram for proper connection

. The RC PWM input code taken from https://www.instructables.com/id/RC-Quadrotor-Helicopter/step12/Arduino-Demo-PWM-Input/
Added some snipets from Airmamaf (Bagaosd https://code.google.com/p/bagaosd/wiki/NazaConnector )
*/

#ifndef NazaDecoderLib_h
#define NazaDecoderLib_h

#include "Arduino.h"

#define NAZA_MESSAGE_NONE    0x00
#define NAZA_MESSAGE_GPS     0x10
#define NAZA_MESSAGE_COMPASS 0x20

class NazaDecoderLib
{
  public:
    typedef enum { NO_FIX = 0, FIX_2D = 2, FIX_3D = 3, FIX_DGPS = 4 } fixType_t;

    NazaDecoderLib();
    //void   pwmInterruptHandler();

    uint8_t decode(int input);
    double getLat();
    double getLon();
    double getAlt();
    double getSpeed();
    double getClimbSpeed();
    fixType_t getFixType();
    uint8_t getNumSat();
    double getHeading();
    double getCog();
    double getHdop();
    int8_t getPitch();
    int8_t getRoll();
    uint8_t getYear();
    uint8_t getMonth();
    uint8_t getDay();
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();

  private:
    int gpsPayload[58];
    int seq;
    int cnt;
    int msgId;
    int msgLen;
    uint8_t cs1; // checksum #1
    uint8_t cs2; // checksum #2
    int16_t magXMin;
    int16_t magXMax;
    int16_t magYMin;
    int16_t magYMax;

    typedef struct
    {
      double lon;     // longitude in degree decimal
      double lat;     // latitude in degree decimal
      double alt;     // altitude in m
      double spd;     // speed in m/s
      double clb; 
      fixType_t fix;   // fix type
      uint8_t sat;     // number of satellites
      double heading; // heading in degrees
      double cog;     // course over ground
      double hdop;    // horizontal dilution of precision
      uint8_t year;
      uint8_t month;
      uint8_t day;
      uint8_t hour;
      uint8_t minute;
      uint8_t second;
    } tGpsData;  
    tGpsData gpsData;

    /*typedef struct
    {
      int8_t   edge;
      uint32_t riseTime;
      uint32_t fallTime;
      uint32_t lastGoodWidth;
    } tPwmData;
    tPwmData pwmData[2];

    volatile uint8_t pcIntLast; */

    int32_t  decodeLong(uint8_t idx, uint8_t mask);
    int16_t  decodeShort(uint8_t idx, uint8_t mask);
    //void     startPwmReader();
    //int8_t   pwm2Deg(uint32_t pulseWidth);
    void     updateCS(int input);
};

extern NazaDecoderLib NazaDecoder;

#endif

