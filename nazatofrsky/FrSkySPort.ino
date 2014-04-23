/*
APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
based on ideas found here http://code.google.com/p/telemetry-convert/
******************************************************
Cut board on the backside to separate Vin from VUSB

Connection on Teensy 3.1:
SPort S --> TX1
SPort + --> Vin
SPort  - --> GND

APM Telemetry DF13-5  Pin 2 --> RX2
APM Telemetry DF13-5  Pin 3 --> TX2
APM Telemetry DF13-5  Pin 5 --> GND

Analog input  --> A0 (pin14) on Teensy 3.1 ( max 3.3 V )


This is the data we send to FrSky, you can change this to have your own
set of data
******************************************************
Data transmitted to FrSky Taranis:
Cell           ( Voltage of Cell=Cells/4. [V] This is my LiPo pack 4S ) 
Cells         ( Voltage from LiPo [V] )
A2             ( Analog voltage from input A0 on Teensy 3.1 )
Alt             ( Altitude from baro.  [m] )
GAlt          ( Altitude from GPS   [m])
HDG         ( Compass heading  [deg])
Rpm         ( Throttle when ARMED [%] )
AccX         ( AccX m/s ? )
AccY         ( AccY m/s ? )
AccZ         ( AccZ m/s ? )
VSpd        ( Vertical speed [m/s] )
Speed      ( Ground speed from GPS,  [km/h] )
T1            ( GPS status = ap_sat_visible*10) + ap_fixtype )
T2            ( ARMED=1, DISARMED=0 )
Vfas          ( same as Cells )
Longitud    
Latitud
Dist          ( Will be calculated by FrSky Taranis as the distance from first received lat/long = Home Position

******************************************************
*/

#include "FrSkySPort.h"


#define _FrSkySPort_C1                UART0_C1
#define _FrSkySPort_C3                UART0_C3
#define _FrSkySPort_S2                UART0_S2
#define _FrSkySPort_BAUD           57600
#define   MAX_ID_COUNT              19

short crc;                         // used for crc calc of frsky-packet
uint8_t lastRx;
uint32_t FR_ID_count = 0;
uint8_t cell_count = 0;
uint8_t latlong_flag = 0;
uint32_t latlong = 0;
uint8_t first=0;
// ***********************************************************************
void FrSkySPort_Init(void)  {
      FrSkySPort_Serial.begin(_FrSkySPort_BAUD);
      _FrSkySPort_C3 = 0x10;            // Tx invert
      _FrSkySPort_C1= 0xA0;            // Single wire mode
      _FrSkySPort_S2 = 0x10;           // Rx Invert
      
}

// ***********************************************************************
void FrSkySPort_Process(void) {
	uint8_t data = 0;
        uint32_t temp=0;
	uint8_t offset;
        while ( FrSkySPort_Serial.available()) 
          {
	  data =  FrSkySPort_Serial.read();
          if (lastRx == START_STOP && ((data == SENSOR_ID1) || (data == SENSOR_ID2) || (data == SENSOR_ID3)  || (data == SENSOR_ID4))) 
            {
             
              switch(FR_ID_count) {
                 case 0:
                   if(gps_fixtype==3) {
                     FrSkySPort_SendPackage(FR_ID_SPEED,groundspeed *20 );  // from GPS converted to km/h
                    }
                   break;
                 case 1:
                   //FrSkySPort_SendPackage(FR_ID_RPM,ap_throttle * 2);   //  * 2 if number of blades on Taranis is set to 2
                   break;
                case 2:
                   //FrSkySPort_SendPackage(FR_ID_CURRENT,ap_current_battery / 10); 
                   break; 
               case 3:        // Sends the altitude value from barometer, first sent value used as zero altitude
                  FrSkySPort_SendPackage(FR_ID_ALTITUDE,bar_altitude);   // from barometer, 100 = 1m
                  break;       
                case 4:        // Sends the longitude value, setting bit 31 high
                   if(gps_fixtype==3) {
                       longitude2 = longitude*10000000;
                       if(longitude2 < 0)
                           latlong=((abs(longitude2)/100)*6)  | 0xC0000000;
                           else
                           latlong=((abs(longitude2)/100)*6)  | 0x80000000;
                       FrSkySPort_SendPackage(FR_ID_LATLONG,latlong);
                       }
                   break;
                 case 5:        // Sends the latitude value, setting bit 31 low  
                     if(gps_fixtype==3) {
                         latitude2 = latitude*10000000;
                         if(latitude2 < 0 )
                             latlong=((abs(latitude2)/100)*6) | 0x40000000;
                             else
                             latlong=((abs(latitude2)/100)*6);
                         FrSkySPort_SendPackage(FR_ID_LATLONG,latlong);
                         }
                    break; 
                 case 6:        // Sends the compass heading
                   FrSkySPort_SendPackage(FR_ID_HEADING,heading * 100);   // 10000 = 100 deg MODIFIED
                   break;    
                 case 7:        // Sends the analog value from input A0 on Teensy 3.1
                    //FrSkySPort_SendPackage(FR_ID_ADC2,adc2);                  
                    break;       
                 case 8:        // First 2 cells
                       //temp=((ap_voltage_battery/(ap_cell_count * 2)) & 0xFFF);
                       //FrSkySPort_SendPackage(FR_ID_CELLS,(temp << 20) | (temp << 8));          // Battery cell 0,1
                       break;
                  case 9:    // Optional 3 and 4 Cells
                      //if(ap_cell_count > 2) {
                      //    offset = ap_cell_count > 3 ? 0x02: 0x01;
                      //    temp=((ap_voltage_battery/(ap_cell_count * 2)) & 0xFFF);
                      //    FrSkySPort_SendPackage(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 2,3
                      //    }
                      break;
                 case 10:    // Optional 5 and 6 Cells
                     // if(ap_cell_count > 4) {
                     //     offset = ap_cell_count > 5 ? 0x04: 0x03;
                     //     temp=((ap_voltage_battery/(ap_cell_count * 2)) & 0xFFF);
                     //     FrSkySPort_SendPackage(FR_ID_CELLS,(temp << 20) | (temp << 8) | offset);  // Battery cell 2,3
                     //     }
                      break;     
                 case 11:
                   //FrSkySPort_SendPackage(FR_ID_ACCX,ap_accX_old - ap_accX);    
                     break;
                case 12:
                   //FrSkySPort_SendPackage(FR_ID_ACCY,ap_accY_old - ap_accY); 
                   break; 
                case 13:
                   //FrSkySPort_SendPackage(FR_ID_ACCZ,ap_accZ_old - ap_accZ ); 
                   break; 
                case 14:        // Sends voltage as a VFAS value
                   //FrSkySPort_SendPackage(FR_ID_VFAS,ap_voltage_battery/10); 
                   break;   
                case 15:
                   FrSkySPort_SendPackage(FR_ID_T1,gps_status);  //T1 
                   break; 
                case 16:
                   FrSkySPort_SendPackage(FR_ID_T2,gps_altitude); //Altitude again on T2 for DEBUG
                   break;
               case 17:
                   FrSkySPort_SendPackage(FR_ID_VARIO,climb_rate );       // 100 = 1m/s        
                   break;
               case 18:
                   if(gps_fixtype==3) {
                       gps_altitude2 = gps_altitude*100;
                       FrSkySPort_SendPackage(FR_ID_GPS_ALT,gps_altitude2);   // from GPS,  100=1m
                     }
                   break;
               case 19:
                   //FrSkySPort_SendPackage(FR_ID_FUEL,ap_custom_mode); 
                   break;      
                   
               }
            FR_ID_count++;
            if(FR_ID_count > MAX_ID_COUNT) FR_ID_count = 0;  
            }
          lastRx=data;
          }
}


// ***********************************************************************
void FrSkySPort_SendByte(uint8_t byte) {
	
       FrSkySPort_Serial.write(byte);
	
        // CRC update
	crc += byte;         //0-1FF
	crc += crc >> 8;   //0-100
	crc &= 0x00ff;
	crc += crc >> 8;   //0-0FF
	crc &= 0x00ff;
}


// ***********************************************************************
void FrSkySPort_SendCrc() {
	FrSkySPort_Serial.write(0xFF-crc);
        crc = 0;          // CRC reset
}


// ***********************************************************************
void FrSkySPort_SendPackage(uint16_t id, uint32_t value) {
        digitalWrite(ledpin,HIGH);
        _FrSkySPort_C3 |= 32;      //  Transmit direction, to S.Port
	FrSkySPort_SendByte(DATA_FRAME);
	uint8_t *bytes = (uint8_t*)&id;
	FrSkySPort_SendByte(bytes[0]);
	FrSkySPort_SendByte(bytes[1]);
	bytes = (uint8_t*)&value;
	FrSkySPort_SendByte(bytes[0]);
	FrSkySPort_SendByte(bytes[1]);
	FrSkySPort_SendByte(bytes[2]);
	FrSkySPort_SendByte(bytes[3]);
	FrSkySPort_SendCrc();
	FrSkySPort_Serial.flush();
	_FrSkySPort_C3 ^= 32;      // Transmit direction, from S.Port
        digitalWrite(ledpin,LOW);
        
}
