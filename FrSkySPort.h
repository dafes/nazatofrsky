/*
APM2.5 Mavlink to FrSky X8R SPort interface using Teensy 3.1  http://www.pjrc.com/teensy/index.html
based on ideas found here http://code.google.com/p/telemetry-convert/
******************************************************
Cut board on the backside to separate Vin from VUSB

Connection on Teensy 3.1:
SPort S --> TX1
SPort + --> Vin
SPort  - --> GND

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

// Frsky Sensor-ID to use. 
#define SENSOR_ID1                   0x1B // ID of sensor. Must be something that is polled by FrSky RX
#define SENSOR_ID2                   0x0D
#define SENSOR_ID3                   0x34
#define SENSOR_ID4                   0x67
// Frsky-specific
#define START_STOP               0x7e
#define DATA_FRAME               0x10


//Frsky DATA ID's 
#define FR_ID_SPEED               0x0830 
#define FR_ID_VFAS                 0x0210 
#define FR_ID_CURRENT         0x0200 
#define FR_ID_RPM                  0x050F      
#define FR_ID_ALTITUDE         0x10
#define FR_ID_FUEL                 0x0600   
#define FR_ID_ADC1                0xF102   
#define FR_ID_ADC2                0xF103      
#define FR_ID_LATLONG         0x0800
#define FR_ID_CAP_USED       0x0600
#define FR_ID_VARIO               0x0110
#define FR_ID_CELLS              0x0300     
#define FR_ID_CELLS_LAST   0x030F      
#define FR_ID_HEADING          0x0840
#define FR_ID_ACCX               0x0700
#define FR_ID_ACCY               0x0710
#define FR_ID_ACCZ               0x0720
#define FR_ID_T1                     0x0400
#define FR_ID_T2                     0x0410
#define FR_ID_GPS_ALT          0x0820


