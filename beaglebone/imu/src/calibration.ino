#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI    // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2        // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire  // Your desired Wire port.      Used when "USE_SPI" is not defined
#define AD0_VAL   1     // The value of the last bit of the I2C address. 
                        // On the SparkFun 9DoF IMU breakout the default is 1, and when 
                        // the ADR jumper is closed the value becomes 0

#ifdef USE_SPI
  ICM_20948_SPI myICM;  // If using SPI create an ICM_20948_SPI object
#else
  ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object
#endif


void setup() {
  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT){};

#ifdef USE_SPI
    SPI_PORT.begin();
#else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
#endif
  
  bool initialized = false;
  while( !initialized ){

#ifdef USE_SPI
    myICM.begin( CS_PIN, SPI_PORT ); 
#else
    myICM.begin( WIRE_PORT, AD0_VAL );
#endif

    SERIAL_PORT.print( F("Initialization of the sensor returned: ") );
    SERIAL_PORT.println( myICM.statusString() );
    if( myICM.status != ICM_20948_Stat_Ok ){
      SERIAL_PORT.println( "Trying again..." );
      delay(500);
    }else{
      initialized = true;
    }
  }
}

void loop() {
  
  if( myICM.dataReady() ){
    myICM.getAGMT();  
    Serial.print("Raw:");
    
    Serial.print(myICM.agmt.acc.axes.x);
    Serial.print(',');
    Serial.print(myICM.agmt.acc.axes.y);
    Serial.print(',');
    Serial.print(myICM.agmt.acc.axes.z);
    Serial.print(',');
    Serial.print(myICM.agmt.gyr.axes.x);
    Serial.print(',');
    Serial.print(myICM.agmt.gyr.axes.y);
    Serial.print(',');
    Serial.print(myICM.agmt.gyr.axes.z);
    Serial.print(',');
    Serial.print(myICM.agmt.mag.axes.x);
    Serial.print(',');
    Serial.print(myICM.agmt.mag.axes.y);
    Serial.print(',');
    Serial.print(myICM.agmt.mag.axes.z);
    Serial.println();
    
    delay(50);
    //originally 30
  }else{
    Serial.println("Waiting for data");
    delay(500);
  }
}
  