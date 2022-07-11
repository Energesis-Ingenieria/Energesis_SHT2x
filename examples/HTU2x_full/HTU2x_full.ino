#include <Arduino.h>
#include <Energesis_HTU2xD.h>

using namespace Energesis;

Energesis_HTU2x sensor;

void setup() {

  Serial.begin(9600);
  delay(500);

  //sht21.begin( SDA, SCL, &Wire);
  
  if( sensor.begin( ) ){
    Serial.println( "SHT2X.....OK" );
  }
  else{
    Serial.println( "SHT2X.....ERROR" );
    while(1);
  }
  
  Serial.println( "-----------------------------" );
  Serial.print( "Serial Number: " );
  Serial.println( sensor.getSerialNumber(), HEX );

  Serial.println( "-----------------------------" );
  Serial.print("Resolución de temperatura: ");
  Serial.println( sensor.getTemperatureResolution() );

  Serial.print("Resolución de humedad relativa: ");
  Serial.println( sensor.getHumidityResolution() );

  sensor.setResolution( RESOLUTION_RH_10BITS_T_13BITS );

  Serial.print("Nueva resolución de temperatura: ");
  Serial.println( sensor.getTemperatureResolution() );

  Serial.print("Nueva resolución de humedad relativa: ");
  Serial.println( sensor.getHumidityResolution() );

  Serial.println( "-----------------------------" );
  Serial.print("Estado de Heater: ");
  Serial.println( sensor.isHeaterOn() );
  
  sensor.heater( true );
  Serial.print("Estado de Heater: ");
  Serial.println( sensor.isHeaterOn() );
  
  sensor.heater( false );
  Serial.print("Estado de Heater: ");
  Serial.println( sensor.isHeaterOn() );
  Serial.println( "-----------------------------" );

}

void loop() {

  Serial.print( "Temperatura: " );
  Serial.print( sensor.getTemperature() );
  Serial.println(" ºC");

  Serial.print( "Humedad: " );
  Serial.print( sensor.getRelativeHumidity() );
  //Serial.print( sht21.getHumidity() );
  Serial.println(" %");


  delay(2000);
}