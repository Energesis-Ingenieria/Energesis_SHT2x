/**
 * @file HTU2x_generic.ino
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief Se muestra como utilizar la interfaz genérica con un sensor de 
 * la serie HTU2xD.
 * @version 0.1
 * @date 2022-05-23
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>
#include <Energesis_HTU2xD.h>

using namespace Energesis;

Energesis_HTU2x sensor;
Energesis_Sensor* sens_arr[2];

sensor_details_t det;

const char *type2str( sensor_type_t t ){
  switch (t)
  {
  case SENSOR_TYPE_TEMPERATURE: return "Temperature";
  case SENSOR_TYPE_CURRENT:     return "Current";
  case SENSOR_TYPE_RELATIVE_HUMIDITY: return "Relative Humidity";
  case SENSOR_TYPE_VOLTAGE: return "Voltage";
  default: return "Invalid";

  }
}

const char *type2unit( sensor_type_t t ){
  switch (t)
  {
  case SENSOR_TYPE_TEMPERATURE: return "ºC";
  case SENSOR_TYPE_CURRENT:     return "mA";
  case SENSOR_TYPE_RELATIVE_HUMIDITY: return "%";
  case SENSOR_TYPE_VOLTAGE: return "V";
  default: return "Invalid";

  }
}

void printDetails (sensor_details_t &d){
  Serial.println("---------------------------------");
  Serial.print( "Name: " );
  Serial.println( String(d.name) + " -> " + type2str(d.type) );

  Serial.print( "Serial Number: " );
  Serial.println( d.sensor_id, HEX );
  
  Serial.print( "Max value: " );
  Serial.print( d.max_value);
  Serial.println( type2unit( d.type ) );

  Serial.print( "Min value: " );
  Serial.print( d.min_value);
  Serial.println( type2unit( d.type ) );

  Serial.print( "Current consum[mA]: " );
  Serial.println( d.power );

  Serial.print( "Resolution: " );
  Serial.print( d.resolution );
  Serial.println( type2unit( d.type ) );

  Serial.println("-----------------------------------------");

}

void setup() {

  Serial.begin(9600);
  delay(500);
  // Init sensor
  if( sensor.begin( ) ){
    Serial.println( "HTU2x.....OK" );
  }
  else{
    Serial.println( "HTU2x.....ERROR" );
    while(1);
  }
  // Get each sensor
  sens_arr[0] = sensor.getTemperatureSensor();
  sens_arr[1] = sensor.getRelativeHumiditySensor();

  // Print sensor's details
  for( int i = 0 ; i < 2; i++ ){
    sens_arr[i]->getSensorDetails( &det );
    printDetails(det);
  }

}

void loop() {
  sensor_sample_t sample;

  for( int i = 0; i < 2; i++ ){
    // Get sample of i-th sensor
    sens_arr[i]->getSample(&sample);

    // Print Type of sensor
    Serial.print( type2str( sample.type ) );
    Serial.print(": ");
    // Use valueFloat for any float value (temperature, humidity, voltage, current)
    Serial.print( sample.valueFloat );
    // Print unit
    Serial.println(String(" ") + type2unit(sample.type));
  }

  delay(2000);
}