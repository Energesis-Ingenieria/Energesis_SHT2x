/**
 * @file Energesis_Si7021.cpp
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief Implementación del controlador para los sensores Si7021
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Energesis_Si7021.h"

#define SI7021_CMD_SERIAL_FIRST_0   0xFA
#define SI7021_CMD_SERIAL_FIRST_1   0x0F

#define SI7021_CMD_SERIAL_SECOND_0  0xFC
#define SI7021_CMD_SERIAL_SECOND_1  0xC9

#define SI7021_CMD_FIRMWARE_REV_0   0x84
#define SI7021_CMD_FIRMWARE_REV_1   0xB8


namespace Energesis{

class Energesis_Si7021_Temp: public Energesis_Sensor{
public:
  Energesis_Si7021_Temp( Energesis_Si7021 *sens ){ m_si7021 = sens; }

  bool getSample(sensor_sample_t *sample){
    sample->type = SENSOR_TYPE_TEMPERATURE;
    sample->temperature = m_si7021->getTemperature();
    sample->timestamp = millis();
    if( sample->temperature == INFINITY )
      return false;
    
    return true;
  }

  void getSensorDetails( sensor_details_t *details ){
    details->max_value = 125; // ºC
    details->min_delay = 50;  // ms
    details->min_value = -40; // ºC
    m_si7021->getName( details->name, sizeof(details->name) );
    details->sensor_id = m_si7021->getSerialNumber();
    details->power = 0.500;   // mA

    details->resolution = 0.04; // ºC
    details->type = SENSOR_TYPE_TEMPERATURE;
    details->version = ENERGESIS_SI702X_VERSION_MAJOR | ( m_si7021->getFirmwareRevision()<<16 );
  }

private:
  Energesis_Si7021 *m_si7021 = NULL;
  
};

class Energesis_Si7021_Humidity: public Energesis_Sensor{
public:
  Energesis_Si7021_Humidity( Energesis_Si7021 *sens ){ m_si7021 = sens; }

  bool getSample(sensor_sample_t *sample){
    sample->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    sample->temperature = m_si7021->getTemperature();
    sample->timestamp = millis();
    if( sample->temperature == INFINITY )
      return false;
    
    return true;
  }

  void getSensorDetails( sensor_details_t* details ){
    details->max_value = 100; // %RH
    details->min_delay = 50;  // ms
    details->min_value = 0;   // %RH
    m_si7021->getName( details->name, sizeof(details->name) );
    details->power = 0.500;   // mA

    details->resolution = 0.04; // %RH
    details->sensor_id = m_si7021->getSerialNumber();
    details->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    details->version = ENERGESIS_HTU2X_VERSION_MAJOR | (m_si7021->getFirmwareRevision()<<16);    
  }

private:
  Energesis_Si7021 *m_si7021 = NULL;

};



uint64_t Energesis_Si7021::getSerialNumber(){
  // union para facilitar la composicion del número
  union{
    uint8_t bytes[8];
    uint64_t serial;
  } res;
  
  m_tx_buff[0] = SI7021_CMD_SERIAL_FIRST_0;
  m_tx_buff[1] = SI7021_CMD_SERIAL_FIRST_1;

  if( !m_device->write( m_tx_buff, 2, false ) ){
    Serial.println( "[E] Intentando solicitar SERIAL FIRST" );
    return 0x00;
  }

  if( !m_device->read( m_rx_buff, 8, true ) ){
    Serial.println( "[E] Intentando leer SERIAL FIRST" );
    return 0x00;
  }

  res.bytes[7] = m_rx_buff[0];
  res.bytes[6] = m_rx_buff[2];
  res.bytes[5] = m_rx_buff[4];
  res.bytes[4] = m_rx_buff[6];
  
  m_tx_buff[0] = SI7021_CMD_SERIAL_SECOND_0;
  m_tx_buff[1] = SI7021_CMD_SERIAL_SECOND_1;

  if( !m_device->write( m_tx_buff, 2, false ) )
    return 0x00;

  if( !m_device->read( m_rx_buff, 8, true ) ){
    return 0x00;
  }

  res.bytes[ 3 ] = m_rx_buff[ 0 ];
  res.bytes[ 2 ] = m_rx_buff[ 1 ];
  res.bytes[ 1 ] = m_rx_buff[ 3 ];
  res.bytes[ 0 ] = m_rx_buff[ 4 ];

  return res.serial;
}

uint8_t Energesis_Si7021::getFirmwareRevision(){

  m_tx_buff[0] = SI7021_CMD_FIRMWARE_REV_0;
  m_tx_buff[1] = SI7021_CMD_FIRMWARE_REV_1;
  
  if( !m_device->write( m_tx_buff, 2, false ) )
    return 0;
  
  if( !m_device->read( m_rx_buff, 1 ) )
    return 0;

  return m_rx_buff[0];
}

size_t Energesis_Si7021::getName( char *name, size_t capacity ){
  uint64_t sn = getSerialNumber();
  uint8_t snb3 = ((uint8_t *)(&sn))[3];
  switch ( snb3 )
  {
  case 0x00:
  case 0xFF:
    return strlcpy(name, "Si70xx engr samples", capacity);
    break;

  case 0x0D: // 13
  case 0x14: // 20
  case 0x15: // 21
    return snprintf( name, capacity, "Si70%d", snb3 );
    break;

  default:
    return snprintf( name, capacity, "Not Si70xx" );
    break;
  }
}

Energesis_Sensor* Energesis_Si7021::getTemperatureSensor(){
  if( !m_temperature_sensor )
    m_temperature_sensor = new Energesis_Si7021_Temp(this);
  return m_temperature_sensor;
}

Energesis_Sensor* Energesis_Si7021::getRelativeHumiditySensor(){
  if( !m_humidity_sensor )
    m_humidity_sensor = new Energesis_Si7021_Humidity(this);
  return m_humidity_sensor;
}


};

