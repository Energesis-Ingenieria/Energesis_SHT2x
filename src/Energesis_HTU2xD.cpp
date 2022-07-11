/**
 * @file Energesis_HTU2.cpp
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief Implementación del controlador para la serie de sensores
 * HTU2x.
 * @note La clase Energesis_HTU2x hereda de Energesis_SHT2x.
 * Por lo tanto el resto de las funciones pertenecen a dicha clase.
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Energesis_HTU2xD.h"


//-------------------------------------
// Macros para la obtención del número de serie. 
// Ver https://www.amsys.de/downloads/notes/HTU2X-Serial-Number-reading-AMSYS-001e.pdf
//------------------------------------- 

#define HTU2X_CMD_SERIAL_FIRST_0  0xFA //!< Involucrado en la secuencia para la obtención del número de serie del sensor
#define HTU2X_CMD_SERIAL_FIRST_1  0x0F //!< Involucrado en la secuencia para la obtención del número de serie del sensor

#define HTU2x_SNB_3_READ_POS      0 //!< Posición del byte SNB_3 en el buffer de lectura
#define HTU2x_SNB_2_READ_POS      2 //!< Posición del byte SNB_2 en el buffer de lectura
#define HTU2x_SNB_1_READ_POS      4 //!< Posición del byte SNB_1 en el buffer de lectura
#define HTU2x_SNB_0_READ_POS      6 //!< Posición del byte SNB_0 en el buffer de lectura

#define HTU2x_SNB_3_SERIAL_POS    5 //!< Posición del byte SNB_3 en el número de serie 
#define HTU2x_SNB_2_SERIAL_POS    4 //!< Posición del byte SNB_2 en el número de serie
#define HTU2x_SNB_1_SERIAL_POS    3 //!< Posición del byte SNB_1 en el número de serie
#define HTU2x_SNB_0_SERIAL_POS    2 //!< Posición del byte SNB_0 en el número de serie

#define HTU2X_CMD_SERIAL_SECOND_0 0xFC  //!< Involucrado en la secuencia para la obtención del número de serie del sensor
#define HTU2X_CMD_SERIAL_SECOND_1 0xC9  //!< Involucrado en la secuencia para la obtención del número de serie del sensor

#define HTU2x_SNC_1_READ_POS      0 //!< Posición del byte SNC_1 en el buffer de lectura
#define HTU2x_SNC_0_READ_POS      1 //!< Posición del byte SNC_0 en el buffer de lectura

#define HTU2x_SNC_1_SERIAL_POS    1 //!< Posición del byte SNC_1 en el número de serie
#define HTU2x_SNC_0_SERIAL_POS    0 //!< Posición del byte SNC_0 en el número de serie

#define HTU2x_SNA_1_READ_POS      3 //!< Posición del byte SNA_1 en el buffer de lectura
#define HTU2x_SNA_0_READ_POS      4 //!< Posición del byte SNA_0 en el buffer de lectura

#define HTU2x_SNA_1_SERIAL_POS    7 //!< Posición del byte SNA_1 en el número de serie
#define HTU2x_SNA_0_SERIAL_POS    6 //!< Posición del byte SNA_0 en el número de serie


namespace Energesis{

/**
 * @brief 
 * 
 */
class Energesis_HTU2x_Temp: public Energesis_Sensor{
public:
  Energesis_HTU2x_Temp( Energesis_HTU2x *sens ){ m_htu2x = sens; }

  bool getSample(sensor_sample_t *sample){
    sample->type = SENSOR_TYPE_TEMPERATURE;
    sample->temperature = m_htu2x->getTemperature();
    sample->timestamp = millis();
    if( sample->temperature == INFINITY )
      return false;
    
    return true;
  }

  void getSensorDetails( sensor_details_t *details ){
    details->max_value = 125; // ºC
    details->min_delay = 50;  // ms
    details->min_value = -40; // ºC
    strlcpy(details->name, "HTU2xD", sizeof("HTU2xD"));
    details->power = 0.500;   // mA

    details->resolution = 0.04; // ºC
    details->sensor_id = m_htu2x->getSerialNumber();
    details->type = SENSOR_TYPE_TEMPERATURE;
    details->version = ENERGESIS_HTU2X_VERSION_MAJOR;
  }

private:
  Energesis_HTU2x *m_htu2x = NULL;
  
};



class Energesis_HTU2x_Humidity: public Energesis_Sensor{
public:
  Energesis_HTU2x_Humidity( Energesis_HTU2x *sens ){ m_htu2x = sens; }

  bool getSample(sensor_sample_t *sample){
    sample->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    sample->temperature = m_htu2x->getTemperature();
    sample->timestamp = millis();
    if( sample->temperature == INFINITY )
      return false;
    
    return true;
  }

  void getSensorDetails( sensor_details_t* details ){
    details->max_value = 100; // %RH
    details->min_delay = 50;  // ms
    details->min_value = 0;   // %RH
    strlcpy(details->name, "HTU2xD", sizeof("HTU2xD"));
    details->power = 0.500;   // mA

    details->resolution = 0.04; // %RH
    details->sensor_id = m_htu2x->getSerialNumber();
    details->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
    details->version = ENERGESIS_HTU2X_VERSION_MAJOR;    
  }

private:
  Energesis_HTU2x *m_htu2x = NULL;

};



uint64_t Energesis_HTU2x::getSerialNumber(){

  // union para facilitar la composicion del número
  union{
    uint8_t bytes[8];
    uint64_t serial;
  } res;
  
  m_tx_buff[0] = HTU2X_CMD_SERIAL_FIRST_0;
  m_tx_buff[1] = HTU2X_CMD_SERIAL_FIRST_1;

  if( !m_device->write( m_tx_buff, 2, false ) ){
    Serial.println( "[E] Intentando solicitar SERIAL FIRST" );
    return 0x00;
  }

  if( !m_device->read( m_rx_buff, 8, true ) ){
    Serial.println( "[E] Intentando leer SERIAL FIRST" );
    return 0x00;
  }

  res.bytes[ HTU2x_SNB_3_SERIAL_POS ] = m_rx_buff[ HTU2x_SNB_3_READ_POS ];
  res.bytes[ HTU2x_SNB_2_SERIAL_POS ] = m_rx_buff[ HTU2x_SNB_2_READ_POS ];
  res.bytes[ HTU2x_SNB_1_SERIAL_POS ] = m_rx_buff[ HTU2x_SNB_1_READ_POS ];
  res.bytes[ HTU2x_SNB_0_SERIAL_POS ] = m_rx_buff[ HTU2x_SNB_0_READ_POS ];

  m_tx_buff[0] = HTU2X_CMD_SERIAL_SECOND_0;
  m_tx_buff[1] = HTU2X_CMD_SERIAL_SECOND_1;

  if( !m_device->write( m_tx_buff, 2, false ) )
    return 0x00;

  if( !m_device->read( m_rx_buff, 8, true ) ){
    return 0x00;
  }

  res.bytes[ HTU2x_SNA_1_SERIAL_POS ] = m_rx_buff[ HTU2x_SNA_1_READ_POS ];
  res.bytes[ HTU2x_SNA_0_SERIAL_POS ] = m_rx_buff[ HTU2x_SNA_0_READ_POS ];
  res.bytes[ HTU2x_SNC_1_SERIAL_POS ] = m_rx_buff[ HTU2x_SNC_1_READ_POS ];
  res.bytes[ HTU2x_SNC_0_SERIAL_POS ] = m_rx_buff[ HTU2x_SNC_0_READ_POS ];

  return res.serial;
}


Energesis_Sensor* Energesis_HTU2x::getTemperatureSensor(){
  if( !m_temperature_sensor )
    m_temperature_sensor = new Energesis_HTU2x_Temp( this );
  return m_temperature_sensor;
}

Energesis_Sensor* Energesis_HTU2x::getRelativeHumiditySensor(){
  if( !m_humidity_sensor )
    m_humidity_sensor = new Energesis_HTU2x_Humidity( this );
  return m_humidity_sensor;
}


}; // namespace
