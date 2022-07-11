/**
 * @file Energesis_SHT2x.cpp
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief Implementación del controlador de la serie SHT2x de sensores
 * de temperatura y humedad relativa.
 * @version 0.1
 * @date 2022-05-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Energesis_SHT2x.h"

#define DEBUG Serial

namespace Energesis{
// Implementaciones de Energesis_SHT2x_Temp 

void Energesis_SHT2x_Temp::getSensorDetails( sensor_details_t* details ){
  uint8_t res = m_sht2x->getTemperatureResolution();
  details->max_value = 120;
  details->min_delay = 0;
  details->min_value = -40;
  strlcpy(details->name, "SHT2x", sizeof("SHT2x"));
  details->power = 0.330;

  if( res == 14 ) details->resolution = 0.01;
  else if( res == 12 ) details->resolution = 0.04;
  details->sensor_id = 0;
  details->type = SENSOR_TYPE_TEMPERATURE;
  details->version = ENERGESIS_SHT2X_VERSION_MAJOR;
}

bool Energesis_SHT2x_Temp::getSample( sensor_sample_t* sample ){

  sample->type = SENSOR_TYPE_TEMPERATURE;
  sample->sensor_id = 0;
  sample->temperature = m_sht2x->getTemperature();
  sample->timestamp = m_sht2x->m_last_read_temperature;

  if( sample->temperature == INFINITY )
    return false;

  return true;
}

// Implementaciones de Energesis_SHT2x_Humidity 
void Energesis_SHT2x_Humidity::getSensorDetails( sensor_details_t* details ){
  uint8_t res = m_sht2x->getHumidityResolution();
  details->max_value = 120;
  details->min_delay = 0;
  details->min_value = -40;
  strlcpy(details->name, "SHT2x", sizeof("SHT2x"));
  details->power = 0.330;

  if( res == 12 ) details->resolution = 0.04;
  else if( res == 8 ) details->resolution = 0.7;
  details->sensor_id = 0;
  details->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  details->version = ENERGESIS_SHT2X_VERSION_MAJOR;  
}

bool Energesis_SHT2x_Humidity::getSample( sensor_sample_t* sample ){
  sample->type = SENSOR_TYPE_RELATIVE_HUMIDITY;
  sample->sensor_id = 0;
  sample->humidity = m_sht2x->getRelativeHumidity();
  sample->timestamp = m_sht2x->m_last_read_humidity;

  if( sample->humidity == INFINITY )
    return false;

  return true;
}


Energesis_SHT2x::Energesis_SHT2x(){
  m_last_read_humidity = 0;
  m_last_read_temperature = 0;
  m_begun = false;

}

#if defined(ESP8266) || defined(ESP32)
bool Energesis_SHT2x::begin( int sda, int scl, TwoWire *wire ){
  if( m_begun )
    return false;
  // Crear bus
  if( !m_device )
    m_device = new Energesis_I2CDevice( SHT2x_ADDRESS, sda, scl, wire );

  return begin( wire );
}
#endif

bool Energesis_SHT2x::begin(TwoWire *wire ){
  
  if( m_begun ){
    DEBUG.println("[E] El sensor ya está inicializado");
    return false;
  }

  // Crear bus
  if( !m_device )
    m_device = new Energesis_I2CDevice( SHT2x_ADDRESS, wire );

  if( !m_device ){
    DEBUG.println( "[E] No se creó la instancia de I2CDevice" );
    return false;
  }

  // Inicializar bus
  if( !m_device->begin() ){
    DEBUG.println( "[E] No se inicializó el bus" );
    delete m_device;
    return false;
  }

  // comprobar que esté conectado
  if( !m_device->isConnected() ){
    DEBUG.println( "[E] Dispositivo no detectado" );
    return false;
  }

  if( !reset() ){
    DEBUG.println( "[E] No se pudo reiniciar el sensor" );
    return false;
  }


  m_begun = true;
  return true;
}


bool Energesis_SHT2x::reset(){
  m_tx_buff[0] = SHT2x_CMD_SOFT_RESET;
  if( !m_device->write( m_tx_buff, 1  ) )
    return false;

  return true;
}

bool Energesis_SHT2x::measureTemperature(){
  m_tx_buff[0] = SHT2x_CMD_TRIGGER_T_MEASUREMENT;
  if( !m_device->write( m_tx_buff, 1 ) ){
    DEBUG.println( "[E] Al enviar orden de medición de temperatura." );
    return false;
  }
  delayMicroseconds(20);
  return true;
}

float Energesis_SHT2x::readTemperature(){
  uint16_t raw;
  if( !m_device->read( m_rx_buff, 3 ) ){
    DEBUG.println( "[E] No se pudo leer la temperatura" );
    return INFINITY;
  }
  
  ///@todo Agregar chequeo de crc

  if( m_rx_buff[1] & 0x02 )
    return INFINITY;

  raw = m_rx_buff[0]<<8;
  raw += m_rx_buff[1];

  if( !checkCRC( raw, m_rx_buff[2] ) )
    return INFINITY;

  raw &= 0xFFFC;

  m_last_read_temperature = millis();
  m_last_temperature = -46.85 + 175.72*(raw)/(1LL<<16);
  return m_last_temperature;
}

#define POLYNOMIAL_DIVISIOR 0x0131 // = x^8 + X^5 + X^4 + 1 
#define POLYNOMIL_MAX_COEF  8

bool Energesis_SHT2x::checkCRC( uint16_t data, uint8_t crc ){
  
  // encadenar los datos con el crc  
  uint32_t remainder = ((uint32_t)data << 8) | crc;

  //  ... <<( <last_data_bit> + extra_shift_for_crc - POLYNOMIL_MAX_COEF )
  uint32_t divsor = (uint32_t)POLYNOMIAL_DIVISIOR<<( (16-1) + 8 - POLYNOMIL_MAX_COEF );

  for (int i = 0 ; i < 16 ; i++)
  {

    //Serial.printf(remainder, HEX);
    //Serial.print(divsor, HEX);
    //Serial.print("-------------------\n");
    if ( remainder & (uint32_t)1 << (23 - i) )
      remainder ^= divsor;

    divsor >>= 1;
  }    

  return ((uint8_t)remainder) == 0;
}

float Energesis_SHT2x::getTemperature(){
  if( !measureTemperature() ){
    return INFINITY;
  }
  //delay(100);
  return readTemperature();
}

float Energesis_SHT2x::getRelativeHumidity(){
  if( !measureRelativeHumidity() )
    return INFINITY;
  
  return readRelativeHumidity();
}


bool Energesis_SHT2x::measureRelativeHumidity(){
  m_tx_buff[0] = SHT2x_CMD_TRIGGER_RH_MEASUREMENT;

  if( !m_device->write( m_tx_buff, 1 ) ){
    DEBUG.println( "[E] Al enviar orden de medición de humedad." );
    return false;
  }
  delayMicroseconds(20); 
  return true;
}

float Energesis_SHT2x::readRelativeHumidity(){
  uint16_t raw;

  if( !m_device->read( m_rx_buff, 3 ) ){
    DEBUG.println( "[E] No se pudo leer la humedad" );
    return INFINITY;
  }
  
  if( !(m_rx_buff[1] & 0x02) ){
    DEBUG.println("[E] Al verificar los datos recibidos");
    return INFINITY;
  }

  raw = m_rx_buff[0]<<8;
  raw += m_rx_buff[1];

  if( !checkCRC( raw, m_rx_buff[2] ) )
    return INFINITY;

  raw &= 0XFFFC;

  m_last_read_humidity = millis();

  m_last_humidity = -6 + 125.0*(raw)/(1LL<<16);
  return m_last_humidity;
}

bool Energesis_SHT2x::setResolution( SHT2x_RESOLUTION res ){
  uint8_t reg;
  
  if( !getUserRegister(&reg) )
    return false;

  reg &= ~SHT2x_RESOLUTION_BITMASK;
  reg |= static_cast<uint8_t>(res);

  if( !setUserRegister(reg) )
    return false;

  return true;
}

uint8_t Energesis_SHT2x::getTemperatureResolution(){
  uint8_t reg;

  if( !getUserRegister(&reg) )
    return 0;

  reg &= SHT2x_RESOLUTION_BITMASK;

  switch ( static_cast<SHT2x_RESOLUTION>(reg) ) {
    case RESOLUTION_RH_10BITS_T_13BITS: return 13;
    case RESOLUTION_RH_11BITS_T_11BITS: return 11;
    case RESOLUTION_RH_12BITS_T_14BITS: return 14;
    case RESOLUTION_RH_8BITS_T_12BITS : return 12;
    default: return 0;
  }

  return 0;
}

uint8_t Energesis_SHT2x::getHumidityResolution(){
  uint8_t reg;

  if( !getUserRegister(&reg) )
    return 0;

  reg &= SHT2x_RESOLUTION_BITMASK;

  switch ( static_cast<SHT2x_RESOLUTION>(reg) ) {
    case RESOLUTION_RH_10BITS_T_13BITS: return 10;
    case RESOLUTION_RH_11BITS_T_11BITS: return 11;
    case RESOLUTION_RH_12BITS_T_14BITS: return 12;
    case RESOLUTION_RH_8BITS_T_12BITS : return 8;
    default: return 0;
  }

  return 0;
}

bool Energesis_SHT2x::heater(bool on){
  uint8_t reg;

  if( !getUserRegister(&reg) ){
    return false;
  }

  reg &= ~SHT2x_HEATER_BITMASK;

  if( on )
    reg |= SHT2x_HEATER_BITMASK;

  if( !setUserRegister(reg) )
    return false;

  return true;
}

bool Energesis_SHT2x::isHeaterOn(){
  uint8_t reg;

  if( !getUserRegister(&reg) ){
    return false;
  }

  return (bool) (reg & SHT2x_HEATER_BITMASK);
}

bool Energesis_SHT2x::setUserRegister( uint8_t reg ){
  m_tx_buff[0] = SHT2x_CMD_WRITE_USER_REGISTER;
  m_tx_buff[1] = reg;

  if( !m_device->write( m_tx_buff, 2 ) ){
    return false;
  }

  return true;
}

bool Energesis_SHT2x::getUserRegister( uint8_t *reg ){
  m_tx_buff[0] = SHT2x_CMD_READ_USER_REGISTER;
  if( !m_device->write( m_tx_buff, 1 )){
    return false;
  }

  if( !m_device->read( m_rx_buff, 1 ) ){
    return false;
  }

  *reg = m_rx_buff[0];
  return true;
}


Energesis_Sensor* Energesis_SHT2x::getTemperatureSensor(){
  if( !m_temperature_sensor ) 
    m_temperature_sensor = new Energesis_SHT2x_Temp( this );
  
  return (Energesis_Sensor*)m_temperature_sensor; 
}

Energesis_Sensor* Energesis_SHT2x::getRelativeHumiditySensor(){ 
  if( !m_humidity_sensor)
    m_humidity_sensor = new Energesis_SHT2x_Humidity(this);
  
  return (Energesis_Sensor*)m_humidity_sensor; 
}


Energesis_SHT2x::~Energesis_SHT2x(){
  
  if( m_device )
    delete m_device;
}



};

