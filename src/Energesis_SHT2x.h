/**
 * @file Energesis_SHT2x.h
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief Definición del controlador para la serie de sensores SHT2x de Sensirion 
 * @version 0.1
 * @date 2022-05-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _ENERGESIS_SHT2X_H_
#define _ENERGESIS_SHT2X_H_

#include "Energesis_Sensor.h"
#include "Energesis_I2CDevice.h"

#define ENERGESIS_SHT2X_VERSION_MAJOR 0
#define ENERGESIS_SHT2X_VERSION_MINOR 1


namespace Energesis{

/**
 * @brief Dirección del sensor en el bus I2C
 */
#define SHT2x_ADDRESS 0x40

/* Las macros SHT2x_CMD_xxxxxxxx representan los distintos comandos que
  se pueden enviar al sensor. */

#define SHT2x_CMD_TRIGGER_T_MEASUREMENT_HOLD    0b11100011  //!< Inicia la medición de temperatura (espera activada) 
#define SHT2x_CMD_TRIGGER_RH_MEASUREMENT_HOLD   0b11100001  //!< Inicia la medición de humedad relativa (espera activada)
#define SHT2x_CMD_TRIGGER_T_MEASUREMENT         0b11110011  //!< Inicia la medición de temperatura 
#define SHT2x_CMD_TRIGGER_RH_MEASUREMENT        0b11110101  //!< Inicia la medición de humedad relativa
#define SHT2x_CMD_WRITE_USER_REGISTER           0b11100110  //!< Escribe al registro de usuario
#define SHT2x_CMD_READ_USER_REGISTER            0b11100111  //!< Lee el registro de usuario
#define SHT2x_CMD_SOFT_RESET                    0b11111110  //!< Reinicia el sensor 

/**
 * @brief Opciones de configuración de resolución en
 * la serie de sensores SHT2x.
 * 
 * @note Utilizado con la función setResolution()
 */
enum SHT2x_RESOLUTION{
  RESOLUTION_RH_12BITS_T_14BITS = 0b00000000, //!< Humedad relativa: 12 bits & Temperatura: 14 bits
  RESOLUTION_RH_8BITS_T_12BITS  = 0b00000001, //!< Humedad relativa: 8 bits & Temperatura: 12 bits
  RESOLUTION_RH_10BITS_T_13BITS = 0b10000000, //!< Humedad relativa: 10 bits & Temperatura: 13 bits
  RESOLUTION_RH_11BITS_T_11BITS = 0b10000001, //!< Humedad relativa: 11 bits & Temperatura: 11 bits
};

#define SHT2x_RESOLUTION_BITMASK  0x81    //!< Mascara para los bits de resolución en el registro de usuario
#define SHT2x_STATUS_BITMASK      (1<<6)  //!< Mascara para el bit de estado en el registro de usuario
#define SHT2x_HEATER_BITMASK      (1<<2)  //!< Mascara para el bit de 'on-chip heater'
#define SHT2x_OTP_RELOAD_BITMASK  (1<<1)  //!< Mascara para el bit OPT reload

class Energesis_SHT2x;

class Energesis_SHT2x_Temp: public Energesis_Sensor{
public:
  Energesis_SHT2x_Temp( Energesis_SHT2x *sens ){ m_sht2x = sens; }

  bool getSample(sensor_sample_t *sample);

  void getSensorDetails( sensor_details_t* );

private:
  Energesis_SHT2x *m_sht2x = NULL;

};

class Energesis_SHT2x_Humidity: public Energesis_Sensor{
public:
  Energesis_SHT2x_Humidity( Energesis_SHT2x *sens ){ m_sht2x = sens; }

  bool getSample(sensor_sample_t *sample);

  void getSensorDetails( sensor_details_t* details );

private:
  Energesis_SHT2x *m_sht2x = NULL;

};

class Energesis_SHT2x: public Energesis_TemperatureSensor, public Energesis_RelativeHumiditySensor{

public:
  Energesis_SHT2x( );
  ~Energesis_SHT2x();

#if defined(ESP8266) || defined(ESP32)

  /**
   * @brief Inicializa el controlador y el bus I2C con los pines indicados.
   * 
   * @param sda Pin utilizado para la señal SDA del bus I2C
   * @param scl Pin utilizado para la señal SCL del bus I2C
   * @param i2c_bus Bus I2C (TwoWire) utilizado para la comunicación.
   * Si no se especifica utiliza Wire.
   * @return true Controlador inicializado correctamente
   * @return false Error al inicializar el controlador
   * 
   * @note Solo disponible para ESP32 y ESP8266
   * 
   * @code {.cpp}
   * Energesis_SHT21 sensor;
   * if( !sensor.begin(21,22,&Wire1) )
   *  Serial.println("Error");
   * @endcode
   * 
   */
  bool begin( int sda, int scl, TwoWire *i2c_bus = &Wire );
#endif

  /**
   * @brief Inicializa el controlador y el bus I2C con los pines indicados.
   * 
   * @param i2c_bus Bus I2C (TwoWire) utilizado para la comunicación.
   * Si no se especifica utiliza Wire.
   * @return true Controlador inicializado correctamente
   * @return false Error al inicializar el controlador
   * 
   * @code {.cpp}
   * Energesis_SHT21 sensor;
   * if( !sensor.begin(&Wire) )
   *  Serial.println("Error");
   * @endcode
   * 
   */
  bool begin( TwoWire *i2c_bus = &Wire );

  /**
   * @brief Retorna el valor de la temperatura en ºC.
   * 
   * @return float Temperatura en grados Celsius.
   * @return INFINITY Error al obtener el valor de humedad relativa
   */
  float getTemperature();

  /**
   * @brief Retorna la humedad relativa en %
   * 
   * @return float Humedad relativa
   * @return INFINITY Error al obtener el valor de humedad relativa
   */
  float getRelativeHumidity();

  Energesis_Sensor *getTemperatureSensor();

  Energesis_Sensor *getRelativeHumiditySensor();

  /**
   * @brief Envía al sensor la orden de iniciar la lectura de temperatura
   * 
   * @return true Orden enviada de forma satisfactoria 
   * @return false Error al enviar la orden
   */
  bool measureTemperature();

  /**
   * @brief Retorna el valor de temperatura despues de haber ejecutado measureTemperature().
   * 
   * 
   * @code {.cpp}
   * Energesis_SHT2x sensor;
   * //...
   * //...
   * if( sensor.measureTemperature() )
   *  temp = sensor.readTemperature();
   * @endcode
   * 
   * 
   * @return float Temperatura en grados Celsius.
   * @return INFINITY Error al obtener el valor de la temperatura
   * 
   */
  float readTemperature();

  /**
   * @brief Envía al sensor la orden de iniciar la lectura de humedad
   * relativa.
   * 
   * @return true Orden enviada de forma satisfactoria 
   * @return false Error al enviar la orden
   */
  bool measureRelativeHumidity();

  /**
   * @brief Retorna el valor de humedad relativa despues de haber ejecutado 
   * measureRelativeHumidity().
   * 
   * @code {.cpp}
   * Energesis_SHT2x sensor;
   * //...
   * //...
   * if( sensor.measureRelativeHumidity() )
   *  humidity = sensor.readRelativeHumidity();
   * @endcode
   * 
   * 
   * @return float Humedad relativa.
   * @return INFINITY Error al obtener el valor de la temperatura
   * 
   */
  float readRelativeHumidity();

  /**
   * @brief Configura la resolución de lectura de la temperatura y la humedad.
   * 
   * @param resolution nueva configuración de resolución. \see SHT2x_RESOLUTION.  
   * @return true Nueva configuración establecida correctamente.
   * @return false Error al establecer la configuracion de resolución.
   */
  bool setResolution( SHT2x_RESOLUTION resolution );

  /**
   * @brief Retorna la resolución de la temperatura en bits.
   * 
   * @return uint8_t Resolución de la temperatura en bits.
   * @return 0 Error al obtener la resolución.
   */
  uint8_t getTemperatureResolution();

  /**
   * @brief Retorna la resolución de la humedad relativa en bits.
   * 
   * @return uint8_t Resolución de la humedad relativa en bits.
   * @return 0 Error al obtener la resolución.
   */
  uint8_t getHumidityResolution();

  /**
   * @brief Establece el estado del calentador (Heater) interno del chip. 
   * 
   * @param on Si es true enciende el calentador. Caso contrario lo apaga.
   * @return true Operación realizada correctamente.
   * @return false Error al realizar la operación.
   */
  bool heater(bool on);

  /**
   * @brief Retorna el estado del calentador (Heater) interno del chip
   * 
   * @return true Heater encendido
   * @return false Heater apagado
   */
  bool isHeaterOn();

  /**
   * @brief Reinicia el sensor
   * 
   * @return true Orden de reinicio enviada correctamente
   * @return false Error al enviar la orden
   */
  bool reset();

private:
  bool checkCRC( uint16_t data, uint8_t crc );

  /**
   * @brief Lee el valor del registro de usuario del sensor
   * 
   * @param reg puntero a la variable uint8_t donde se alamacenará
   * el valor del registro de usuario.
   * @return true registro obtenido satisfactoriamente
   * @return false error al leer el registro.
   */
  bool getUserRegister( uint8_t *reg );

  /**
   * @brief Asignar nuevo valor al registro de usuario
   * 
   * @param reg nuevo valor a asignar al registro
   * @return true valor asignado satisfactoriamente
   * @return false error al asignar el valor
   */
  bool setUserRegister( uint8_t reg );
  
  bool m_begun;                       //!< Indica si el controlador se ha inicializado
  ulong m_last_read_temperature = 0;  //!< Timestamp del último valor de temperatura 
  ulong m_last_read_humidity = 0;     //!< Timestamp del último valor de humedad
  float m_last_temperature;           //!< Último valor de temperatura leído
  float m_last_humidity;              //!< Último valor de humedad leído

protected:

  Energesis_I2CDevice *m_device = NULL; //!< Bus de comunicación
  uint8_t m_tx_buff[5];                 //!< Buffer de transmisión
  uint8_t m_rx_buff[10];                //!< Buffer de recepción


  friend class Energesis_SHT2x_Temp;
  friend class Energesis_SHT2x_Humidity;
};


};

#endif