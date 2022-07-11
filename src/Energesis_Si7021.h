/**
 * @file Energesis_Si7021.h
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief Controlador para el sensor Si7021
 * @version 0.1
 * @date 2022-05-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _ENERGESIS_SI7021_H_
#define _ENERGESIS_SI7021_H_

#include "Energesis_HTU2xD.h"

#define ENERGESIS_SI702X_VERSION_MAJOR 0
#define ENERGESIS_SI702X_VERSION_MINOR 1


namespace Energesis{

class Energesis_Si7021: public Energesis_HTU2x{

public:

  uint64_t getSerialNumber();

  /**
   * @brief Retorna la version del firmware del sensor.
   * 
   * @return uint8_t version del firmware del sensor
   */
  uint8_t getFirmwareRevision();
  
  size_t getName( char *name, size_t capacity = 16 );

  //!< @todo pendiente de implementación 
  Energesis_Sensor* getTemperatureSensor();

  //!< @todo pendiente de implementación 
  Energesis_Sensor* getRelativeHumiditySensor();



};

};



#endif
