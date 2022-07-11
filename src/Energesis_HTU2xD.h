/**
 * @file Energesis_HTU2xD.h
 * @author José Guerra Carmenate <joseguerracarmenate@gmail.com>
 * @brief 
 * @version 0.1
 * @date 2022-05-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _ENERGESIS_HTU2x_H_
#define _ENERGESIS_HTU2x_H_

#include "Energesis_SHT2x.h"

#define ENERGESIS_HTU2X_VERSION_MAJOR 0
#define ENERGESIS_HTU2X_VERSION_MINOR 1

namespace Energesis{

class Energesis_HTU2x: public Energesis_SHT2x{
public:
  Energesis_HTU2x() :Energesis_SHT2x(){}
  
  /**
   * @brief Retorna el número de serie del sensor. Cada sensor tiene un número único.
   * 
   * @return uint64_t número de serie del sensor.
   */
  uint64_t getSerialNumber();

  Energesis_Sensor* getTemperatureSensor();

  Energesis_Sensor* getRelativeHumiditySensor();


};


};
#endif
