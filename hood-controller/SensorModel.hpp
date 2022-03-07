/*
 * SensorData.hpp
 *
 *  Created on: 2020-09-01
 *      Author: yulay
 */

#ifndef SENSORMODEL_HPP_
#define SENSORMODEL_HPP_



typedef struct SensorData
{
    uint8_t sensorId;
    uint8_t temperature;
    uint8_t current;
    uint8_t voltage;
} SensorData;




#endif /* SENSORMODEL_HPP_ */
