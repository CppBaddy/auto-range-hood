/*
 * SensorModel.h
 *
 *  Created on: 2020-09-01
 *      Author: yulay
 */

#ifndef SENSORMODEL_H_
#define SENSORMODEL_H_



typedef struct SensorData
{
    uint8_t sensorId;
    uint8_t temperature;
    uint8_t current;
    uint8_t voltage;
} SensorModel;




#endif /* SENSORMODEL_H_ */
