/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_MultiTof.h"
#include "log.h"
#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"
#include "cfassert.h"
#




void kalmanCoreUpdateWithMultiTof(kalmanCoreData_t* this, const MultitofMeasurement_t *flow) {
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  
  //Debug with averages in Each Direction
  uint16_t averages[NR_OF_SENSORS];
  
  for(uint8_t i=0; i<NR_OF_SENSORS; i++){
    uint16_t temp = 0;
   
  for(uint8_t j = 0; j<NR_OF_PIXELS; j++){
   
    temp += flow->distances[j + i * NR_OF_PIXELS];
    averages[i] = temp / NR_OF_PIXELS;

  }
  }
  
  for(uint8_t i=0; i<NR_OF_SENSORS; i++){
    DEBUG_PRINT("%d: %d", i, averages[i]);
  }
}