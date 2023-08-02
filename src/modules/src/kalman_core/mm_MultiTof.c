
#include "mm_MultiTof.h"
#include "log.h"
#define DEBUG_MODULE ""
#include "debug.h"
#include "cfassert.h"
#




void kalmanCoreUpdateWithMultiTof(kalmanCoreData_t* this, const MultitofMeasurement_t *multitof,  const Axis3f *gyro) {

float omegax_b = gyro->x * DEG_TO_RAD;
float omegay_b = gyro->y * DEG_TO_RAD;

//pointing at wall head on, accounting for pitch
uint64_t dt = multitof->timestampprev - multitof->timestamp;

float h[KC_STATE_DIM] = {0};
arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};  
  



























// DEBUG_PRINT("Prev: ");
 
// DEBUG_PRINT("%d ", multitof->timestampprev);
// DEBUG_PRINT("\n");
// DEBUG_PRINT("Curr: ");
 
// DEBUG_PRINT("%d ", multitof->timestamp);
// DEBUG_PRINT("\n");
   
//  uint32_t timedebug = multitof->timestamp;
//  DEBUG_PRINT("Multitof Timestamp: %d\n", timedebug);
//  uint16_t stddev = multitof->stdDev;
//  DEBUG_PRINT("Avg Delta: %d\n", stddev); 
  
// DEBUG_PRINT("Prev: ");
//  for(uint8_t j = 0; j<NR_OF_PIXELS; j++){
//       DEBUG_PRINT("%d ", multitof->distancesprev[j]);
//    }
// DEBUG_PRINT("\n");
// DEBUG_PRINT("Curr: ");
//  for(uint8_t j = 0; j<NR_OF_PIXELS; j++){
//       DEBUG_PRINT("%d ", multitof->distances[j]);
//    }
// DEBUG_PRINT("\n");
  
  
  
  
  
  
  //Debug with averages in Each Direction
  // uint16_t averages[NR_OF_SENSORS];
  
  // for(uint8_t i=0; i<NR_OF_SENSORS; i++){
  //   uint16_t temp = 0;
   
  // for(uint8_t j = 0; j<NR_OF_PIXELS; j++){
   
  //   temp += multitof->distances[j + i * NR_OF_PIXELS];
  //   averages[i] = temp / NR_OF_PIXELS;

  // }
  // }
  
  // for(uint8_t i=0; i<NR_OF_SENSORS; i++){
  //   DEBUG_PRINT("%d: %d", i, averages[i]);
  // }
}