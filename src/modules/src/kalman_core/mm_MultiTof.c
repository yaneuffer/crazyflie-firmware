
#include "mm_MultiTof.h"
#include "log.h"
#define DEBUG_MODULE ""
#include "debug.h"
#include "cfassert.h"
#include "mm_pose.h"
#include "math3d.h"


static float alphas[8] = {0.39269, 0.2805, 0.1683, 0.0561, -0.0561, -0.1683, -0.2805, -0.39296};
static float betaprevF;
static float betaprevR;
static uint16_t colAveragesPrevF[8] = {0}; 
static uint16_t colAveragesPrevR[8] = {0}; 
static uint64_t timestampprev = 0;
float initialbetas[4] = {0};
float referenceDistances[4] = {0};
bool initial = 1;
static uint32_t iterations = 0;
//valid yaw update for current measurement on sensor 1/2/3/4
static bool active[4] = {true,true,true,true};
//Sensor currently in use for yaw update (5 for all inactive)
static uint8_t inUse = 0;
static float referenceYawKalmanF = 0;
static float referenceYawKalmanR = 0;


void kalmanCoreUpdateWithMultiTofY(kalmanCoreData_t* this, const MultitofMeasurement_t *multitof,  const Axis3f *gyro) {
 
uint64_t dtx = (multitof->timestamp - timestampprev) / 1000;
// DEBUG_PRINT("%d  ", timestampprev);
 DEBUG_PRINT("%d\n", timestampprev);
// DEBUG_PRINT("*%f  ", multitof->stdDevF);
// DEBUG_PRINT("*%f  ", multitof->stdDevR);

double dt = dtx / 1000.0f;
// DEBUG_PRINT("%f\n", dt);
 // 1000000.0f
////////////////////////////////////////////////Front sensor//////////////////////////////////////////////////////////
uint16_t colAveragesF[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t colAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    colAv += multitof->distancesF[i + j * 8];
  }
  colAveragesF[i] = colAv / 8;
}

float rowAveragesF[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t rowAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    rowAv += multitof->distancesF[j + i * 8];
  }
  rowAveragesF[i] = rowAv / 8;
}

float betaF = atanf((tanf(alphas[0]) * colAveragesF[0] - tanf(alphas[7]) * colAveragesF[7]) / (colAveragesF[7] - colAveragesF[0]));
//same as beta but y axis
float gammaF = -atanf((tanf(alphas[0]) * rowAveragesF[0] - tanf(alphas[7]) * rowAveragesF[7]) / (rowAveragesF[7] - rowAveragesF[0]));

DEBUG_PRINT("%f\n", this->S[KC_STATE_PX]);
float predictedmmF = this->S[KC_STATE_PX] * dt * 1000;

float  currentdistF = ((colAveragesF[3] + colAveragesF[4]) / 2 );
float prevdistF = ((colAveragesPrevF[3] + colAveragesPrevF[4]) / 2 );

float measuredmmF = prevdistF - currentdistF;
    DEBUG_PRINT("%f   %f\n", measuredmmF, predictedmmF);
float hF[KC_STATE_DIM] = {0};
arm_matrix_instance_f32 HF = {1, KC_STATE_DIM, hF}; 
hF[KC_STATE_PX] = 1;


if(colAveragesPrevF[0] != 0) {
kalmanCoreScalarUpdate(this, &HF, (measuredmmF-predictedmmF), multitof->stdDevF);
}

memcpy(&colAveragesPrevF, colAveragesF, sizeof(uint16_t) * 8);
betaprevF = betaF;

//////////////////////////////////////////Right sensor//////////////////////////////////////////////////////////
uint16_t colAveragesR[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t colAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    colAv += multitof->distancesR[i + j * 8];
  }
  colAveragesR[i] = colAv / 8;
}

float rowAveragesR[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t rowAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    rowAv += multitof->distancesR[j + i * 8];
  }
  rowAveragesR[i] = rowAv / 8;
}


float betaR = atanf((tanf(alphas[0]) * colAveragesR[0] - tanf(alphas[7]) * colAveragesR[7]) / (colAveragesR[7] - colAveragesR[0]));
//same as beta but y axis
float gammaR = -atanf((tanf(alphas[0]) * rowAveragesR[0] - tanf(alphas[7]) * rowAveragesR[7]) / (rowAveragesR[7] - rowAveragesR[0]));


float predictedmmR = (this->S[KC_STATE_PY]) * dt * 1000;
float currentdistR = ((colAveragesR[3] + colAveragesR[4]) / 2 );
float prevdistR = ((colAveragesPrevR[3] + colAveragesPrevR[4]) / 2 );

float measuredmmR = -(prevdistR - currentdistR);
//  DEBUG_PRINT("%f\n", measuredmmR);
float hR[KC_STATE_DIM] = {0};
arm_matrix_instance_f32 HR = {1, KC_STATE_DIM, hR}; 
hR[KC_STATE_PY] = 1;


if(colAveragesPrevF[0] != 0) {
kalmanCoreScalarUpdate(this, &HR, (measuredmmR-predictedmmR), multitof->stdDevR);
}

memcpy(&colAveragesPrevR, colAveragesR, sizeof(uint16_t) * 8);
betaprevR = betaF;
//Final changes
 timestampprev = multitof->timestamp;

}



































void kalmanCoreUpdateWithMultiTofC(kalmanCoreData_t* this, const MultitofMeasurement_t *multitof,  const Axis3f *gyro) {
}

















//attitude updates

void kalmanCoreUpdateWithMultiTof(kalmanCoreData_t* this, const MultitofMeasurement_t *multitof,  const Axis3f *gyro) {


// ////////////////////////////////////////////////Front sensor//////////////////////////////////////////////////////////
uint16_t colAveragesF[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t colAv = 0;
  uint8_t numInvalid = 0;
  for(uint8_t j = 0; j < 8; j++) {
    colAv += multitof->distancesF[i + j * 8];
    if(multitof->distancesF[i + j * 8] == 0) {numInvalid++;}
  }
  if(numInvalid >= 6) {
    colAveragesF[i] = 0;
  }
  else {
  colAveragesF[i] = colAv / (8 - numInvalid);
    }
}


// find the leftmost and rightmost valid column index, store left in v[0], right in v[1]
uint8_t v[2] = {10, 10};
for(uint8_t i = 0; i < 8; i++) {
if(colAveragesF[i] != 0) {
  v[0] = i;
  break;
}
}
for(int8_t i = 7; i >= 0; i--) {
  if(colAveragesF[i] != 0) {
    v[1] = i;
    break;
  }
}


//no gaps in between relevant outside columns, else invalid
if(v[0] != 10) {
  for(uint8_t i = v[0] + 1; i < v[1]; i++){
    if (colAveragesF[i] == 0) {
      v[0] = 10;
      v[1] = 10;
    }
  }
}


//want at least 4 valid columns for angle, otherwise set 10,10 for invalid
if(v[1] - v[0] < 3) {
  v[0] = 10;
  v[1] = 10;
  }
 DEBUG_PRINT("%d   %d\n", v[0], v[1]);

//set sensor inactive if measurement is invalid
if(v[0] == 10) {
  active[0] = false;
  }

// float rowAveragesF[8]; 
// for(uint8_t i = 0; i < 8; i++) {
//   uint16_t rowAv = 0;
//   for(uint8_t j = 0; j < 8; j++) {
//     rowAv += multitof->distancesF[j + i * 8];
//   }
//   rowAveragesF[i] = rowAv / 8;
// }

//angle computation, if measurement valid
float betaF;
if(v[0] != 10) {
betaF = RAD_TO_DEG * -atanf((tanf(alphas[v[0]]) * colAveragesF[v[0]] - tanf(alphas[v[1]]) * colAveragesF[v[1]]) / (colAveragesF[v[1]] - colAveragesF[v[0]]));
  

if(betaF > 0) {betaF = fabs(betaF - 90);}
else {betaF = -(90 + betaF);}
}
else{
betaF = betaprevF;
}

float currentyaw = RAD_TO_DEG*(quat2rpy(mkquat(this->q[1], this->q[2], this->q[3], this->q[0])).z);

//set new reference at first acceptable measurement after period of invalid ones, set sensor active again
if(active[0] == false && v[0] != 10) {
initialbetas[0] = betaF;
active[0] = true;
//extract current yaw from EKF
referenceYawKalmanF = RAD_TO_DEG*(quat2rpy(mkquat(this->q[1], this->q[2], this->q[3], this->q[0])).z);
}

DEBUG_PRINT("%f    %f   %f  %f\n", betaF, initialbetas[0], referenceYawKalmanF, currentyaw);

//ignore outliers
if(fabs((betaF - betaprevF)) > 20){
  betaF = betaprevF;
}

//same as beta but y axis

// float gammaF = -atanf((tanf(alphas[0]) * rowAveragesF[0] - tanf(alphas[7]) * rowAveragesF[7]) / (rowAveragesF[7] - rowAveragesF[0]));

memcpy(&colAveragesPrevF, colAveragesF, sizeof(uint16_t) * 8);
betaprevF = betaF;
//startup, set reference after 10th measurement as first couple are often useless
if(iterations == 10){
  initialbetas[0] = betaF;
}


//////////////////////////////////////////Right sensor//////////////////////////////////////////////////////////
uint16_t colAveragesR[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t colAv = 0;
  uint8_t numInvalid = 0;
  for(uint8_t j = 0; j < 8; j++) {
    colAv += multitof->distancesR[i + j * 8];
    if(multitof->distancesR[i + j * 8] == 0) {numInvalid++;}
  }
  if(numInvalid >= 6) {
    colAveragesR[i] = 0;
  }
  else {
  colAveragesR[i] = colAv / (8 - numInvalid);
    }
}



// find the leftmost and rightmost valid column index, store left in b[0], right in b[1]
uint8_t b[2] = {10, 10};
for(uint8_t i = 0; i < 8; i++) {
if(colAveragesR[i] != 0) {
  b[0] = i;
  break;
}
}
for(int8_t i = 7; i >= 0; i--) {
if(colAveragesR[i] != 0) {
  b[1] = i;
  break;
}
}
//need at least two valid columns for angle, otherwise set 10,10 for invalid
if(b[0] == b[1]) {
  b[0] = 10;
  b[1] = 10;
  }
// DEBUG_PRINT("%d   %d\n", b[0], b[1]);

// float rowAveragesR[8]; 
// for(uint8_t i = 0; i < 8; i++) {
//   uint16_t rowAv = 0;
//   for(uint8_t j = 0; j < 8; j++) {
//     rowAv += multitof->distancesR[j + i * 8];
//   }
//   rowAveragesR[i] = rowAv / 8;
// }
float betaR;
if(b[0] != 10) {
betaR = atanf((tanf(alphas[b[0]]) * colAveragesR[b[0]] - tanf(alphas[b[1]]) * colAveragesR[b[1]]) / (colAveragesR[b[1]] - colAveragesR[b[0]]));
if(betaR > 0) {betaR = fabs(betaR - 90);}
else {betaR = -(90 + betaR);}
}
else{
  betaR = betaprevR;
}
//same as beta but y axis
// float gammaR = -atanf((tanf(alphas[0]) * rowAveragesR[0] - tanf(alphas[7]) * rowAveragesR[7]) / (rowAveragesR[7] - rowAveragesR[0]));

memcpy(&colAveragesPrevR, colAveragesR, sizeof(uint16_t) * 8);
betaprevR = betaR;
  
////////////////////////////////////////Attitude Computation//////////////////////////////////////////////////////

float currentRollKalman = (quat2rpy(mkquat(this->q[1], this->q[2], this->q[3], this->q[0])).x);
float currentPitchKalman = (quat2rpy(mkquat(this->q[1], this->q[2], this->q[3], this->q[0])).y);
float currentYawKalman = (quat2rpy(mkquat(this->q[1], this->q[2], this->q[3], this->q[0])).z);

struct vec measurement;
if (active[0] == true) {
  measurement = mkvec(currentRollKalman, currentPitchKalman, DEG_TO_RAD*(betaF - initialbetas[0] + referenceYawKalmanF));
}
else {
  measurement = mkvec(currentRollKalman, currentPitchKalman, currentYawKalman);
}
// DEBUG_PRINT("%f   %f   %d\n", betaF, initialbetas[0], iterations);
struct quat measurementq = rpy2quat(measurement);
// compute orientation error
struct quat const q_ekf = mkquat(this->q[1], this->q[2], this->q[3], this->q[0]);
struct quat const q_measured = mkquat(measurementq.x, measurementq.y, measurementq.z, measurementq.w);
struct quat const q_residual = qqmul(qinv(q_ekf), q_measured);

struct vec const err_quat = vscl(2.0f / q_residual.w, quatimagpart(q_residual));

// do a scalar update for each state
  
float h[KC_STATE_DIM] = {0};
arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};
// h[KC_STATE_D0] = 1;
// kalmanCoreScalarUpdate(this, &H, err_quat.x, 0);
// h[KC_STATE_D0] = 0;

// h[KC_STATE_D1] = 1;
// kalmanCoreScalarUpdate(this, &H, err_quat.y, 0);
// h[KC_STATE_D1] = 0;

h[KC_STATE_D2] = 1;
kalmanCoreScalarUpdate(this, &H, err_quat.z, 0);

//Final changes
 timestampprev = multitof->timestamp;
 iterations++;
}
























































//absolute distances
void kalmanCoreUpdateWithMultiTofX(kalmanCoreData_t* this, const MultitofMeasurement_t *multitof,  const Axis3f *gyro) {
 
uint64_t dtx = (multitof->timestamp - timestampprev) / 1000;
// DEBUG_PRINT("%d  ", timestampprev);
//  DEBUG_PRINT("%d\n", timestampprev);
// DEBUG_PRINT("*%f  ", multitof->stdDevF);
// DEBUG_PRINT("*%f  ", multitof->stdDevR);

double dt = dtx / 1000.0f;
// DEBUG_PRINT("%f\n", dt);
 // 1000000.0f
////////////////////////////////////////////////Front sensor//////////////////////////////////////////////////////////
uint16_t colAveragesF[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t colAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    colAv += multitof->distancesF[i + j * 8];
  }
  colAveragesF[i] = colAv / 8;
}

float rowAveragesF[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t rowAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    rowAv += multitof->distancesF[j + i * 8];
  }
  rowAveragesF[i] = rowAv / 8;
}


float currentdistF = ((colAveragesF[3] + colAveragesF[4]) / 2 );
float prevdistF = ((colAveragesPrevF[3] + colAveragesPrevF[4]) / 2 );
//only on first stable measurement, check whether consecutive measurements are close enough to be stable
if(initial == 1 && fabs((currentdistF/prevdistF) - 1) < 0.15){
  referenceDistances[0] = currentdistF;
  initial = 0;
}

// DEBUG_PRINT("%f   %f\n", currentdistF, referenceDistances[0]);

float hF[KC_STATE_DIM] = {0};
arm_matrix_instance_f32 HF = {1, KC_STATE_DIM, hF}; 
hF[KC_STATE_X] = 1;


kalmanCoreScalarUpdate(this, &HF, (referenceDistances[0] - currentdistF)/1000 - this->S[KC_STATE_X], 0.0002);

// kalmanCoreScalarUpdate(this, &HF, 0.08 - this->S[KC_STATE_X], 0);

memcpy(&colAveragesPrevF, colAveragesF, sizeof(uint16_t) * 8);

//////////////////////////////////////////Right sensor//////////////////////////////////////////////////////////
uint16_t colAveragesR[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t colAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    colAv += multitof->distancesR[i + j * 8];
  }
  colAveragesR[i] = colAv / 8;
}

float rowAveragesR[8]; 
for(uint8_t i = 0; i < 8; i++) {
  uint16_t rowAv = 0;
  for(uint8_t j = 0; j < 8; j++) {
    rowAv += multitof->distancesR[j + i * 8];
  }
  rowAveragesR[i] = rowAv / 8;
}




float currentdistR = ((colAveragesR[3] + colAveragesR[4]) / 2 );
float prevdistR = ((colAveragesPrevF[3] + colAveragesPrevF[4]) / 2 );
//only on first stable measurement, check whether consecutive measurements are close enough to be stable
if(initial == 1 && fabs((currentdistR/prevdistR) - 1) < 0.15){
  referenceDistances[3] = currentdistR;
  initial = 0;
}


float hR[KC_STATE_DIM] = {0};
arm_matrix_instance_f32 HR = {1, KC_STATE_DIM, hR}; 
hR[KC_STATE_Y] = 1;



kalmanCoreScalarUpdate(this, &HR, (referenceDistances[3] - currentdistR)/1000 - this->S[KC_STATE_Y], 0.0002);

memcpy(&colAveragesPrevR, colAveragesR, sizeof(uint16_t) * 8);


//Final changes
 timestampprev = multitof->timestamp;

}
