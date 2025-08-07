#include <string.h>
#include "math.h"
#include "Wire.h"
#include <ADXL345.h>
#include <HMC5883L.h>
#include <L3G4200D.h>
#include "Matrix.h"

#define degToRad(angleInDegrees) ((angleInDegrees) * M_PI / 180.0)
#define radToDeg(angleInRadians) ((angleInRadians) * 180.0 / M_PI)

ADXL345 accelerometer;
HMC5883L compass;
L3G4200D gyroscope;

float accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ;
Vector norm;

float phiXa=0.0, phiYa=0.0, phiZm=0.0, phiX=0.0, phiY=0.0, phiZ=0.0, bx = 0.0, by = 0.0, bz = 0.0, V1 = 0.0, V2 = 0.0, V3 = 0.0, xHead, yHead;
uint16_t t0 = 0, t1 = 0;
float dt = 0.0;

// kalman filter params
float Qa = 0.0003;
float Qb = 0.0003;
float Racc = 0.003;
float Rmag = 0.001;

float P[36] = {1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
float K[18];
float S[9];

void readAngles();
void kalmanFilter();

void setup() {
  Wire.begin();
  Serial.begin(115200);
  if(!accelerometer.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }
  
  while(!gyroscope.begin(L3G4200D_SCALE_2000DPS, L3G4200D_DATARATE_400HZ_50))
  {
    Serial.println("Could not find a valid L3G4200D sensor, check wiring!");
    delay(500);
  }

  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  delay(20);
  accelerometer.setRange(ADXL345_RANGE_2G);
  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

}

void loop() {
  readAngles(); // We get the measure
    
  kalmanFilter(); // We filter it and update our angles

  Serial.print("Roll: "); Serial.print(radToDeg(phiX)); Serial.print(", ");
  Serial.print("Pitch: "); Serial.print(radToDeg(phiY)); Serial.print(", ");
  Serial.print("Yaw: "); Serial.print(radToDeg(phiZ)); Serial.print(", ");
  Serial.print("Loop time: "); Serial.print(dt*1000.0); Serial.println("ms."); 
//  Serial.print("X-Bias: "); Serial.print(radToDeg(bx)); Serial.print(", ");
//  Serial.print("Y-Bias: "); Serial.print(radToDeg(by)); Serial.print(", ");
//  Serial.print("Z-Bias: "); Serial.print(radToDeg(bz)); Serial.println(", ");
 // delay(200);

}

void readAngles()
{

  t1 = millis();
  dt = (t1 - t0)/1000.0;
  t0 = t1;

  norm = accelerometer.readNormalize();
  accX = norm.XAxis;
  accY = norm.YAxis;
  accZ = norm.ZAxis;

  phiXa = atan2(accY,accZ);
  phiYa = atan2(-accX,sqrt(accY*accY+accZ*accZ));
  //Serial.print(accX); Serial.print(", "); Serial.print(accY); Serial.print(", "); Serial.print(accZ); Serial.print(", "); Serial.print(radToDeg(phiXa)); Serial.print(", "); Serial.println(radToDeg(phiYa));

  norm = gyroscope.readNormalize();
  gyroX = degToRad(norm.XAxis);
  gyroY = degToRad(norm.YAxis);
  gyroZ = degToRad(norm.ZAxis);

  //Serial.print(gyroX); Serial.print(", "); Serial.print(gyroY); Serial.print(", "); Serial.println(gyroZ);

  norm = compass.readNormalize();
  magX = norm.XAxis;
  magY = norm.YAxis;
  magZ = norm.ZAxis;

  xHead = magX*cos(phiXa) + magY*sin(phiYa)*sin(phiXa) + magZ*sin(phiYa)*cos(phiXa);
  yHead = magZ*sin(phiXa) - magY*cos(phiXa);

  phiZm = atan2(yHead,xHead);
  if(phiZm < 0)
    phiZm += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(phiZm > 2*PI)
    phiZm -= 2*PI;

//  Serial.print(magX); Serial.print(", "); Serial.print(magY); Serial.print(", "); Serial.print(magZ); Serial.print(", "); Serial.println(radToDeg(phiZm));

}

void kalmanFilter()
{
  // Measurement update
  float H[18] = {1,0,0,0,0,0, 0,0,1,0,0,0, 0,0,0,0,1,0};
  float Ht[18];
  matrix_transpose(H, Ht, 3, 6);

  float R[9] = {Racc, 0, 0, 0, Racc, 0, 0, 0, Rmag};

  float P_Ht[18];
  matrix_multiply(P, Ht, P_Ht, 6, 6, 3);

  float H_P_Ht[9];
  matrix_multiply(H, P_Ht, H_P_Ht, 3, 6, 3);

  matrix_add(H_P_Ht, R, S, 3, 3);

  float S_inv[9];
  if (matrix_invert(S, S_inv) == 0) {
    // S is singular
    return;
  }

  matrix_multiply(P_Ht, S_inv, K, 6, 3, 3);

  // State estimation
  float err[3] = {phiXa - phiX, phiYa - phiY, phiZm - phiZ};
  
  float K_err[6];
  matrix_multiply(K, err, K_err, 6, 3, 1);

  phiX += K_err[0];
  bx += K_err[1];
  phiY += K_err[2];
  by += K_err[3];
  phiZ += K_err[4];
  bz += K_err[5];

  // Covariance update
  float K_H[36];
  matrix_multiply(K, H, K_H, 6, 3, 6);

  float I[36] = {1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1};
  float I_KH[36];
  matrix_subtract(I, K_H, I_KH, 6, 6);

  float P_new[36];
  matrix_multiply(I_KH, P, P_new, 6, 6, 6);
  memcpy(P, P_new, sizeof(P_new));

  // Model Prediction
  // State prediction
  float sinPhiX = sin(phiX);
  float cosPhiX = cos(phiX);
  float tanPhiY = tan(phiY);
  float cosPhiY = cos(phiY);

  phiX = phiX - bx*dt + (gyroX + sinPhiX*tanPhiY*gyroY + cosPhiX*tanPhiY*gyroZ)*dt;
  phiY = phiY - by*dt + (cosPhiX*gyroY - sinPhiX*gyroZ)*dt;
  if (cosPhiY != 0) {
    phiZ = phiZ - bz*dt + ((sinPhiX/cosPhiY)*gyroY + (cosPhiX/cosPhiY)*gyroZ)*dt;
  } else {
    // Avoid division by zero
    phiZ = phiZ - bz*dt;
  }

  // Covariance prediction
  float F[36] = {
    1, -dt, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, -dt, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, -dt,
    0, 0, 0, 0, 0, 1
  };
  float Ft[36];
  matrix_transpose(F, Ft, 6, 6);
  
  float P_F_t[36];
  matrix_multiply(P, Ft, P_F_t, 6, 6, 6);
  
  float F_P_Ft[36];
  matrix_multiply(F, P_F_t, F_P_Ft, 6, 6, 6);

  float Q[36] = {
    Qa, 0, 0, 0, 0, 0,
    0, Qb, 0, 0, 0, 0,
    0, 0, Qa, 0, 0, 0,
    0, 0, 0, Qb, 0, 0,
    0, 0, 0, 0, Qa, 0,
    0, 0, 0, 0, 0, Qb
  };

  matrix_add(F_P_Ft, Q, P, 6, 6);
}
