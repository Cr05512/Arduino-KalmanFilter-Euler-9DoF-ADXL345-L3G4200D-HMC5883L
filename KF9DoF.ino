#include "math.h"
#include "Wire.h"
#include <ADXL345.h>
#include <HMC5883L.h>
#include <L3G4200D.h>

 

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

float P00 = 1.0, P01 = 0.0, P02 = 0.0, P03 = 0.0, P04 = 0.0, P05 = 0.0;
float P10 = 0.0, P11 = 1.0, P12 = 0.0, P13 = 0.0, P14 = 0.0, P15 = 0.0;
float P20 = 0.0, P21 = 0.0, P22 = 1.0, P23 = 0.0, P24 = 0.0, P25 = 0.0;
float P30 = 0.0, P31 = 0.0, P32 = 0.0, P33 = 1.0, P34 = 0.0, P35 = 0.0;
float P40 = 0.0, P41 = 0.0, P42 = 0.0, P43 = 0.0, P44 = 1.0, P45 = 0.0;
float P50 = 0.0, P51 = 0.0, P52 = 0.0, P53 = 0.0, P54 = 0.0, P55 = 1.0;

float S00 = 1.0, S01 = 0.0, S02 = 0.0;
float S10 = 0.0, S11 = 1.0, S12 = 0.0;
float S20 = 0.0, S21 = 0.0, S22 = 1.0;
float denS = 1.0;

float S00inv = 1.0, S01inv = 0.0, S02inv = 0.0;
float S10inv = 0.0, S11inv = 1.0, S12inv = 0.0;
float S20inv = 0.0, S21inv = 0.0, S22inv = 1.0;

float K00 = 0.0, K01 = 0.0, K02 = 0.0;
float K10 = 0.0, K11 = 0.0, K12 = 0.0;
float K20 = 0.0, K21 = 0.0, K22 = 0.0;
float K30 = 0.0, K31 = 0.0, K32 = 0.0;
float K40 = 0.0, K41 = 0.0, K42 = 0.0;
float K50 = 0.0, K51 = 0.0, K52 = 0.0;

float errX = 0.0, errY = 0.0, errZ = 0.0;


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
  //Measurement update

  //Innovavtion covariance S
  S00 = Racc + P00;
  S01 = P02;
  S02 = P04;
  S10 = P20;
  S11 = Racc + P22;
  S12 = P24;
  S20 = P40;
  S21 = P42;
  S22 = Rmag + P44;

  denS = S00*S11*S22 - S00*S12*S21 - S01*S10*S22 + S01*S12*S20 + S02*S10*S21 - S02*S11*S20;
  //Inverse Innovation covariance Sinv
  S00inv = (S11*S22 - S12*S21)/denS;
  S01inv = -(S01*S22 - S02*S21)/denS;
  S02inv = (S01*S12 - S02*S11)/denS;
  S10inv = -(S10*S22 - S12*S20)/denS;
  S11inv = (S00*S22 - S02*S20)/denS;
  S12inv = -(S00*S12 - S02*S10)/denS;
  S20inv = (S10*S21 - S11*S20)/denS;
  S21inv = -(S00*S21 - S01*S20)/denS;
  S22inv = (S00*S11 - S01*S10)/denS;

  S00 = S00inv;
  S01 = S01inv;
  S02 = S02inv;
  S10 = S10inv;
  S11 = S11inv;
  S12 = S12inv;
  S20 = S20inv;
  S21 = S21inv;
  S22 = S22inv;



  //Kalman Gain
  K00 = P00*S00 + P02*S10 + P04*S20;
  K01 = P00*S01 + P02*S11 + P04*S21;
  K02 = P00*S02 + P02*S12 + P04*S22;
  K10 = P10*S00 + P12*S10 + P14*S20;
  K11 = P10*S01 + P12*S11 + P14*S21;
  K12 = P10*S02 + P12*S12 + P14*S22;
  K20 = P20*S00 + P22*S10 + P24*S20;
  K21 = P20*S01 + P22*S11 + P24*S21;
  K22 = P20*S02 + P22*S12 + P24*S22;
  K30 = P30*S00 + P32*S10 + P34*S20;
  K31 = P30*S01 + P32*S11 + P34*S21;
  K32 = P30*S02 + P32*S12 + P34*S22;
  K40 = P40*S00 + P42*S10 + P44*S20;
  K41 = P40*S01 + P42*S11 + P44*S21;
  K42 = P40*S02 + P42*S12 + P44*S22;
  K50 = P50*S00 + P52*S10 + P54*S20;
  K51 = P50*S01 + P52*S11 + P54*S21;
  K52 = P50*S02 + P52*S12 + P54*S22;

  //State estimation
  errX = phiXa - phiX;
  errY = phiYa - phiY;
  errZ = phiZm - phiZ;

  
  phiX = phiX + K00*errX + K01*errY + K02*errZ;
  bx = bx + K10*errX + K11*errY + K12*errZ;
  phiY = phiY + K20*errX + K21*errY + K22*errZ;
  by = by + K30*errX + K31*errY + K32*errZ;
  phiZ = phiZ + K40*errX + K41*errY + K42*errZ;
  bz = bz + K50*errX + K51*errY + K52*errZ;
  //Covariance update
  

  P00 = - P00*(K00 - 1) - K01*P20 - K02*P40;
  P01 = - P01*(K00 - 1) - K01*P21 - K02*P41;
  P02 = - P02*(K00 - 1) - K01*P22 - K02*P42;
  P03 = - P03*(K00 - 1) - K01*P23 - K02*P43;
  P04 = - P04*(K00 - 1) - K01*P24 - K02*P44;
  P05 = - P05*(K00 - 1) - K01*P25 - K02*P45;

  P10 = P10 - K10*P00 - K11*P20 - K12*P40;
  P11 = P11 - K10*P01 - K11*P21 - K12*P41;
  P12 = P12 - K10*P02 - K11*P22 - K12*P42;
  P13 = P13 - K10*P03 - K11*P23 - K12*P43;
  P14 = P14 - K10*P04 - K11*P24 - K12*P44;
  P15 = P15 - K10*P05 - K11*P25 - K12*P45;

  P20 = - P20*(K21 - 1) - K20*P00 - K22*P40;
  P21 = - P21*(K21 - 1) - K20*P01 - K22*P41;
  P22 = - P22*(K21 - 1) - K20*P02 - K22*P42;
  P23 = - P23*(K21 - 1) - K20*P03 - K22*P43;
  P24 = - P24*(K21 - 1) - K20*P04 - K22*P44;
  P25 = - P25*(K21 - 1) - K20*P05 - K22*P45;

  P30 = P30 - K30*P00 - K31*P20 - K32*P40;
  P31 = P31 - K30*P01 - K31*P21 - K32*P41;
  P32 = P32 - K30*P02 - K31*P22 - K32*P42;
  P33 = P33 - K30*P03 - K31*P23 - K32*P43;
  P34 = P34 - K30*P04 - K31*P24 - K32*P44;
  P35 = P35 - K30*P05 - K31*P25 - K32*P45;

  P40 = - P40*(K42 - 1) - K40*P00 - K41*P20;
  P41 = - P41*(K42 - 1) - K40*P01 - K41*P21;
  P42 = - P42*(K42 - 1) - K40*P02 - K41*P22;
  P43 = - P43*(K42 - 1) - K40*P03 - K41*P23;
  P44 = - P44*(K42 - 1) - K40*P04 - K41*P24;
  P45 = - P45*(K42 - 1) - K40*P05 - K41*P25;

  P50 = P50 - K50*P00 - K51*P20 - K52*P40;
  P51 = P51 - K50*P01 - K51*P21 - K52*P41;
  P52 = P52 - K50*P02 - K51*P22 - K52*P42;
  P53 = P53 - K50*P03 - K51*P23 - K52*P43;
  P54 = P54 - K50*P04 - K51*P24 - K52*P44;
  P55 = P55 - K50*P05 - K51*P25 - K52*P45;

  //Model Prediction

  //State prediction
  phiX = phiX - bx*dt + (gyroX + sin(phiX)*tan(phiY)*gyroY + cos(phiX)*tan(phiY)*gyroZ)*dt;
  phiY = phiY - by*dt + (cos(phiX)*gyroY - sin(phiX)*gyroZ)*dt;
  phiZ = phiZ - bz*dt + ((sin(phiX)/cos(phiY))*gyroY + (cos(phiX)/cos(phiY))*gyroZ)*dt;

  //Covariance prediction

  P00 = P00 + Qa - P10*dt - dt*(P01 - P11*dt);
  P01 = P01 - P11*dt;
  P02 = P02 - P12*dt - dt*(P03 - P13*dt);
  P03 = P03 - P13*dt;
  P04 = P04 - P14*dt - dt*(P05 - P15*dt);
  P05 = P05 - P15*dt;

  P10 = P10 - P11*dt;
  P11 = P11 + Qb;
  P12 = P12 - P13*dt;
  P13 = P13;
  P14 = P14 - P15*dt;
  P15 = P15;

  P20 = P20 - P30*dt - dt*(P21 - P31*dt);
  P21 = P21 - P31*dt;
  P22 = P22 - P32*dt - dt*(P23 - P33*dt) + Qa;
  P23 = P23 - P33*dt;
  P24 = P24 - P34*dt - dt*(P25 - P35*dt);
  P25 = P25 - P35*dt;

  P30 = P30 - P31*dt;
  P31 = P31;
  P32 = P32 - P33*dt;
  P33 = P33 + Qb;
  P34 = P34 - P35*dt;
  P35 = P35;

  P40 = P40 - P50*dt - dt*(P41 - P51*dt);
  P41 = P41 - P51*dt;
  P42 = P42 - P52*dt - dt*(P43 - P53*dt);
  P43 = P43 - P53*dt;
  P44 = P44 - P54*dt - dt*(P45 - P55*dt) + Qa;
  P45 = P45 - P55*dt;

  P50 = P50 - P51*dt;
  P51 = P51;
  P52 = P52 - P53*dt;
  P53 = P53;
  P54 = P54 - P55*dt;
  P55 = P55 + Qb;
  
  
}
