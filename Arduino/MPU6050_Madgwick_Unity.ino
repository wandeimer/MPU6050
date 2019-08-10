#include <MadgwickAHRS.h>
#include <MPU6050_jarzebski.h>
#define VECTOR_STRUCT_H
struct myVector
{
    float X;
    float Y;
    float Z;
};

// множитель фильтра
#define BETA 0.22f
#define G 9.88f
#define SCALE 0.015267f
#define RANGE 0.000244f
#define DEGtoRAD 0.0174533f

// an MPU6050 object with the MPU-6050 sensor
MPU6050 IMU_1;
MPU6050 IMU_2;

// переменные для данных с гироскопа, акселерометра и компаса
float movX = 4.0;
myVector ax_raw_1, ax_1, gyro_1;
float quternion_1[4];
float gX_1 = 0;
float gY_1 = 0;
float gZ_1 = 0;
float rotMatrix_1[3][3];
myVector abs_Acc_raw_1, abs_Acc_1;
myVector spd_raw_1, spd_1;
myVector dist_raw_1, dist_1;
myVector correct_ax_1;
bool int_IMU_1;

unsigned long deltaMicros = 8750;
unsigned long timeLastSend = 0;
float fps = 100.0f;

myVector ax_raw_2, ax_2, gyro_2;
float quternion_2[4];
float gX_2 = 0;
float gY_2 = 0;
float gZ_2 = 0;
float rotMatrix_2[3][3];
myVector abs_Acc_raw_2, abs_Acc_2;
myVector spd_raw_2, spd_2;
myVector dist_raw_2, dist_2;
myVector correct_ax_2;
bool int_IMU_2;
 
// создаём объект для фильтра Madgwick
Madgwick filter_1;
Madgwick filter_2;
  
float lowPassFilter(float data, float newData, float filter_koef, float filter_trigger);
float quternionToRotationMatrix(float quaternion[4], float rotMatrix_2[3][3]);

void setup() {
  // serial to display data
  Serial.begin(115200);

  // start communication with IMU 
  if (IMU_1.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_8G, 0x68) == true)
    int_IMU_1 = true;
  else
    int_IMU_1 = false;
  if (IMU_2.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_8G, 0x69) == true)
    int_IMU_2 = true;
  else
    int_IMU_2 = false;

  correct_ax_1.X = -363;
  correct_ax_1.Y = 88;
  correct_ax_1.Z = 1053;

  correct_ax_2.X = 153;
  correct_ax_2.Y = -648;
  correct_ax_2.Z = 1323;

  if (int_IMU_1 == true)
  IMU_1.calibrateGyro();
  if (int_IMU_2 == true)
  IMU_2.calibrateGyro();

}
void loop() {
  // start timer to madgwick filter
  unsigned long startMicros = micros();
  //get data from chip 2
  if (int_IMU_1 == true)
  {
  Vector Gyro_1 = IMU_1.readRawGyro();
  Vector Accel_1 = IMU_1.readRawAccel();

  // amending the data and bringing them to meter per second
  ax_raw_1.X = (Accel_1.XAxis + correct_ax_1.X) * RANGE * G;
  ax_raw_1.Y = (Accel_1.YAxis + correct_ax_1.Y) * RANGE * G;  //убрать умножение на G и добавить после(может повысить точность)
  ax_raw_1.Z = (Accel_1.ZAxis + correct_ax_1.Z) * RANGE * G;

  //filtering accel data
  ax_1.X = lowPassFilter(ax_1.X, ax_raw_1.X, 0.25, 1.5);
  ax_1.Y = lowPassFilter(ax_1.Y, ax_raw_1.Y, 0.25, 1.5);
  ax_1.Z = lowPassFilter(ax_1.Z, ax_raw_1.Z, 0.25, 1.5);
  
  // raw data conversionraw data conversion to rad per second
  gyro_1.X = Gyro_1.XAxis * SCALE * DEGtoRAD;
  gyro_1.Y = Gyro_1.YAxis * SCALE * DEGtoRAD;
  gyro_1.Z = Gyro_1.ZAxis * SCALE * DEGtoRAD;

  // set filter coefficients
  filter_1.setKoeff(fps, BETA);
  // update input to filter
  filter_1.update(gyro_1.X, gyro_1.Y, gyro_1.Z, ax_1.X, ax_1.Y, ax_1.Z, &quternion_1[0], &quternion_1[1], &quternion_1[2], &quternion_1[3]);

  // getting rotation matrix from quaternion// getting rotation matrix from quaternion
  quternionToRotationMatrix(quternion_1, rotMatrix_1);

  // multiplication accel data on rotation matrix to adduction to global coordinate system

  abs_Acc_raw_1.X = ax_1.X * rotMatrix_1[0][0] + ax_1.Y * rotMatrix_1[1][0] + ax_1.Z * rotMatrix_1[2][0];
  abs_Acc_raw_1.Y = ax_1.X * rotMatrix_1[0][1] + ax_1.Y * rotMatrix_1[1][1] + ax_1.Z * rotMatrix_1[2][1];
  abs_Acc_raw_1.Z = ax_1.X * rotMatrix_1[0][2] + ax_1.Y * rotMatrix_1[1][2] + ax_1.Z * rotMatrix_1[2][2];

  //filtering
  abs_Acc_1.X = lowPassFilter(abs_Acc_1.X, abs_Acc_raw_1.X, 0.25, 1.0);
  abs_Acc_1.Y = lowPassFilter(abs_Acc_1.Y, abs_Acc_raw_1.Y, 0.25, 1.0);
  abs_Acc_1.Z = lowPassFilter(abs_Acc_1.Z, abs_Acc_raw_1.Z, 0.25, 1.0);

  //integrate accel to get speed
  spd_raw_1.X = spd_raw_1.X + abs_Acc_1.X * deltaMicros;
  spd_raw_1.Y = spd_raw_1.Y + abs_Acc_1.Y * deltaMicros;
  spd_raw_1.Z = spd_raw_1.Z + abs_Acc_1.Z * deltaMicros;

  // filtering
  spd_1.X = lowPassFilter(spd_1.X, spd_raw_1.X, 0.25, 0.001);
  spd_1.Y = lowPassFilter(spd_1.Y, spd_raw_1.Y, 0.25, 0.001);
  spd_1.Z = lowPassFilter(spd_1.Z, spd_raw_1.Z, 0.25, 0.001);

  // integrate speed to get distance
  dist_raw_1.X = dist_raw_1.X + spd_1.X * deltaMicros;
  dist_raw_1.Y = dist_raw_1.Y + spd_1.Y * deltaMicros;
  dist_raw_1.Z = dist_raw_1.Z + spd_1.Z * deltaMicros;

  // filtering 
  dist_1.X = lowPassFilter(dist_1.X, dist_raw_1.Z, 0.25, 0.001);
  dist_1.Y = lowPassFilter(dist_1.Y, dist_raw_1.Y, 0.25, 0.001);
  dist_1.Z = lowPassFilter(dist_1.Z, dist_raw_1.X, 0.25, 0.001);
  }

  else
  {
  quternion_1[0] = 1;
  quternion_1[1] = 0;
  quternion_1[2] = 0;
  quternion_1[3] = 0;
  dist_1.X = 0;
  dist_1.Y = 0;
  dist_1.Z = 0;
  }

  // обрабатываем второй датчик
  //get data from chip 2
  if (int_IMU_2 == true)
  {
  Vector Gyro_2 = IMU_2.readRawGyro();
  Vector Accel_2 = IMU_2.readRawAccel();

  // amending the data and bringing them to meter per second
  ax_raw_2.X = (Accel_2.XAxis + correct_ax_2.X) * RANGE * G;
  ax_raw_2.Y = (Accel_2.YAxis + correct_ax_2.Y) * RANGE * G;
  ax_raw_2.Z = (Accel_2.ZAxis + correct_ax_2.Z) * RANGE * G;

  //filtering accel data
  ax_2.X = lowPassFilter(ax_2.X, ax_raw_2.X, 0.25, 1.5);
  ax_2.Y = lowPassFilter(ax_2.Y, ax_raw_2.Y, 0.25, 1.5);
  ax_2.Z = lowPassFilter(ax_2.Z, ax_raw_2.Z, 0.25, 1.5);
  
  // raw data conversionraw data conversion to rad per second
  gyro_2.X = Gyro_2.XAxis * SCALE * DEGtoRAD;
  gyro_2.Y = Gyro_2.YAxis * SCALE * DEGtoRAD;
  gyro_2.Z = Gyro_2.ZAxis * SCALE * DEGtoRAD;

  // set filter coefficients
  filter_2.setKoeff(fps, BETA);
  // update input to filter
  filter_2.update(gyro_2.X, gyro_2.Y, gyro_2.Z, ax_2.X, ax_2.Y, ax_2.Z, &quternion_2[0], &quternion_2[1], &quternion_2[2], &quternion_2[3]);

  // getting rotation matrix from quaternion// getting rotation matrix from quaternion
  quternionToRotationMatrix(quternion_2, rotMatrix_2);

  // multiplication accel data on rotation matrix to adduction to global coordinate system
  abs_Acc_raw_2.X = ax_2.X * rotMatrix_2[0][0] + ax_2.Y * rotMatrix_2[1][0] + ax_2.Z * rotMatrix_2[2][0];
  abs_Acc_raw_2.Y = ax_2.X * rotMatrix_2[0][1] + ax_2.Y * rotMatrix_2[1][1] + ax_2.Z * rotMatrix_2[2][1];
  abs_Acc_raw_2.Z = ax_2.X * rotMatrix_2[0][2] + ax_2.Y * rotMatrix_2[1][2] + ax_2.Z * rotMatrix_2[2][2];

  //filtering
  abs_Acc_2.X = lowPassFilter(abs_Acc_2.X, abs_Acc_raw_2.X, 0.25, 1);
  abs_Acc_2.Y = lowPassFilter(abs_Acc_2.Y, abs_Acc_raw_2.Y, 0.25, 1);
  abs_Acc_2.Z = lowPassFilter(abs_Acc_2.Z, abs_Acc_raw_2.Z, 0.25, 1);

  //integrate accel to get speed
  spd_raw_2.X = spd_raw_2.X + abs_Acc_2.X * deltaMicros;
  spd_raw_2.Y = spd_raw_2.Y + abs_Acc_2.Y * deltaMicros;
  spd_raw_2.Z = spd_raw_2.Z + abs_Acc_2.Z * deltaMicros;

  // filtering
  spd_2.X = lowPassFilter(spd_2.X, spd_raw_2.X, 0.25, 0.001);
  spd_2.Y = lowPassFilter(spd_2.Y, spd_raw_2.Y, 0.25, 0.001);
  spd_2.Z = lowPassFilter(spd_2.Z, spd_raw_2.Z, 0.25, 0.001);

  // integrate speed to get distance
  dist_raw_2.X = dist_raw_2.X + spd_2.X * deltaMicros;
  dist_raw_2.Y = dist_raw_2.Y + spd_2.Y * deltaMicros;
  dist_raw_2.Z = dist_raw_2.Z + spd_2.Z * deltaMicros;

  // filtering 
  dist_2.X = lowPassFilter(dist_2.X, dist_raw_2.Z, 0.25, 0.001);
  dist_2.Y = lowPassFilter(dist_2.Y, dist_raw_2.Y, 0.25, 0.001);
  dist_2.Z = lowPassFilter(dist_2.Z, dist_raw_2.X, 0.25, 0.001);
  }
  
  else
  {
  quternion_2[0] = 1;
  quternion_2[1] = 0;
  quternion_2[2] = 0;
  quternion_2[3] = 0;
  dist_2.X = 0;
  dist_2.Y = 0;
  dist_2.Z = 0;
  }
int i = 0;
  // time cut-off for sending data with frequency 40 HZ
  unsigned long timeToSend = micros() - timeLastSend;
  if(timeToSend >= 17000){
        movX -= 0.01;
    timeToSend = 0;
    // sending data
    
    Serial.print(quternion_2[1], 2);
    Serial.print(", ");
    Serial.print(quternion_2[2], 2);
    Serial.print(", ");
    Serial.print(quternion_2[3], 2);
    Serial.print(", ");
    Serial.print(quternion_2[0], 2);
    Serial.print(", ");
    Serial.print(dist_1.X, 2);
    Serial.print(", ");
    Serial.print(dist_1.Y, 2);
    Serial.print(", ");
    Serial.print(dist_1.Z, 2);
    Serial.print(", ");
    Serial.print(quternion_1[1], 2);
    Serial.print(", ");
    Serial.print(quternion_1[2], 2);
    Serial.print(", ");
    Serial.print(quternion_1[3], 2);
    Serial.print(", ");
    Serial.print(quternion_1[0], 2);
    Serial.print(", ");
    Serial.print(dist_2.X, 2);
    Serial.print(", ");
    Serial.print(dist_2.Y, 2);
    Serial.print(", ");
    Serial.print(dist_2.Z, 2);
    Serial.flush(); // Empty the memory each time in the loop
    Serial.println();      
    
    timeLastSend = micros();
   }

  //get fps to madgwick filter
  deltaMicros = micros() - startMicros;
  fps = 1000000 / deltaMicros; 
}

float lowPassFilter(float data, float newData, float filter_koef, float filter_trigger)
{
  if ((newData - data < filter_trigger) || (data - newData < filter_trigger)){
    data = data + filter_koef * (newData - data);
  }
  else{
    data = newData;
  }
  return data;
}

float quternionToRotationMatrix(float quaternion[4], float rotMatrix_2[3][3])
{
  // getting rotation matrix from quaternion
  rotMatrix_2[0][0] = 1 - 2 * (quaternion[2] * quaternion[2]) - 2 * (quaternion[3] * quaternion[3]);
  rotMatrix_2[0][1] = 2 * quaternion[1] * quaternion[2] - 2 * quaternion[3] * quaternion[0];
  rotMatrix_2[0][2] = 2 * quaternion[1] * quaternion[3] + 2 * quaternion[2] * quaternion[0];
  rotMatrix_2[1][0] = 2 * quaternion[1] * quaternion[2] + 2 * quaternion[3] * quaternion[0];
  rotMatrix_2[1][1] = 1 - 2 * (quaternion[1] * quaternion[1]) - 2 * (quaternion[3] * quaternion[3]);
  rotMatrix_2[1][2] = 2 * quaternion[2] * quaternion[3] - 2 * quaternion[1] * quaternion[0];
  rotMatrix_2[2][0] = 2 * quaternion[1] * quaternion[3] - 2 * quaternion[2] * quaternion[0];
  rotMatrix_2[2][1] = 2 * quaternion[2] * quaternion[3] + 2 * quaternion[1] * quaternion[0];
  rotMatrix_2[2][2] = 1 - 2 * (quaternion[1] * quaternion[1]) - 2 * (quaternion[2] * quaternion[2]);

  return 0;
}
