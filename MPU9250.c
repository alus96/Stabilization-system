#include "stm32f4xx_hal.h"
#include "math.h"
#include "delay_hal.h"
#include "MPU9250.h"

SPI_HandleTypeDef * hspi;

uint8_t Ascale = AFS_2G;     
uint8_t Gscale = GFS_250DPS; 
uint8_t Mscale = MFS_16BITS; 
uint8_t Mmode = 0x06;        
float aRes, gRes, mRes;      

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values


// parameters for 6 DoF sensor fusion calculations
float PI = 3.14159265358979323846f;
float GyroMeasError;// = PI * (60.0f / 180.0f);     
float GyroMeasDrift;// = PI * (1.0f / 180.0f);     
float zeta;// = sqrt(3.0f / 4.0f) * GyroMeasDrift;  
#define Kp 0.5f 
#define Ki 0.01f

// imuStruct imuData;
//float pitch, yaw, roll;                           
int lastUpdate = 0, firstUpdate = 0, Now = 0;                                
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};          
float eInt[3] = {0.0f, 0.0f, 0.0f};              

uint32_t MPU9250_lastUpdate, MPU9250_firstUpdate, MPU9250_now;
float MPU9250_deltat;// = 0.0f;                              

int16_t ax_raw,ay_raw,az_raw,
        gx_raw,gy_raw,gz_raw,
		mx_raw,my_raw,mz_raw;
float ax,ay,az,gx,gy,gz,mx,my,mz;


uint8_t writeReg( uint8_t writeAddr, uint8_t writeData )
{
	uint8_t address = 0;
	uint8_t data = 0;

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
	address = writeAddr;
	HAL_SPI_TransmitReceive(hspi, &address, &data, 1, 0x1000);
	address = writeData;
	HAL_SPI_TransmitReceive(hspi, &address, &data, 1, 0x1000);
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

	return data;
}

uint8_t readReg( uint8_t readAddr)
{
	return writeReg( readAddr | READ_FLAG, 0x00);
}

void readRegs( uint8_t readAddr, uint8_t count, uint8_t * dest)
{
	unsigned int  i = 0;

	uint8_t address = 0;
	uint8_t data = 0;

	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

	address = readAddr | READ_FLAG;
	HAL_SPI_TransmitReceive(hspi, &address, &data, 1, 0x1000);
	//HAL_Delay(5);
	address = 0x00;
	for(i=0; i<count; i++){
	    	HAL_SPI_TransmitReceive(hspi, &address, &dest[i], 1, 0x1000);
	    	//HAL_Delay(5);
	}
	HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);

}

uint8_t readMagReg(uint8_t subAddr)
{
  writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS|0b10000000);
  writeReg(I2C_SLV0_REG, subAddr);
  writeReg(I2C_SLV0_CTRL, 0x81);
  HAL_Delay(2);
  uint8_t rb = readReg(EXT_SENS_DATA_00);

  return rb;
}

void readMagRegs(uint8_t subAddr, uint8_t count, uint8_t *dest)
{
  int i = 0;
  for(; i < count; ++i){
    writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS|0b10000000);
    writeReg(I2C_SLV0_REG, subAddr+i);
    writeReg(I2C_SLV0_CTRL,0x81);
    HAL_Delay(2);
    dest[i] = readReg(EXT_SENS_DATA_00);

  }

}

void writeMagReg(uint8_t addr,uint8_t data){
    writeReg(I2C_SLV0_ADDR, AK8963_ADDRESS&0b01111111);
    writeReg(I2C_SLV0_REG, addr);
    writeReg(I2C_SLV0_DO, data);
    writeReg(I2C_SLV0_CTRL, 0x81);
    HAL_Delay(2);
}


uint8_t initMPU(SPI_HandleTypeDef * spi)
{
	hspi = spi;

	GyroMeasError = PI * (60.0f / 180.0f);    
	beta = sqrt(3.0f / 4.0f) * GyroMeasError; 
	GyroMeasDrift = PI * (1.0f / 180.0f);     
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  

	resetMPU9250(); 
	HAL_Delay(200);

	HAL_Delay(200);
	initMPU9250();
	getAres(); /
	getGres(); 
	getMres(); 

	return readReg(WHO_AM_I_MPU9250);
}

void getMres()
{
  switch (Mscale)
  {
       case MFS_14BITS:
          mRes = 10.0*4219.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.0*4219.0/32760.0; // Proper scale to return milliGauss
          break;
  }
}

void getGres()
{
  switch (Gscale)
  {
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}


void getAres()
{
  switch (Ascale)
  {
     case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}


void MPU9250_getAcceleration(int16_t * ax_raw, int16_t * ay_raw, int16_t * az_raw)
{
  uint8_t rawData[6]; 
  readRegs(ACCEL_XOUT_H, 6, &rawData[0]); 
  * ax_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; 
  * ay_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  * az_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void MPU9250_getRotation(int16_t * gx_raw, int16_t * gy_raw, int16_t * gz_raw)
{
  uint8_t rawData[6];  
  readRegs(GYRO_XOUT_H, 6, &rawData[0]); 
  * gx_raw = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  
  * gy_raw = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
  * gz_raw = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

void MPU9250_getCompass(int16_t * mx_raw, int16_t * my_raw, int16_t * mz_raw)
{
  uint8_t rawData[7];  
  if(readMagReg(AK8963_ST1) & 0x01) { 
//    HAL_Delay(1);
	 readMagRegs(AK8963_XOUT_L, 7, &rawData[0]);  
    uint8_t c = rawData[6]; 
    if(!(c & 0x08)) { 
    	* mx_raw = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
    	* my_raw = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
    	* mz_raw = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
    }
  }
}

void resetMPU9250()
{
  
  writeReg(PWR_MGMT_1, 0x80); 
  HAL_Delay(100);
}



void initMPU9250()
{

	uint8_t tries;
    for (tries = 0; tries<5; tries++){
	    writeReg(PWR_MGMT_1,0x80);
        HAL_Delay(100);
    }

	writeReg(PWR_MGMT_1, 0x01); 
	writeReg(PWR_MGMT_2, 0x00); 

	HAL_Delay(200); 

	writeReg(FIFO_EN,0x00);
	writeReg(PWR_MGMT_1,0x01);
	HAL_Delay(100);

	writeReg(CONFIG, 0b01000011);

	writeReg(SMPLRT_DIV, 0x00);
	

	writeReg(GYRO_CONFIG, 0x00 );
	writeReg(ACCEL_CONFIG, 0x00);

	writeReg(USER_CTRL,0x20);
	writeReg(I2C_MST_CTRL,0x0D);

	writeReg(I2C_SLV0_ADDR,AK8963_ADDRESS);
	writeReg(I2C_SLV0_REG,AK8963_CNTL2);
	writeReg(I2C_SLV0_CTRL,0x81);
	writeReg(I2C_SLV0_DO,0x01);
	writeReg(I2C_SLV0_CTRL,0x81);
	HAL_Delay(500);

	writeReg(I2C_SLV0_ADDR,AK8963_ADDRESS);
	writeReg(I2C_SLV0_REG,AK8963_CNTL1);
	writeReg(I2C_SLV0_CTRL,0x81);
	writeReg(I2C_SLV0_DO,0x16);
	writeReg(I2C_SLV0_CTRL,0x81);
	HAL_Delay(100);

	writeReg(I2C_SLV0_ADDR,AK8963_ADDRESS|READ_FLAG);
	writeReg(I2C_SLV0_REG,AK8963_CNTL1);
	writeReg(I2C_SLV0_CTRL,0x81);
	writeReg(I2C_SLV0_DO,0x16);
	writeReg(I2C_SLV0_CTRL,0x81);
	writeReg(I2C_SLV0_REG,AK8963_XOUT_L);
	writeReg(I2C_SLV0_CTRL,0x86);

	writeReg(USER_CTRL,0x0C);
	HAL_Delay(150);

	writeReg(USER_CTRL,0b01000000);
	//HAL_Delay(100);
	//writeReg(USER_CTRL,0x0C);
	writeReg(FIFO_EN, 0b01111001);
	HAL_Delay(150);	


}

void MPU9250_MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	float _8bx = 2.0f * _4bx;
	float _8bz = 2.0f * _4bz;

	// Gradient decent algorithm corrective step
	s1= -_2q3*(2*(q2q4 - q1q3) - ax) + _2q2*(2*(q1q2 + q3q4) - ay) + -_4bz*q3*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx) + (-_4bx*q4+_4bz*q2)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my) + _4bx*q3*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s2= _2q4*(2*(q2q4 - q1q3) - ax) + _2q1*(2*(q1q2 + q3q4) - ay) + -4*q2*(2*(0.5 - q2q2 - q3q3) - az) + _4bz*q4*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx) + (_4bx*q3+_4bz*q1)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my) + (_4bx*q4-_8bz*q2)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s3= -_2q1*(2*(q2q4 - q1q3) - ax) + _2q4*(2*(q1q2 + q3q4) - ay) + (-4*q3)*(2*(0.5 - q2q2 - q3q3) - az) + (-_8bx*q3-_4bz*q1)*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx)+(_4bx*q2+_4bz*q4)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my)+(_4bx*q1-_8bz*q3)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	s4= _2q2*(2*(q2q4 - q1q3) - ax) + _2q3*(2*(q1q2 + q3q4) - ay)+(-_8bx*q4+_4bz*q2)*(_4bx*(0.5 - q3q3 - q4q4) + _4bz*(q2q4 - q1q3) - mx)+(-_4bx*q1+_4bz*q3)*(_4bx*(q2q3 - q1q4) + _4bz*(q1q2 + q3q4) - my)+(_4bx*q2)*(_4bx*(q1q3 + q2q4) + _4bz*(0.5 - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * MPU9250_deltat;
	q2 += qDot2 * MPU9250_deltat;
	q3 += qDot3 * MPU9250_deltat;
	q4 += qDot4 * MPU9250_deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];         // short name local variable for readability
  float norm;                                               // vector norm
  float f1, f2, f3;                                         // objective funcyion elements
  float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
  float qDot1, qDot2, qDot3, qDot4;
  float hatDot1, hatDot2, hatDot3, hatDot4;
  float gerrx, gerry, gerrz, gbiasx = 0.0f, gbiasy = 0.0f, gbiasz = 0.0f;  // gyro bias error

  // Auxiliary variables to avoid repeated arithmetic
  float _halfq1 = 0.5f * q1;
  float _halfq2 = 0.5f * q2;
  float _halfq3 = 0.5f * q3;
  float _halfq4 = 0.5f * q4;
  float _2q1 = 2.0f * q1;
  float _2q2 = 2.0f * q2;
  float _2q3 = 2.0f * q3;
  float _2q4 = 2.0f * q4;
  //            float _2q1q3 = 2.0f * q1 * q3;
  //            float _2q3q4 = 2.0f * q3 * q4;

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f/norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Compute the objective function and Jacobian
  f1 = _2q2 * q4 - _2q1 * q3 - ax;
  f2 = _2q1 * q2 + _2q3 * q4 - ay;
  f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
  J_11or24 = _2q3;
  J_12or23 = _2q4;
  J_13or22 = _2q1;
  J_14or21 = _2q2;
  J_32 = 2.0f * J_14or21;
  J_33 = 2.0f * J_11or24;

  // Compute the gradient (matrix multiplication)
  hatDot1 = J_14or21 * f2 - J_11or24 * f1;
  hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
  hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
  hatDot4 = J_14or21 * f1 + J_11or24 * f2;

  // Normalize the gradient
  norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
  hatDot1 /= norm;
  hatDot2 /= norm;
  hatDot3 /= norm;
  hatDot4 /= norm;

  // Compute estimated gyroscope biases
  gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
  gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
  gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

  // Compute and remove gyroscope biases
  gbiasx += gerrx * MPU9250_deltat * zeta;
  gbiasy += gerry * MPU9250_deltat * zeta;
  gbiasz += gerrz * MPU9250_deltat * zeta;
  gx -= gbiasx;
  gy -= gbiasy;
  gz -= gbiasz;

  // Compute the quaternion derivative
  qDot1 = -_halfq2 * gx - _halfq3 * gy - _halfq4 * gz;
  qDot2 =  _halfq1 * gx + _halfq3 * gz - _halfq4 * gy;
  qDot3 =  _halfq1 * gy - _halfq2 * gz + _halfq4 * gx;
  qDot4 =  _halfq1 * gz + _halfq2 * gy - _halfq3 * gx;

  // Compute then integrate estimated quaternion derivative
  q1 += (qDot1 -(beta * hatDot1)) * MPU9250_deltat;
  q2 += (qDot2 -(beta * hatDot2)) * MPU9250_deltat;
  q3 += (qDot3 -(beta * hatDot3)) * MPU9250_deltat;
  q4 += (qDot4 -(beta * hatDot4)) * MPU9250_deltat;

  // Normalize the quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
  norm = 1.0f/norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}


void MPU9250_TakeAndCalcData()
{
  int i;

  //MPU9250_getMotion6(&ax_raw,&ay_raw,&az_raw,&gx_raw,&gy_raw,&gz_raw);
/*
  MPU9250_getAcceleration(&ax_raw,&ay_raw,&az_raw);
  MPU9250_getRotation(&gx_raw,&gy_raw,&gz_raw);
  int c1 = HAL_GetTick();

  MPU9250_getCompass(&mx_raw,&my_raw,&mz_raw);*/
  uint8_t data[2];
 // readRegs(FIFO_COUNTH,2,data);
  //uint16_t d =  (uint16_t)((uint16_t)(data[0] << 8) | data[1]);
  //printf("%d\n",d);

  uint8_t rawData[36];
  readRegs(FIFO_R_W,36,&rawData[0]);
  writeReg(USER_CTRL,0b01000100);
  
  ax_raw =  (int16_t)(((int16_t) rawData[0] << 8) | rawData[1]);
  ay_raw =  (int16_t)(((int16_t) rawData[2] << 8) | rawData[3]);
  az_raw =  (int16_t)(((int16_t) rawData[4] << 8) | rawData[5]);

  gx_raw =  (int16_t)(((int16_t) rawData[6] << 8) | rawData[7]);
  gy_raw =  (int16_t)(((int16_t) rawData[8] << 8) | rawData[9]);
  gz_raw =  (int16_t)(((int16_t) rawData[10] << 8) | rawData[11]);

  mx_raw =  (int16_t)(((int16_t) rawData[13] << 8) | rawData[12]);
  my_raw =  (int16_t)(((int16_t) rawData[15] << 8) | rawData[14]);
  mz_raw =  (int16_t)(((int16_t) rawData[17] << 8) | rawData[16]);


  ax = (float)ax_raw * aRes;  
  ay = (float)ay_raw * aRes;
  az = (float)az_raw * aRes;

  gx = (float)gx_raw * gRes;  
  gy = (float)gy_raw * gRes;
  gz = (float)gz_raw * gRes;


  mx = 0.9946214903*mx_raw +0.0549210486*my_raw+0.0320404556*mz_raw+20.539999;
  my =  -0.0875282929*mx_raw + 0.9934846275*my_raw+0.0730786347*mz_raw-282.00888;
  mz = -0.0172358411*mx_raw  -0.0230787479*my_raw+0.9977625377*mz_raw-30.009537;// - gyroBias[2];
  
 // mx = (float)mx_raw * mRes;  
//  my = (float)my_raw * mRes;
 // mz = (float)mz_raw * mRes;

  //debug_take();

  for (i = 1; i <= 1; i++)
  {
    MPU9250_now = HAL_GetTick();
    MPU9250_deltat = (float)((MPU9250_now - MPU9250_lastUpdate)/1000.0f) ; // set integration time by time elapsed since last filter update
    MPU9250_lastUpdate = MPU9250_now;

    MPU9250_MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz );
    //MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f);

  }

  if (MPU9250_lastUpdate - MPU9250_firstUpdate > 10000.0f) {
    //beta = 0.04;  
    //zeta = 0.015; 
    beta = 0.09;  
    zeta = 0.005;
  }

}

void debug_calc(){
	printf("Yaw, Pitch, Roll: %f %f %f\n\r", imuData.yaw, imuData.pitch, imuData.roll);
}

void MPU9250_CalcYPR()
{
  imuData.yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  imuData.pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  imuData.roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
  imuData.pitch *= 180.0f / PI;
  imuData.yaw   *= 180.0f / PI;
  imuData.roll  *= 180.0f / PI;

 // debug_calc();
}

imuStruct * imuGetPtr(){
	return &imuData;
}

