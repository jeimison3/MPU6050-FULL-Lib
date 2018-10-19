//Baseado em: https://github.com/jarzebski/Arduino-MPU6050/blob/master/MPU6050.cpp
#include <stdint>
#include "mpulib.h"


bool MPU6050::setup(mpu6050_dps_t scale, mpu6050_range_t range, int i2cAddr){
	char fileName[] = "/dev/i2c-1";
  //Abre ponteiro para arquivo
	if ((fd = open(fileName, O_RDWR)) < 0) {
        printf("Falha ao abrir porta I2C\n");
        return false;
  }
  if (ioctl(fd, I2C_SLAVE, i2cAddr) < 0) {
      printf("Não foi possivel obter acesso do barramento.\n");
      return false;
  }

	//Reseta valores de calibragem:
	dg.XAxis = 0;
  dg.YAxis = 0;
  dg.ZAxis = 0;
  useCalibrate = false;

  //Reseta limiares
  tg.XAxis = 0;
  tg.YAxis = 0;
  tg.ZAxis = 0;
  actualThreshold = 0;

  setScale(scale);
  setRange(range);

  //Equivalente a iniciar:
  setSleepEnabled(false);

  return read8(MPU6050_REG_WHO_AM_I) == 0x68;
}

void MPU6050::write16(int Addr, int16_t data){
  write8(Addr,(int8_t) (data>>8));
  write8(Addr+1,(int8_t) data);
}

bool MPU6050::readRbit(int reg, int pos) {
    uint8_t value;
    value = read8(reg);
    return ((value >> pos) & 1); //Isola bit
}

void MPU6050::writeRbit(int reg, int pos, bool state) {
    uint8_t value;
    value = read8(reg);
    if (state) {
        value |= (1 << pos);
    } else {
        value &= ~(1 << pos);
    }
    write8(reg, value);
}


//
// Operações de PROCESSAMENTO
//

void MPU6050::readRawGyro(){
  rg.XAxis = read16(MPU6050_REG_GYRO_XOUT_H);
  rg.YAxis = read16(MPU6050_REG_GYRO_YOUT_H);
  rg.ZAxis = read16(MPU6050_REG_GYRO_ZOUT_H);
}

void MPU6050::readRawAccel(){
  ra.XAxis = read16(MPU6050_REG_ACCEL_XOUT_H);
  ra.YAxis = read16(MPU6050_REG_ACCEL_YOUT_H);
  ra.ZAxis = read16(MPU6050_REG_ACCEL_ZOUT_H);
}

void MPU6050::calibrateGyro(int amostras) {
    // Define calibragem
    useCalibrate = true;

    // Reseta valores
    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    float sigmaX = 0;
    float sigmaY = 0;
    float sigmaZ = 0;

    // Lê n-amostras
    for (int i = 0; i < amostras; ++i) {
	     readRawGyro();
	      sumX += rg.XAxis;
	      sumY += rg.YAxis;
	      sumZ += rg.ZAxis;

	      sigmaX += rg.XAxis * rg.XAxis;
	      sigmaY += rg.YAxis * rg.YAxis;
	      sigmaZ += rg.ZAxis * rg.ZAxis;
	      sleep(5 * 1000);
    }

    // Calcula vetores delta
    dg.XAxis = sumX / amostras;
    dg.YAxis = sumY / amostras;
    dg.ZAxis = sumZ / amostras;

    // Calcula vetores de limite
    th.XAxis = sqrt((sigmaX / 50) - (dg.XAxis * dg.XAxis));
    th.YAxis = sqrt((sigmaY / 50) - (dg.YAxis * dg.YAxis));
    th.ZAxis = sqrt((sigmaZ / 50) - (dg.ZAxis * dg.ZAxis));

    // Se atingiu o limite, recalcula vetores de limite
    if (actualThreshold > 0) {
	     setThreshold(actualThreshold);
    }
}



void MPU6050::setThreshold(int multiple) {
  if (multiple > 0) {
	   if (!useCalibrate) calibrateGyro();

	   // Calcula vetor de limiares
	   tg.XAxis = th.XAxis * multiple;
	   tg.YAxis = th.YAxis * multiple;
	   tg.ZAxis = th.ZAxis * multiple;
  } else { // Se sem limiares
	   tg.XAxis = 0;
	   tg.YAxis = 0;
	   tg.ZAxis = 0;
  }
  // Remember old threshold value
  actualThreshold = multiple;
}

float MPU6050::getTemp(){
  int16_t T;
  T = read16(MPU6050_REG_TEMP_OUT_H);
  return (float)T/340 + 36.53;
}


void MPU6050::setClockSource(mpu6050_clockSource_t fonte) {
    uint8_t value;
    value = read8(MPU6050_REG_PWR_MGMT_1); // Lê
    value &= 0b11111000;
    value |= fonte; // Mascara
    write8(MPU6050_REG_PWR_MGMT_1, value); // Cast
}

mpu6050_clockSource_t MPU6050::getClockSource() {
  uint8_t value;
  value = read8(MPU6050_REG_PWR_MGMT_1); // Lê
  value &= 0b00000111; // Mascara
  return (mpu6050_clockSource_t)value; // Cast
}

void MPU6050::setScale(mpu6050_dps_t escala) {
    uint8_t value;
    switch (escala) {
      case MPU6050_SCALE_250DPS:
	      dpsPerDigit = .007633f;
	      break;
	    case MPU6050_SCALE_500DPS:
	      dpsPerDigit = .015267f;
	      break;
	    case MPU6050_SCALE_1000DPS:
	      dpsPerDigit = .030487f;
	      break;
	    case MPU6050_SCALE_2000DPS:
	      dpsPerDigit = .060975f;
	      break;
	    default:
	      break;
    }

    value = read8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b11100111;
    value |= (escala << 3);
    write8(MPU6050_REG_GYRO_CONFIG, value);
}

mpu6050_dps_t MPU6050::getScale() {
    uint8_t value;
    value = read8(MPU6050_REG_GYRO_CONFIG);
    value &= 0b00011000; // Mascara
    value >>= 3;
    return (mpu6050_dps_t)value; // Cast
}

void MPU6050::setRange(mpu6050_range_t range) {
    uint8_t value;
    switch (range) {
	     case MPU6050_RANGE_2G:
	       rangePerDigit = .000061f;
	       break;
	     case MPU6050_RANGE_4G:
	       rangePerDigit = .000122f;
	       break;
	     case MPU6050_RANGE_8G:
	       rangePerDigit = .000244f;
	       break;
	     case MPU6050_RANGE_16G:
	       rangePerDigit = .0004882f;
	       break;
	     default:
	       break;
    }

    value = read8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b11100111;
    value |= (range << 3);
    write8(MPU6050_REG_ACCEL_CONFIG, value);
}

mpu6050_range_t MPU6050::getRange() {
    uint8_t value;
    value = read8(MPU6050_REG_ACCEL_CONFIG);
    value &= 0b00011000; // Mascara
    value >>= 3;
    return (mpu6050_range_t) value; // Cast
}

void MPU6050::setAccelPowerOnDelay(mpu6050_onDelay_t delay) {
    uint8_t value;
    value = read8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b11001111;
    value |= (delay << 4);
    write8(MPU6050_REG_MOT_DETECT_CTRL, value);
}

mpu6050_onDelay_t MPU6050::getAccelPowerOnDelay() {
    uint8_t value;
    value = read8(MPU6050_REG_MOT_DETECT_CTRL);
    value &= 0b00110000; // Mascara
    return (mpu6050_onDelay_t)(value >> 4); // Cast
}

//
// Processamento de EIXOS:
//

Vector MPU6050::readNormalizeGyro(){
    readRawGyro();
    if (useCalibrate){
      ng.XAxis = (rg.XAxis - dg.XAxis) * dpsPerDigit;
      ng.YAxis = (rg.YAxis - dg.YAxis) * dpsPerDigit;
      ng.ZAxis = (rg.ZAxis - dg.ZAxis) * dpsPerDigit;
    } else {
      ng.XAxis = rg.XAxis * dpsPerDigit;
      ng.YAxis = rg.YAxis * dpsPerDigit;
      ng.ZAxis = rg.ZAxis * dpsPerDigit;
    }
    if (actualThreshold){
      if (abs(ng.XAxis) < tg.XAxis) ng.XAxis = 0;
      if (abs(ng.YAxis) < tg.YAxis) ng.YAxis = 0;
      if (abs(ng.ZAxis) < tg.ZAxis) ng.ZAxis = 0;
    }
    return ng;
}

Vector MPU6050::readNormalizeAccel() {
    readRawAccel();
    na.XAxis = ra.XAxis * rangePerDigit * 9.80665f;
    na.YAxis = ra.YAxis * rangePerDigit * 9.80665f;
    na.ZAxis = ra.ZAxis * rangePerDigit * 9.80665f;
    return na;
}

Vector MPU6050::readScaledAccel() {
    readRawAccel();
    na.XAxis = ra.XAxis * rangePerDigit;
    na.YAxis = ra.YAxis * rangePerDigit;
    na.ZAxis = ra.ZAxis * rangePerDigit;
    return na;
}

//
// Relacionado aos EIXOS:
//

int16_t MPU6050::getGyroOffsetX() {
    return read16(MPU6050_REG_GYRO_XOFFS_H);
}

int16_t MPU6050::getGyroOffsetY() {
    return read16(MPU6050_REG_GYRO_YOFFS_H);
}

int16_t MPU6050::getGyroOffsetZ() {
    return read16(MPU6050_REG_GYRO_ZOFFS_H);
}

void MPU6050::setGyroOffsetX(int16_t offset){
    write16(MPU6050_REG_GYRO_XOFFS_H, offset);
}

void MPU6050::setGyroOffsetY(int16_t offset){
    write16(MPU6050_REG_GYRO_YOFFS_H, offset);
}

void MPU6050::setGyroOffsetZ(int16_t offset){
    write16(MPU6050_REG_GYRO_ZOFFS_H, offset);
}

int16_t MPU6050::getAccelOffsetX(){
    return read16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t MPU6050::getAccelOffsetY(){
    return read16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t MPU6050::getAccelOffsetZ(){
    return read16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void MPU6050::setAccelOffsetX(int16_t offset){
    write16(MPU6050_REG_ACCEL_XOFFS_H, offset);
}

void MPU6050::setAccelOffsetY(int16_t offset) {
    write16(MPU6050_REG_ACCEL_YOFFS_H, offset);
}

void MPU6050::setAccelOffsetZ(int16_t offset) {
    write16(MPU6050_REG_ACCEL_ZOFFS_H, offset);
}


//
// Operações em BITS de REGISTRADORES
//

bool MPU6050::getSleepEnabled() {
	return readRbit(MPU6050_REG_PWR_MGMT_1, 6);
}

void MPU6050::setSleepEnabled(bool state) {
  writeRbit(MPU6050_REG_PWR_MGMT_1, 6, state);
}

bool MPU6050::getI2CMasterModeEnabled() {
    return readRbit(MPU6050_REG_USER_CTRL, 5);
}

void MPU6050::setI2CMasterModeEnabled(bool state) {
    writeRbit(MPU6050_REG_USER_CTRL, 5, state);
}

bool MPU6050::getI2CBypassEnabled() {
    return readRbit(MPU6050_REG_INT_PIN_CFG, 1);
}

void MPU6050::setI2CBypassEnabled(bool state) {
	return writeRbit(MPU6050_REG_INT_PIN_CFG, 1, state);
}

bool MPU6050::getIntZeroMotionEnabled() {
	return readRbit(MPU6050_REG_INT_ENABLE, 5);
}

void MPU6050::setIntZeroMotionEnabled(bool state){
    writeRbit(MPU6050_REG_INT_ENABLE, 5, state);
}

bool MPU6050::getIntMotionEnabled() {
    return readRbit(MPU6050_REG_INT_ENABLE, 6);
}

void MPU6050::setIntMotionEnabled(bool state) {
    writeRbit(MPU6050_REG_INT_ENABLE, 6, state);
}

bool MPU6050::getIntFreeFallEnabled() {
    return readRbit(MPU6050_REG_INT_ENABLE, 7);
}

void MPU6050::setIntFreeFallEnabled(bool state) {
    writeRbit(MPU6050_REG_INT_ENABLE, 7, state);
}


//
// Operações em REGISTRADORES
//

uint8_t MPU6050::getMotionDetectionThreshold() {
    return read8(MPU6050_REG_MOT_THRESHOLD);
}

void MPU6050::setMotionDetectionThreshold(uint8_t threshold) {
    write8(MPU6050_REG_MOT_THRESHOLD, threshold);
}

uint8_t MPU6050::getMotionDetectionDuration() {
    return read8(MPU6050_REG_MOT_DURATION);
}

void MPU6050::setMotionDetectionDuration(uint8_t duration) {
    write8(MPU6050_REG_MOT_DURATION, duration);
}

uint8_t MPU6050::getZeroMotionDetectionThreshold() {
    return read8(MPU6050_REG_ZMOT_THRESHOLD);
}

void MPU6050::setZeroMotionDetectionThreshold(uint8_t threshold) {
    write8(MPU6050_REG_ZMOT_THRESHOLD, threshold);
}

uint8_t MPU6050::getZeroMotionDetectionDuration() {
    return read8(MPU6050_REG_ZMOT_DURATION);
}

void MPU6050::setZeroMotionDetectionDuration(uint8_t duration) {
    write8(MPU6050_REG_ZMOT_DURATION, duration);
}

uint8_t MPU6050::getFreeFallDetectionThreshold() {
    return read8(MPU6050_REG_FF_THRESHOLD);
}

void MPU6050::setFreeFallDetectionThreshold(uint8_t threshold) {
    write8(MPU6050_REG_FF_THRESHOLD, threshold);
}

uint8_t MPU6050::getFreeFallDetectionDuration() {
    return read8(MPU6050_REG_FF_DURATION);
}

void MPU6050::setFreeFallDetectionDuration(uint8_t duration) {
    write8(MPU6050_REG_FF_DURATION, duration);
}
