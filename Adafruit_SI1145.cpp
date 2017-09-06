/***************************************************
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_SI1145.h"

Adafruit_SI1145::Adafruit_SI1145(uint8_t address) {
  _addr = address;
  setVisiableOffset(0);
  setIROffset(0);  
}


bool Adafruit_SI1145::begin() {
  Wire.begin();

  uint8_t id = read8(SI1145_REG_PARTID);
  if (id != 0x45) return false; // look for SI1145

  reset();

  // enable sensors
  enableSensor(SENSOR_UV);
  enableSensor(SENSOR_IR);
  enableSensor(SENSOR_VIS);

  // measurement rate for auto
  setMeasureRate(255); // 255 * 31.25uS = 8ms

  // auto run
  setMode(MODE_AUTO);

  return true;
}

void Adafruit_SI1145::reset() {
  setMeasureRate(0);
  write8(SI1145_REG_IRQEN, 0);
  write8(SI1145_REG_IRQMODE1, 0);
  write8(SI1145_REG_IRQMODE2, 0);
  write8(SI1145_REG_INTCFG, 0);
  write8(SI1145_REG_IRQSTAT, 0xFF);

  write8(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  write8(SI1145_REG_HWKEY, 0x17);

  delay(10);
}

//////////////////////////////////////////////////////

// enable sensor component
void Adafruit_SI1145::enableSensor(Sensor_Enable sensor) {
  uint8_t currentCHLIST = readParam(SI1145_PARAM_CHLIST);
  switch (sensor) {
    case SENSOR_UV:
      // enable UVindex measurement coefficients!
      write8(SI1145_REG_UCOEFF0, 0x29);
      write8(SI1145_REG_UCOEFF1, 0x89);
      write8(SI1145_REG_UCOEFF2, 0x02);
      write8(SI1145_REG_UCOEFF3, 0x00);

      writeParam(SI1145_PARAM_CHLIST, currentCHLIST | SI1145_PARAM_CHLIST_ENUV);
      break;
    case SENSOR_AUX:
      writeParam(SI1145_PARAM_CHLIST, currentCHLIST | SI1145_PARAM_CHLIST_ENAUX);
      break;
    case SENSOR_IR:
      writeParam(SI1145_PARAM_CHLIST, currentCHLIST | SI1145_PARAM_CHLIST_ENALSIR);

      writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);
      // fastest clocks, clock div 1
      writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
      // take 511 clocks to measure
      writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
      // in high range mode
      writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);
      break;
    case SENSOR_VIS:
      writeParam(SI1145_PARAM_CHLIST, currentCHLIST | SI1145_PARAM_CHLIST_ENALSVIS);

      // fastest clocks, clock div 1
      writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
      // take 511 clocks to measure
      writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
      // in high range mode (not normal signal)
      writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);
      break;
    case SENSOR_P1:
      proximityEnabled = true;
      writeParam(SI1145_PARAM_CHLIST, currentCHLIST | SI1145_PARAM_CHLIST_ENPS1);

      // program LED current
      write8(SI1145_REG_PSLED21, 0x03); // 20mA for LED 1 only
      writeParam(SI1145_PARAM_PS1ADCMUX, SI1145_PARAM_ADCMUX_LARGEIR);
      // prox sensor #1 uses LED #1
      writeParam(SI1145_PARAM_PSLED12SEL, SI1145_PARAM_PSLED12SEL_PS1LED1);
      // fastest clocks, clock div 1
      writeParam(SI1145_PARAM_PSADCGAIN, 0);
      // take 511 clocks to measure
      writeParam(SI1145_PARAM_PSADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
      // in prox mode, high range
      writeParam(SI1145_PARAM_PSADCMISC, SI1145_PARAM_PSADCMISC_RANGE | SI1145_PARAM_PSADCMISC_PSMODE);
      break;
    case SENSOR_P2:
      proximityEnabled = true;
      break;
    case SENSOR_P3:
      proximityEnabled = true;
      break;
  }
}

// set the mode of the sensor
void Adafruit_SI1145::setMode(Sensor_Mode mode) {
  switch (mode) {
    case MODE_FORCED:
      if (!proximityEnabled){
        setMeasureRate(0);
        write8(SI1145_REG_COMMAND, SI1145_ALS_FORCE);
      }
      else{
        setMeasureRate(0);
        rite8(SI1145_REG_COMMAND, SI1145_PSALS_FORCE);
      }
      break;
    case MODE_AUTO:
      if (!proximityEnabled)
        write8(SI1145_REG_COMMAND, SI1145_ALS_AUTO);
      else
        write8(SI1145_REG_COMMAND, SI1145_PSALS_AUTO);
      break;
  }
}

// set the measure rate
void Adafruit_SI1145::setMeasureRate(uint16_t rate) {
  write8(SI1145_REG_MEASRATE0, 0xFF | rate);
  write8(SI1145_REG_MEASRATE1, 0xFF | (rate >> 8));
}

// enable inerupt
void Adafruit_SI1145::enableInerupt(bool ALS) {
  if (ALS) {
    // enable interrupt on every sample
    write8(SI1145_REG_INTCFG, SI1145_REG_INTCFG_INTOE);
    write8(SI1145_REG_IRQEN, SI1145_REG_IRQEN_ALSEVERYSAMPLE);
  }
  else {

  }
}


//////////////////////////////////////////////////////
// returns the visible gain
Adafruit_SI1145::Gain Adafruit_SI1145::readVisibleGain() {
  return (Adafruit_SI1145::Gain) readParam(SI1145_PARAM_ALSVISADCGAIN);
}

// adjust the visible gain
void Adafruit_SI1145::setVisibleGain(Adafruit_SI1145::Gain gain) {
  uint8_t g = (uint8_t) gain;
  writeParam(SI1145_PARAM_ALSVISADCGAIN, g);
}

// returns the IR gain
Adafruit_SI1145::Gain Adafruit_SI1145::readIRGain() {
  return (Adafruit_SI1145::Gain) readParam(SI1145_PARAM_ALSIRADCGAIN);
}

// adjust the IR gain
void Adafruit_SI1145::setIRGain(Adafruit_SI1145::Gain gain) {
  uint8_t g = (uint8_t) gain;
  writeParam(SI1145_PARAM_ALSIRADCGAIN, g);
}

// take measurement when in forced mode
void Adafruit_SI1145::takeForcedMeasurement() {
  if (read8(SI1145_REG_CHIPSTAT) == SI1145_REG_CHIPSTAT_SLEEP) {
    setMode(MODE_FORCED);
    while (read8(SI1145_REG_CHIPSTAT) != SI1145_REG_CHIPSTAT_SLEEP)
      delay(1);
  }
}

// returns the UV index * 100 (divide by 100 to get the index)
uint16_t Adafruit_SI1145::readUV(void) {
  return read16(0x2C);
}

// returns a calculated lux value as per SI144x AN523.6.
float Adafruit_SI1145::calculateLux(uint16_t vis, uint16_t ir) {
  uint8_t gain = readVisibleGain();
  float lux = ((5.41f * vis) + (-0.08f * ir)) / (1 + 2 * gain);
  return lux;
}

// returns visible+IR light levels
uint16_t Adafruit_SI1145::readVisible(void) {
  uint16_t vis = read16(SI1145_REG_ALSVISDATA0);
  if (vis < _vis_dark)
    _vis_dark = vis;
  vis -= _vis_dark;
  return vis;
}

// returns IR light levels
uint16_t Adafruit_SI1145::readIR(void) {
  uint16_t ir = read16(SI1145_REG_ALSIRDATA0);
  if (ir < _ir_dark)
    _ir_dark = ir;
  ir -= _ir_dark;
  return ir;
}

// returns "Proximity" - assumes an IR LED is attached to LED
uint16_t Adafruit_SI1145::readProx(void) {
  return read16(SI1145_REG_PS1DATA0);
}

// set visiable offset
void Adafruit_SI1145::setVisiableOffset(int16_t val){
  vis_dark = val + 256; // as per Si114x manual - previously ADC_OFFSET but now fixed at 256
}

// set ir offset
void Adafruit_SI1145::setIROffset(int16_t val){
  _ir_dark = val + 256; // as per Si114x manual - previously ADC_OFFSET but now fixed at 256
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v) {
  //Serial.print("Param 0x"); Serial.print(p, HEX);
  //Serial.print(" = 0x"); Serial.println(v, HEX);

  write8(SI1145_REG_PARAMWR, v);
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p) {
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
  return read8(SI1145_REG_PARAMRD);
}

/*********************************************************************/

uint8_t  Adafruit_SI1145::read8(uint8_t reg) {
  uint16_t val;
  Wire.beginTransmission(_addr);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
  return Wire.read();
}

uint16_t Adafruit_SI1145::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission

  Wire.requestFrom(_addr, (uint8_t)2);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= (uint16_t)Wire.read() << 8; // receive DATA

  return ret;
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val) {

  Wire.beginTransmission(_addr); // start transmission to device
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}
