#include "QEC_1X_SPI.h"

QEC_1X::QEC_1X(int CS)
{
    _cs=CS;
}


void QEC_1X::QEC_init(int id_, float scale_, int sign_) {
  _id = id_; _encoderScale = scale_ ; _sign = sign_;
  _encoderCount=0; _encoderOffset=0;
  outDimension=0.0f;
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  this->QEC_config();
}

void QEC_1X::QEC_read(){
  long buff[4];
  digitalWrite(_cs,LOW);
  SPI.transfer(0x60); // Request count , Read Counter
  buff[0] = SPI.transfer(0x00); // most significant byte - 4 byte mode (32 bits)
  buff[1] = SPI.transfer(0x00);
  buff[2] = SPI.transfer(0x00);
  buff[3] = SPI.transfer(0x00); // least significant byte
  digitalWrite(_cs,HIGH); 
  _encoderCount = (long)(buff[0]<<24) + (long)(buff[1]<<16) + (long)(buff[2]<<8) + (long)buff[3] - _encoderOffset;
}

void QEC_1X::QEC_getPose()

{
  this->QEC_read();
  outDimension = (float) _sign * _encoderCount * _encoderScale;
}

void QEC_1X::QEC_config()
{
  SPI.begin();
  delay(10);
  digitalWrite(_cs,LOW);
  SPI.transfer(0x88); //! WRITE_MDR0 
  SPI.transfer(0x03); //! X4 quadrature mode
  digitalWrite(_cs,HIGH);
  delay(10);
  digitalWrite(_cs,LOW);
  SPI.transfer(0x20); //! CLR_COUNTER
  digitalWrite(_cs,HIGH);
}

void QEC_1X::QEC_offset() //To restart the counter.

{
 //!_encoderOffset=_encoderCount; 
 digitalWrite(_cs,LOW);
 SPI.transfer(0x20); /// Clear Counter
 digitalWrite(_cs,HIGH);
}
