#ifndef __QEC_1X_SPI_H__
#define __QEC_1X_SPI_H__

#include <SPI.h>

class QEC_1X
{
	public:
		QEC_1X(int CS); 
		void QEC_init(int id_, float scale_, int sign_);
		float outDimension;  
		void QEC_read(void);
		void QEC_getPose(void);
		void QEC_config(void);
		void QEC_offset(void);
		
	private:
		long _encoderCount;
		int _cs;
		int _id;
		float _encoderScale;
		int _sign;
		long _encoderOffset;
};

#endif /*__QEC_1X_SPI_H__*/