#ifndef FILTER_H
#define FILTER_H
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <stdio.h>

//Класс реализует очиску от помех для измеряемых датчиками значений. При получении менее 5 значений - возвращается среднее, если значений больше - из последних 5 значений 
//отбрасываются иминимальное и максимально и возвращается среднее по 3м оставшимся.
class F5_float {
	private:
		float val[5], buf[5];
		uint8_t cnt;
	public:
		F5_float();
		void Write(float);
		float Read();
		//write		
		F5_float& operator=(const float right) {
			Write(right);
		};
		operator const float() {
			return Read();
		};
		operator const int() {
			return round(Read());
		};
};


#endif
