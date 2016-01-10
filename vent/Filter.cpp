#include "Filter.h"

void sort_float(float a[], int size) {
    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    float t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}

F5_float::F5_float() {
	cnt = 0;
}

float F5_float::Read() {
	if (cnt == 0) return NAN;  //пусто в фильтре
	//Если cnt < 5 пишем в массив значение 
	
	if (cnt < 5) {
		val[4] = 0;
		for(int i = 0; i < cnt; i++)
				val[4] += val[i];
		return val[4]/cnt;
	} else {
		//Копируем значения в буффер
		memcpy(&buf, &val, sizeof(buf));
		//сортируем массив
		sort_float(buf, 5);		
		//Усредняем значение по трем средним (крайние - нахер)
		return (buf[1]+buf[2]+buf[3])/3;
	}	
}

void F5_float::Write(float p_val) {
	//ошибки в буфер не пишем
	if(isnan(p_val)) return;
	//Если cnt < 5 пишем в массив значение 
	if (cnt < 5) {
		val[cnt] = p_val;
		cnt++;
	} else {
		//продвигаем значения вперед
		val[0] = val[1];
		val[1] = val[2];
		val[2] = val[3];
		val[3] = val[4];
		//в конец пишем полученное
		val[4] = p_val;		
	}	
}