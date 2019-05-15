#include <SoftwareServo.h>
#include <Wire.h>
#include <Time.h>
#include <RTClib.h>
/*#include <LiquidCrystal.h>*/
#include <I2C_eeprom.h>
#include <stdio.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <TimerThree.h>

#include <DHT.h>

#include "vent.h"
#include "heater.h"
#include "Filter.h"
//------------------------------------------------------------

//#define DEBUG 0

struct LED_MODE {
	bool blink;
	uint16_t t1, t2, t3, t4; // время нарастания, свечения, затухания, пауза
	uint8_t r, g, b;
	char name[12];
};

#define MODE_ERROR 0
#define MODE_STOP 1 
#define MODE_NORMAL 2
#define MODE_COOLING_FAN 3
#define MODE_COOLING_AC 4
#define MODE_HEATING 5
#define MODE_HEATING_ADD 6
#define MODE_FORCE_MAN 7
#define MODE_FORCE_Rh 8
#define MODE_FORCE_Aq 9
#define MODE_FORCE_Mv 10
#define MODE_FORCE_EXH 11

uint8_t MODE = 0;

LED_MODE 		 lm[MODE_FORCE_EXH+1] = {{1, 100,100,100,100, 255, 0, 0, "ERROR"}, //MODE_ERROR, красный часто мигает
																		 {1, 1000,1000,1000,1000, 255, 0, 0, "STOP"}, //MODE_STOP, красный редко мигает
																		 {1, 100,2000,100,100,0, 255, 0, "NORMAL"}, //MODE_NORMAL, зеленый
																		 {1, 100,2000,100,100,0, 0, 255, "COOLING_FAN"}, //MODE_COOLING_FAN, синий
																		 {1, 500,500,500,500, 0, 0, 255, "COOLING_AC"}, //MODE_COOLING_AC, синий мигает
																		 {1, 100,2000,100,100,255, 0, 0, "HEATING"}, //MODE_HEATING, красный
																		 {1, 500,500,500,500, 255, 0, 0, "HEATING_ADD"}, //MODE_HEATING_ADD, красный, мигает
																		 {1, 500,500,500,500, 0, 255, 0, "FORCE_MAN"}, //MODE_FORCE_MAN, зеленый мигает
																		 {1, 500,500,500,500, 210, 150, 0, "FORCE_Rh"}, //MODE_FORCE_Rh, желто-зеленый мигает
																		 {1, 500,500,500,500, 0, 255, 255, "FORCE_Aq"}, //MODE_FORCE_AQ, сине-зеленый мигает
																		 {1, 500,500,500,500, 255, 0, 255, "FORCE_Mv"}, //MODE_FORCE_Mv, фиолетовый мигает
																		 {1, 500,500,500,500, 255, 255, 0, "FORCE_EXH"}, //MODE_FORCE_EXH, желтый мигает
																		};
																		
//------------------------------------------------------------
//память EEPROM
#define I2C_EEPROM_EXTENDED 1
#define EEPROM_ADR 0x50  // Address with three address pins grounded. 
I2C_eeprom Eeprom(EEPROM_ADR);

//------------------------------
/* адресная карта 
00 				пусто
01-24 		ADR_VENT_1 = 1
25-48 		ADR_VENT_2 = 25
49-113 		адреса датчиков 1w (8*8)
114-115 	ADR_AQ_MIN 2 байта (исп 1)
116-116 	ADR_AQ_PREHEAT_TIME 1 байт
*/

// ---------------------------------
//адрес скорость вентилятора 1 - комната 24 байта
const uint16_t ADR_VENT_1 = 1; 
//адрес скорость вентилятора 2 - спальни 24 байта
const uint16_t ADR_VENT_2 = 25;
//адреса термометров
const uint16_t ADR_1W_DEV = 49; // 48 байт (6 блоков по 8 байт)
//Датчик кач-ва воздуха
//Адрес макс качества
const uint16_t ADR_AQ_MIN = 114; //(храним int16)
const uint16_t ADR_AQ_PREHEAT_TIME = 116; //(храним int8)

const uint16_t ADR_T = 117; //(храним int8)
const uint16_t ADR_dT = 118; //(храним int8)

const uint16_t ADR_MOVE_MAX = 119; // порог для датчика движения (храним int8) 

const uint16_t ADR_dRh = 120; // порог для различия по влажности

// ---------------------------------
#define POOL_INT  300000	//интервал опроса датчиков
#define REG_INT   600000	//интервал регулирования

//Пины
#define ONE_WIRE_BUS 70
#define DHT_INT_1 71 //Room3_DHT(DHT_INT_1, 	DHT22, "Room3"), 		//Датчик в комнате
#define	DHT_INT_2 73 //Exhaust_DHT(DHT_INT_2, DHT22, "Exhaust"), 	//Датчик в ванной, в вентканале
#define	DHT_EXT 72   //External_DHT(DHT_EXT, 	DHT22, "External"); //Датчик на улице

#define	AV_EN	84 //Включение питания заслонок
#define	AV_1 79
#define	AV_2 80
#define	AV_3 81
#define	IR_Tx 75
#define	IR_Rx 76
#define	EXH_SENS 77
#define	MOVE_SENS 78
	
//#define	AC_CUR_SENS 
	
#define	AQ_IN 62
	
#define	BUTTON 82
#define	LED_R 44
#define	LED_G 45
#define	LED_B 46

#define	AQ_EN	83 //Включение датчика кач-ва воздуха


#define	REL_A_1	22 //Вент спален S=5
#define	REL_A_2	23 //Вент спален S=4 
#define	REL_A_3	24 //Вент спален S=3
#define	REL_A_4	25 //Вент спален S=2
#define	REL_A_5	26 //Вент спален S=1
#define	REL_A_6	27 
#define	REL_A_7	28 
#define	REL_A_8	29 

//Батарея спальня детская H1	
//Батарея спальня взрослая H2
//Батарея комната H3

#define	REL_B_1	30 //Вент комн S=5
#define	REL_B_2	31 //Вент комн S=4
#define	REL_B_3	32 //Вент комн S=3
#define	REL_B_4	33 //Вент комн S=2
#define	REL_B_5	34 //Вент комн S=1
#define	REL_B_6	35 
#define	REL_B_7	36 
#define	REL_B_8	37 

char get_obj[] 		= "GET /objects/?object=";
char temp_obj[] 	= "_TS&op=m&m=tempChanged&t=";
char status_obj[] = "_kS&op=m&m=statusChanged&status=";
char state_obj[] = "&op=m&m=stateChanged&state=";
// ---------------------------------
// Данные для вентиляторов

Vent Vent1 (REL_B_5,REL_B_4,REL_B_3,REL_B_2, REL_B_1, AV_1, AV_EN, ADR_VENT_1, &Eeprom); // Вентилятор комнаты
Vent Vent2 (REL_A_5,REL_A_4,REL_A_3,REL_A_2, REL_A_1, AV_2, AV_EN, ADR_VENT_2, &Eeprom); // Вентилятор спальни

//------------------------------------------------------------------
// Термодатчики
// Объекты для хранения найденных датчиков, ограничимся 10шт
#define MAX_1W_DEV_COUNT 10

DeviceAddress Searching1Wire[MAX_1W_DEV_COUNT];
int numberOfDevices;
DeviceAddress tempDeviceAddress;


#define TEMPERATURE_PRECISION 11
#define TEMP_UNDEF -255

// Объекты для 1-Wire термометров
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

//Печать адреса в hex
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++)  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

class TempSens {
	public:
		DeviceAddress adr;	// Адрес датчика на шине
		DallasTemperature *sensors;
		uint16_t eeprom_adr;	//адрес для сохранения адреса на шине
		int8_t index; 			//Номер устройства на шине
		F5_float temp;					// Текущая температура
		char name[10];			// Имя датчика
		char obj[16];				// Имя объекта
		I2C_eeprom *eeprom;
	//Функции
		TempSens(const uint16_t p_epprom_adr, DallasTemperature *p_sensors, char *p_name, I2C_eeprom *p_eeprom) {
			memcpy(name, p_name, 10); 
			index = -1;
			eeprom_adr = p_epprom_adr;
			sensors = p_sensors;
			temp = NAN;
			eeprom = p_eeprom;
		};
		void begin(void) {
			eeprom->readBlock(eeprom_adr, adr, 8);
			temp = NAN;
		};
		bool Link(uint8_t p_index, DeviceAddress &p_adr, bool printSerial = false) {
			if(p_adr)
			if(index>-1) {// датчик занят
				if(printSerial) {Serial.print(F("ERR - 1-Wire - device ")); Serial.print(name); Serial.print(F(" is already associated to index \'")); Serial.print(index, DEC); Serial.println("\'");}
				return false;
			}
			else {
				memcpy(adr, p_adr, sizeof(adr));
				index = p_index;
				temp = NAN;
				for(int i=0; i<8; i++) eeprom->writeByte(eeprom_adr+i, adr[i]);
				if(printSerial) {Serial.print(F("Associate device ")); printAddress(adr); Serial.print(" to \'"); Serial.print(name); Serial.println("\'");}
				return true;				
			}				
		};
		bool SetIndex(uint8_t p_index, DeviceAddress &p_adr, bool printSerial = false) {
			if(index>-1) {// датчик занят
				if(printSerial) {Serial.print(F("ERR - 1-Wire - device ")); Serial.print(name); Serial.print(F(" is already associated to index \'")); Serial.print(index, DEC); Serial.println("\'");}
				return false;
			}
			else if (memcmp(adr, p_adr, sizeof(p_adr))==0) {
							memcpy(adr, p_adr, sizeof(adr));
							index = p_index;
							temp = NAN;
							return true;
						}
					 else return false;							
		};
		void UnLink(bool printSerial = false){
			if(printSerial) {Serial.print(F("Clearing associate device ")); printAddress(adr); Serial.print(F(" to \'")); ; Serial.print(name); Serial.println("\'");}
			memset(adr, 0, sizeof(adr));
			index = -1;
			temp = NAN;
			for(int i=0; i<8; i++) eeprom->writeByte(eeprom_adr+i, 0);
		};
		float GetTemp(){
			if (index > -1) 
				temp = sensors->getTempC(adr);
			else temp = NAN;
			return temp;
		};
		void PrintToSerial(void) { 
			if (index == -1) {
				Serial.print(F("Sensor ")); Serial.print(name); Serial.print(F(" is not assigned")); Serial.println();
			}
			else {
				Serial.print(F("INDEX="));  Serial.print(index, DEC); 
				Serial.print(F("\t ADR="));  	printAddress(adr);				
				Serial.print(F("\t Temperature=")); Serial.print(float(temp));	Serial.print(F(" *C")); Serial.println();
			} 
		};
		void PrintToSerialWEB(void) {
			Serial.print(get_obj);	Serial.print(name); Serial.print(temp_obj); Serial.println(float(temp), 1); 				
		};
};

#define TS_COUNT 5

TempSens 									
	Room1_DST				(ADR_1W_DEV+ 0, &sensors, "Room1", 			&Eeprom),			// 1 Детская
	Room2_DST				(ADR_1W_DEV+ 8, &sensors, "Room2", 			&Eeprom),			// 2 Спальня родительская
	//Kitchen_DST			(ADR_1W_DEV+16, &sensors, "Kitchen", 		&Eeprom),			// 3 Над плитой, кухня
	Aqua_DST				(ADR_1W_DEV+24, &sensors, "Aqua", 			&Eeprom),			// 6 Аквариум
	HeatTransfer_DST(ADR_1W_DEV+32, &sensors, "HeatTransf", &Eeprom), 		// 7 Теплоноситель в трубе (центр. отопление)
	Freon_DST				(ADR_1W_DEV+40, &sensors, "Freon", 			&Eeprom);			// 8 Фреон газ (толстая трубка) горячая - греет, холодная - холодит)
						
//Массив указателей на класс (что-бы удобно всех циклом окучивать)						
TempSens* TS[TS_COUNT] = {&Room1_DST, &Room2_DST, /*&Kitchen_DST,*/ &Aqua_DST, &HeatTransfer_DST, &Freon_DST};
									
Heater H1(REL_A_6, "Heat_1"), H2(REL_A_7, "Heat_2"), H3(REL_A_8, "Heat_3");							
//------------------------------------------------------------
// Переменные пуллинга					
unsigned long curMillis;
unsigned long btnTime = 0;
unsigned long prevMillis1 = 0;
unsigned long prevMillis2 = 0;
unsigned long prevMillis3 = 0;
unsigned long prevMillis4 = 0;

//------------------------------------------------------------
// Часы РВ (RTC)
RTC_DS1307 RTC;
DateTime tmpDateTime;

//------------------------------------------------------------
// Переменные для различных целей
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete 

float tmpf, T = 24, dT = 5;

uint8_t isCommand, isConfig_1W, isConfig_Vent, isConfig_EEPROM, isConfig_DHT, NeedAddHeat, isEnable, MoveMax, dRh;

uint16_t forceCounter, exhCounter,  moveCounter;

//--------------------------------------------------------------
// Датчик кач воздуха
struct AQ_SENS{
	uint16_t MinValue, CurrentHeatingTime;
	F5_float Value;
	uint8_t PreheatTime;
};

AQ_SENS AQ;

//-------------------------------------------------------------
// Датчики влажность/температура (AM2302/DHT22)
class DHT_my : public DHT {
	public: 
		char name[10];
		F5_float temp, rh;
		DHT_my(uint8_t pin, uint8_t type, char * p_name) : DHT(pin, type)
			{ memcpy(name, p_name, 10); };
		bool ReadSensor(void){
			rh = readHumidity();
			//Врет минимум на градус вверх, подкорректируем чуток
			temp = readTemperature()-1;
			if(isnan(temp) || isnan(rh)) return false;
			else return true;
		};
		void PrintToSerial(void) { 
			if (isnan(temp) || isnan(rh)) {
				Serial.print(F("No data from sensor - ")); Serial.print(name); Serial.println();
			} else {
				Serial.print(F("Name: ")); Serial.print(name);  Serial.print("\t");
				Serial.print(F("Humidity: "));  	 Serial.print((float)rh);
				Serial.print(F(" %\t"));
				Serial.print(F("Temperature: ")); Serial.print((float)temp);	Serial.print(F(" *C")); Serial.println();
			} 
		};
		void PrintToSerialWEB(void) {
			if (!isnan(temp) && !isnan(rh)) {
				Serial.print(get_obj);	Serial.print(name); Serial.print(temp_obj); Serial.println((float)temp, 1); 	
				Serial.print(get_obj);	Serial.print(name); Serial.print("Hum"); Serial.print(temp_obj); Serial.println((int)rh); 	
			}
		};
};

DHT_my 	Room3_DHT(DHT_INT_1, 		DHT22, "Room3"), 		//Датчик в комнате
				Exhaust_DHT(DHT_INT_2, 	DHT22, "Exhaust"), 	//Датчик в ванной, в вентканале
				External_DHT(DHT_EXT, 	DHT22, "External"); //Датчик на улице

void Print1WDev(){
	//Сеть 1-wire
  Serial.print(F("Found "));
  Serial.print(numberOfDevices, DEC);
  Serial.println(F(" 1-wire devices."));

  if (!numberOfDevices) return;
  
  Serial.print(F("Parasite power is: ")); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF"); 

  Serial.print(F("Requesting temperatures..."));
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE"); 

  Serial.println(F("ID Adress           Temp  Name"));
  // Loop through each device, print out address
  for(int i = 0; i < numberOfDevices && i < MAX_1W_DEV_COUNT; i++) {
    Serial.print(i, DEC);
    Serial.print("  ");
    printAddress(Searching1Wire[i]);
    Serial.print(" ");
		tmpf=sensors.getTempC(Searching1Wire[i]);
		
		if (tmpf == DEVICE_DISCONNECTED) { 
			Serial.print("ERR"); 
		}	else { 
			Serial.print(tmpf);
				for (int j = 0; j < TS_COUNT; j++) {
					if (memcmp(TS[j]->adr, Searching1Wire[i], sizeof(tempDeviceAddress))==0) { 
						Serial.print(" "); Serial.print(TS[j]->name); 
					}
				}
			}
    Serial.println();
    }
}

//Функция прописывает температуры в структуры и сбрасывает флаг конфигурации если не все датчики нашлись
// если датчики найдены - 
bool Refresh1WData(){
	sensors.requestTemperatures(); // Send the command to get temperatures
  // Loop through each device, print out address
	isConfig_1W = true;
  for(int i = 0; i < TS_COUNT; i++) {
		//Вытащим температуру по адресу
		tmpf = TS[i]->GetTemp();
		if ( isnan(tmpf)) { 
			isConfig_1W = false; 
			Serial.print(F("ERR - 1-Wire - device disconected. Sensor name - "));	Serial.print(TS[i]->name); Serial.println();
		}
	}
	return isConfig_1W;
}

int Search1W(){
  //Поищем 1-wire термометры
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();  	
	// Loop through each device
  for(int i=0; i<numberOfDevices && i<MAX_1W_DEV_COUNT; i++)  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {
			memcpy(Searching1Wire[i], tempDeviceAddress, sizeof(tempDeviceAddress));
			sensors.setResolution(Searching1Wire[i], TEMPERATURE_PRECISION);
			for (int j = 0; j < TS_COUNT; j++) {
				if (memcmp(TS[j]->adr, tempDeviceAddress, sizeof(tempDeviceAddress))==0) { 
					TS[j]->SetIndex(i, tempDeviceAddress);
					}
			}
		} else{
			Serial.println();
			Serial.print(F("ERR - 1-wire - found ghost device at "));
			Serial.print(i, DEC);
			Serial.print(F(" but could not detect address. Check power and cabling"));
			Serial.println();
		}
  } 
	return numberOfDevices;
}

//Печать параметров в сериал с разделителем
void PrintIntParamString(const uint8_t *buf, const int len, const char delim) {
  char tmp_c[8];
  for (int i = 0; i < len; i++) {
		itoa(buf[i], tmp_c, 10);
    Serial.print(tmp_c); 
    if(i<len-1) Serial.print(delim);          
    }
}

// Вывод параметров через гейт на веб-сервер
void StatusToWEB(){
	Serial.println();
	
	//Статус 
	Serial.print(get_obj); Serial.print(F("EnableVH")); Serial.print(status_obj); Serial.println(isEnable); 	
	
	// Сбросим  настройку текущей температуры
	Serial.print(get_obj); Serial.print(F("RoomTemp_P&op=m&m=valueChanged&value=")); Serial.println(T, 1);
	Serial.print(get_obj); Serial.print(F("RoomDTemp_P&op=m&m=valueChanged&value=")); Serial.println(dT, 1);
	
	// Сбросим датчики температуры на сервер
	for (int i = 0; i < TS_COUNT; i++) TS[i]->PrintToSerialWEB();
	
	// Сбросим  датчик качества воздуха на сервер
	//Serial.print(get_obj); Serial.print(F("AirQuality")); Serial.print(temp_obj); Serial.println((int)AQ.Value);
				
	// Сбросим датчики DHT 
	External_DHT.PrintToSerialWEB(); 	
	Room3_DHT.PrintToSerialWEB();
	Exhaust_DHT.PrintToSerialWEB();
	
	// Сбросим  индикатор недостатка тепла
	Serial.print(get_obj); Serial.print(F("NeedAddHeat")); Serial.print(status_obj); Serial.println(NeedAddHeat); 
	
	// Сбросим статус клапанов батарей
	H1.PrintToSerialWEB(); 
	H2.PrintToSerialWEB(); 
	H3.PrintToSerialWEB(); 
	
	// Сбросим скорости вентиляторов
	Serial.print(get_obj); Serial.print(F("Vent_1")); Serial.print(temp_obj); Serial.println(Vent1.Speed, DEC); 	
	Serial.print(get_obj); Serial.print(F("Vent_2")); Serial.print(temp_obj); Serial.println(Vent2.Speed, DEC); 			
	
	//Сбросим счетчики
	Serial.print(get_obj); Serial.print(F("MoveCnt")); Serial.print(temp_obj); Serial.println(round(moveCounter*100/(REG_INT/1000))); 	
	Serial.print(get_obj); Serial.print(F("ForceCnt")); Serial.print(temp_obj); Serial.println(round(forceCounter/60)); 
	Serial.print(get_obj); Serial.print(F("ExhaustCnt")); Serial.print(temp_obj); Serial.println(round(exhCounter/60)); 
	
	//Сбросим режим работы
	Serial.print(get_obj); Serial.print(F("VH_Control")); Serial.print(state_obj); Serial.println(lm[MODE].name);
	
}

void callback_t3() { 
	static float k;
	if (lm[MODE].blink) {
		//Если цикл режима светодиода кончился - ставим цикл в начало
		if(curMillis - prevMillis4 > lm[MODE].t1+lm[MODE].t2+lm[MODE].t3+lm[MODE].t4)
			prevMillis4 = curMillis;
		if((prevMillis4 < curMillis) && (curMillis <= (prevMillis4+lm[MODE].t1))){ //фаза нарастания яркости
			k = ((float)(curMillis-prevMillis4))/((float)lm[MODE].t1);
			setRGB(lm[MODE].r*k, lm[MODE].g*k, lm[MODE].b*k);
		} else {
			if(prevMillis4+lm[MODE].t1 < curMillis && curMillis <= prevMillis4+lm[MODE].t1+lm[MODE].t2){ //фаза удержания яркости
				setRGB(lm[MODE].r, lm[MODE].g, lm[MODE].b);
			}	else {
				if(prevMillis4+lm[MODE].t1+lm[MODE].t2 < curMillis && curMillis <= prevMillis4+lm[MODE].t1+lm[MODE].t2+lm[MODE].t3){ //фаза уменьшения яркости
					k = 1-((float)(curMillis-prevMillis4-lm[MODE].t1-lm[MODE].t2))/(float)lm[MODE].t3;
					setRGB(lm[MODE].r*k, lm[MODE].g*k, lm[MODE].b*k);
				}	else {
					if(prevMillis4+lm[MODE].t1+lm[MODE].t2+lm[MODE].t3 < curMillis && curMillis <= prevMillis4+lm[MODE].t1+lm[MODE].t2+lm[MODE].t3+lm[MODE].t4){ //фаза выключения
						setRGB(0, 0, 0);
					}
				}
			}
		}
	}
	else
		setRGB(lm[MODE].r, lm[MODE].g, lm[MODE].b);		 
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
		// Пришлось придумать символ-заменитель переводу строки - в кач-ве параметра в cmd \n передать не удалось
    if (inChar == '\n' || inChar == '@') {
      inputString.replace('@', '\n');
			stringComplete = true;
    } 
  }
} 

void StatusToSerial(){	
	Serial.println(F("------------System status------------"));
	//EEPROM
	if (isConfig_EEPROM){
		Serial.print(F("EEPROM is OK. Size is "));
		Serial.print(isConfig_EEPROM, DEC);
		Serial.println("k.");
	} else
		Serial.print(F("ERR - EEPROM. Unable to determine size."));
	//Время
  if (RTC.isrunning()){ 
		// Определяем время
    tmpDateTime = RTC.now();
    // Выводим время в монитор порта
    Serial.print(F("TIME: "));
    Serial.print(tmpDateTime.year(), DEC);
    Serial.print('/');
    Serial.print(tmpDateTime.month(), DEC);
    Serial.print('/');
    Serial.print(tmpDateTime.day(), DEC);
    Serial.print(' ');
    Serial.print(tmpDateTime.hour(), DEC);
    Serial.print(':');
    Serial.print(tmpDateTime.minute(), DEC);
    Serial.print(':');
    Serial.print(tmpDateTime.second(), DEC);
    Serial.println();
    } else
			Serial.println(F("ERR - TIME - RTC is not runing. Check battery, set real date/time."));
	  
  Print1WDev();
		
	//Параметры скорости вентиляторов по времени
  unsigned char i;
   
  Serial.print(F("Vent 1")); Serial.println();
	Serial.print(F("\tSpeed profile=")); PrintIntParamString(Vent1.MaxSpeedByHour, 24, '.'); Serial.println();
  Serial.print(F("\tSpeed (current/maximum) =")); Serial.print(Vent1.Speed, DEC); Serial.print('/'); Serial.print(Vent1.MaxSpeed, DEC); Serial.println();
	
  Serial.print(F("Vent 2")); Serial.println();
	Serial.print(F("\tSpeed profile=")); PrintIntParamString(Vent2.MaxSpeedByHour, 24, '.'); Serial.println();
  Serial.print(F("\tSpeed (current/maximum) =")); Serial.print(Vent2.Speed, DEC); Serial.print('/'); Serial.print(Vent2.MaxSpeed, DEC); Serial.println();   
	
	//Датчик качества воздуха
	//Serial.print(F("Air quality sensor: minimum value, (aq) = ")); Serial.print(AQ.MinValue, DEC); Serial.print(F("%, heat-up time ")); Serial.print(AQ.PreheatTime, DEC); Serial.println(F("sec.")); 
	
	//Климат
	Serial.print(F("Target temperature = ")); Serial.print(T, 2); Serial.print(F("'C, accuracy ")); Serial.print(dT, 0); Serial.print(F("/10 'C.")); Serial.println();
	
	Serial.print(F("Move sensitivity (mv) = ")); Serial.print(MoveMax); Serial.println();
	Serial.print(F("Current movement value = ")); Serial.print(round(moveCounter*100/(REG_INT/1000))); Serial.println();
	
	Serial.print(F("Maximum humidity difference (dh) = ")); Serial.print(dRh); Serial.print("%"); Serial.println();

	Serial.print(F("Current humidity abs. difference value = ")); Serial.print(abs((int)Room3_DHT.rh-(int)Exhaust_DHT.rh)); Serial.print("%"); Serial.println();

	// Сбросим датчики температуры на сервер
	for (int i = 0; i < TS_COUNT; i++) TS[i]->PrintToSerial();
	
	//Датчики влажности/температуры
	External_DHT.ReadSensor();
	External_DHT.PrintToSerial(); 	
	Room3_DHT.ReadSensor();
	Room3_DHT.PrintToSerial();
	Exhaust_DHT.ReadSensor();
	Exhaust_DHT.PrintToSerial();
		
	Serial.print(F("System status - "));	Serial.println(lm[MODE].name);
	if (MODE == MODE_ERROR) {
		Serial.println(F("Config flags:\t"));
		Serial.print(F("1W=")); Serial.print(isConfig_1W); Serial.print("\t");
		Serial.print(F("Vent=")); Serial.print(isConfig_Vent); Serial.print("\t");
		Serial.print(F("EEPROM=")); Serial.print(isConfig_EEPROM); Serial.print("\t");
		Serial.print(F("DHT=")); Serial.print(isConfig_DHT); Serial.print("\t");
		Serial.println();
	}	
	
	Serial.println(F("Counters:\t"));
	Serial.print(F("move=")); Serial.print(moveCounter); Serial.print("\t");
	Serial.print(F("force=")); Serial.print(forceCounter); Serial.print("\t");
	Serial.print(F("exh=")); Serial.print(exhCounter);
	Serial.println();
	
}


SoftwareServo aqua_fan;

void setup() {

	uint8_t tmpi;
	
  Serial.begin(57600);
		
	// светодиод 
  pinMode(13, OUTPUT);
  
	Timer3.initialize(10000); //делаем таймер на 10 миллисекунд 
  Timer3.attachInterrupt(callback_t3);
	
	MODE = MODE_STOP;
	  
  // reserve 128 bytes for the inputString:
  inputString.reserve(128); 
  	
  //инициализация I2C шины (мега - 20/21 uno 4/5)
  Wire.begin();
	
  //Инициализация часов
  RTC.begin();
		
	isConfig_EEPROM = Eeprom.determineSize();

	isConfig_Vent = Vent1.ReadEepromParams() & Vent2.ReadEepromParams();	
	
	pinMode(AV_EN, OUTPUT);	
	digitalWrite(AV_EN, LOW);
	
	/*Vent1.TestValve();
	Vent2.TestValve();*/
	
	Room1_DST.begin();
	Room2_DST.begin();
	//Kitchen_DST.begin();
	Aqua_DST.begin();
	HeatTransfer_DST.begin();
	Freon_DST.begin();
	
	//Найдем все термометры
	Search1W();
		
	//Настройки датчика кач-ва воздуха
	Eeprom.readBlock(ADR_AQ_MIN, (uint8_t*)&(AQ.MinValue), 1);
	Eeprom.readBlock(ADR_AQ_PREHEAT_TIME, &AQ.PreheatTime, 1);
	pinMode(AQ_EN, OUTPUT);
	digitalWrite(AQ_EN, LOW);
	
	//Вычитываем настройки климата
	Eeprom.readBlock(ADR_T, &tmpi, 1);
	T = tmpi;
	Eeprom.readBlock(ADR_dT, &tmpi, 1);
	dT = tmpi;
	
	Eeprom.readBlock(ADR_MOVE_MAX, (uint8_t*)&(MoveMax), 1); 
	Eeprom.readBlock(ADR_dRh, (uint8_t*)&(dRh), 1); 
	
	//Настроим порт вытяжки
	pinMode(EXH_SENS, INPUT);
	
	//Настроим порт датчика движения
	pinMode(MOVE_SENS, INPUT);
	
	Room3_DHT.begin(); 
	Exhaust_DHT.begin(); 
	External_DHT.begin(); 
	isConfig_DHT = false;
	
	pinMode(AV_EN, OUTPUT);
	digitalWrite(AV_EN, LOW);
	pinMode(REL_A_3, OUTPUT);
	digitalWrite(REL_A_3, LOW);	
	pinMode(REL_A_4, OUTPUT);
	digitalWrite(REL_A_4, LOW);
	pinMode(REL_A_5, OUTPUT);
	digitalWrite(REL_A_5, LOW);	
	pinMode(REL_A_6, OUTPUT);
	digitalWrite(REL_A_6, LOW);
	pinMode(REL_A_7, OUTPUT);
	digitalWrite(REL_A_7, LOW);	
	pinMode(REL_A_8, OUTPUT);
	digitalWrite(REL_A_8, LOW);
	
	
	isEnable = 1;
		
	moveCounter = 0;
	forceCounter = 0;
	exhCounter = 0;
	
	//Обновим температуры 
	Refresh1WData();
	//Снимем влажность в комнате и вытяжке (250msec/call)
	//Вытяжка
	if(!Room3_DHT.ReadSensor() || !Exhaust_DHT.ReadSensor()	|| !External_DHT.ReadSensor()) isConfig_DHT = false;
	else isConfig_DHT = true;
		
	//отчитаемся о инициализации
  StatusToSerial();
		
	aqua_fan.attach(AV_3);	
	
	digitalWrite(AV_EN, HIGH);	
	delay(50);
  aqua_fan.write(13); 
	SoftwareServo::refresh();	delay(50);
	SoftwareServo::refresh();	delay(50);
	SoftwareServo::refresh();	delay(50);
	SoftwareServo::refresh();	delay(50);
	digitalWrite(AV_EN, LOW);
	Serial.println("Aqua fan closed (init)");	
}

int CalcVentSpeed(const float p_temp){
	static float Tmin, Tmed, Tmax, k, b, Savg, Smax, Smin, S;
	Tmin=-15;
	Tmed=p_temp-4;
	Tmax=30;
	Smax = VENT_MAX_SPEED;
	Smin = VENT_MIN_SPEED;
	
	
	//Если еще не совсем дубак - проветриваем пока не похолодает до 20
	if (p_temp > 20 && forceCounter > 0) {
		MODE = MODE_FORCE_MAN;	
		Serial.println("MODE_FORCE_MAN, VENT_MAX_SPEED");
		return VENT_MAX_SPEED;
	}
	//Вытяжка работает пока до 22х не опустится температура 
	if (p_temp > 22 && exhCounter > 0) {
		MODE = MODE_FORCE_EXH;	
		Serial.println("MODE_FORCE_EXH, VENT_MAX_SPEED");
		return VENT_MAX_SPEED;
	}
	//Вытяжка работает пока до 21х не опустится температура на чуть меньшей скорости
	if (p_temp > 21 && exhCounter > 0) {
		MODE = MODE_FORCE_EXH;	
		Serial.println("MODE_FORCE_EXH, VENT_MAX_SPEED-1");
		return VENT_MAX_SPEED-1;
	}
	//Вытяжка работает пока до 21х не опустится температура на еще чуть меньшей скорости
	if (p_temp > 20 && exhCounter > 0) {
		MODE = MODE_FORCE_EXH;	
		Serial.println("MODE_FORCE_EXH, VENT_MAX_SPEED-2");		
		return VENT_MAX_SPEED-2;
	}
	
	//На улице холоднее чем дома
	if ((int)External_DHT.temp <= p_temp) {
		//Рассчет средней скорости для температур
		
		//Если сильно холодно (меньше -15) скорость минимальная=1
		if ( (int)External_DHT.temp <= Tmin ) {
			Savg = 1;
		}
		//Если тепло - скорость рассчитаем в зависимости от внешней температуры
		else {
			//Если на улице теплее чем у нас в среднеминимальной отметке
			if ( (int)External_DHT.temp >= Tmed) {
				//Скорость на всю
				Savg = Smax;
			}
			else {
				Savg = ((Smax-1)/(Tmed-Tmin)*((float)External_DHT.temp)+1-((Smax-1)/(Tmed-Tmin))*Tmin);
			}
		}
		
		//Рассчитаем коэффициент к
		k = (Savg-Smin)/(2*dT/10);
		//Рассчитаем коэффициент и
		b = Smin-k*(T-2*dT/10);
		
	}
	//На улице теплее
	else {
		//Если сильно тепло скорость минимальная=1
		if ( (int)External_DHT.temp > Tmax ) {
			Savg = 1;
		}
		//Если тепло не сильно - скорость рассчитаем в зависимости от внешней температуры
		else {
			Savg = ((1-Smax)/(Tmax-Tmed)*((float)External_DHT.temp)+1-((1-Smax)/(Tmax-Tmed))*Tmax);
		}
		
		//Рассчитаем коэффициент к
		k = (Savg-Smin)/(-2*dT/10);
		//Рассчитаем коэффициент и
		b = Smin-k*(T+2*dT/10);
	}
	
	S = k*p_temp+b;
	
	MODE = MODE_NORMAL;
	
	//Скорость
	Serial.println("---------------------------------------------------");
	Serial.print(F("\tTout=")); Serial.print((float)External_DHT.temp, DEC); 
	Serial.print(F("\tTin=")); Serial.print(p_temp, DEC); 
	Serial.print(F("\tSavg=")); Serial.print(Savg, DEC); 
	Serial.print(F("\tk=")); Serial.print(k, DEC); 
	Serial.print(F("\tb=")); Serial.print(b, DEC); 
	Serial.print(F("\tS=")); Serial.print(S, DEC); 
	Serial.print(F("\tround(S)=")); Serial.println(round(S), DEC);
	Serial.println("---------------------------------------------------");
	return round(S);
	/*
	//Если минимальная температура в норме
	if (T-dT/10 <= p_temp && p_temp <= T+dT/10) {
		//Если есть движение или кач-во воздуха упало ниже нормы или влажность удаляемого воздуха сильно отличается от комнатной - вентиляцию на максимум
		if(1==0 
		//abs((int)Room3_DHT.rh-(int)Exhaust_DHT.rh) > dRh
		) {
			MODE = MODE_FORCE_Rh;	
			return VENT_MAX_SPEED;
		} else 	if(1==0
		//(int)AQ.Value < AQ.MinValue
		) {
							MODE = MODE_FORCE_Aq;	
							return VENT_MAX_SPEED;
						} else 	if(moveCounter >= round((REG_INT/1000)*(100-MoveMax)/100) ) { //если накопленное значение превышает относительный порог
											MODE = MODE_FORCE_Mv;	
											return VENT_MAX_SPEED;
										} else {
			MODE = MODE_NORMAL;
			
			if (   (p_temp-T > 0 && (float)External_DHT.temp <= T) // теплее чем надо и на улице холоднее - будем холодить улицей
					|| (p_temp-T < 0 && (float)External_DHT.temp > T ) // холоднее чем надо и на улице теплее - будем греть улицей
				 )
				return map((p_temp-T)*10/dT, 0, 1, 1, VENT_MAX_SPEED-1); //отразим относительную ошибку температуры на скорость от 1 до макс (при увеличении ошибки - увеличиваем скорость)
			if (   (p_temp-T < 0 && (float)External_DHT.temp <= T) // холоднее чем надо и на улице холоднее - не стоит холодить улицей
					|| (p_temp-T > 0 && (float)External_DHT.temp > T)  // теплее чем надо и на улице теплее - не стоит греть улицей
				 )
				return map((p_temp-T)*10/dT, 0, 1, VENT_MAX_SPEED-1, 1); //отразим относительную ошибку температуры от макс до 1 (при увеличении ошибки уменьшаем скорость)
		}
	}	else {
		//Холодно
		if (min((float)Room1_DST.temp, (float)Room2_DST.temp) < T-dT/10) {
			MODE = MODE_HEATING;
			//На улице очень холодно <-20 - очень сильно прижмем заслонку
			if ((int)External_DHT.temp < -20) 
				return -3;
			//На улице очень холодно <-15 - сильно прижмем заслонку
			if ((int)External_DHT.temp < -15) 
				return -2;
			//На улице холодно <-10 - прижмем заслонку
			if((int)External_DHT.temp < -10)
				return -1;
			if ((int)External_DHT.temp > 25)	
				return -1;
			return 1;
		} else { //Жарко
			//Можно охладиться улицей (там холоднее более чем на 5 градусов) или просто холоднее 13
			if ((int)External_DHT.temp < 13) {
				MODE = MODE_COOLING_FAN;
				return VENT_MAX_SPEED-1;
			} else {
				//спасет только кондей
				MODE = MODE_COOLING_AC;
				if (p_temp > (int)External_DHT.temp+5) {
					// Поможем кондею уличным притоком
					return map((int)External_DHT.temp, 13, p_temp, 1, VENT_MAX_SPEED-1); //отразим относительную ошибку температуры на скорость от 1 до макс-1 (при увеличении ошибки - увеличиваем скорость)
				}	else {
					// На улице слишком тепло - вентиляцию на минимум
					return 1;
				}
			}
		}
	}*/
}

void setRGB(uint8_t p_R, uint8_t  p_G, uint8_t p_B){
	//инвертнем ибо общий анод и 0 зажигает, а 255 гасит диод
	analogWrite(LED_R, 255 - p_R);
	analogWrite(LED_G, 255 - p_G);
	analogWrite(LED_B, 255 - p_B);
}

void loop() {
  
	int i, tmpi, tmpj, tmpk, btn, pos;
	float k;
	// put your main code here, to run repeatedly: 
  curMillis = millis();
	
	//digitalWrite(13, digitalRead(MOVE_SENS));
	
	//Проверим кнопку
	btn = !digitalRead(BUTTON);
	
	if (btn && btnTime == 0) {
		//Serial.print("fe=1 - ");
		delay(5); //защитимся от помех и дребезга пауза  и повтор чтения.
		btn = !digitalRead(BUTTON);
		if (btn && btnTime == 0) {
			//Serial.print("1");
			btnTime = curMillis;
		}
		Serial.println(" ");
	}
	
	//проверим на переполнение
	if(prevMillis1 > curMillis || prevMillis2 > curMillis || prevMillis3 > curMillis || prevMillis4 > curMillis){
		prevMillis1 = 0;
		prevMillis2 = 0;
		prevMillis3 = 0;
		prevMillis4 = 0;
	}
	
	//Ежесекундные действия
	if(curMillis - prevMillis3 > 1000) {
		
		//Если счетчик не насыщен (20 минут) - наращиваем
		if(digitalRead(MOVE_SENS)){
			moveCounter = constrain(moveCounter++, 0, round(REG_INT/1000));
		} 

		//Если счетчик не насыщен (5 минут) - наращиваем		
		if(!digitalRead(EXH_SENS)) {
			if(!exhCounter) // в первый раз зашли - форсируем регулировку
				prevMillis2 = curMillis + REG_INT + 1;
			exhCounter = constrain(exhCounter+1, 0, 60);
		} 
		else
			if(exhCounter) exhCounter--;

		if(forceCounter) forceCounter--;
		
		prevMillis3 = curMillis;
		
		//обработаем кнопку - считаем сколько ее держали
		if (btn) {
		  //таймер стоп
			Timer3.stop();
			//Нажатие - 10 минут проветриваем
			forceCounter = constrain(forceCounter+600, 0, 3600);
			setRGB(0, 0, 0);
			delay(100);
			//мигнули белым 
			setRGB(255, 255, 255);
			delay(100);
			setRGB(0, 0, 0);
			delay(200);
		}	else {
			if(btnTime>0){
				//отпустили батон
				btnTime = 0; 
				//таймер старт
				Timer3.start();
				//Форсирование цикла регулирования
				prevMillis2 = curMillis + REG_INT + 1;
			}
		}
	}
	
	// print the string when a newline arrives:
  if (stringComplete) { 
		inputString.toLowerCase();
    inputString.trim();
    isCommand = false;
    Serial.println(inputString);
    if (inputString.equalsIgnoreCase(F("help")) or inputString.equals("?")) {
			Serial.println(F("----------------Commands------------------"));
			Serial.println(F("all commands are case insensitive !"));
			Serial.println(F("status - print system information"));
			Serial.println(F("status-web - print web-proxy output"));
			Serial.println(F("enable/disable - enable/disable automatic control "));
			Serial.println(F("date=yyyy.mm.dd - set rtc date (date=2013.12.1)"));
			Serial.println(F("time=hh.mi.ss - set rtc time (time=13:54:52)"));
			Serial.println(F("sp#=####################### - set speed profile of ventilator. sp1 and sp2. (sp1=234444333333355555555522)"));
			Serial.println(F("search1w (search 1w, s1w) - search one-wire devices and display result"));			
			Serial.println(F("print1w (print 1w, p1w) - print one-wire devices data and addresses"));			
			Serial.println(F("set [t1, t2, ta, th, tc]=[0...10] - associate 1-wire device to T1-sensor"));
			Serial.println(F("clear [t1, t2, ta, th, tc] - unassociate 1-wire device to T1-sensor"));
			Serial.println(F("test vent - test vents"));
			//Serial.println(F("aq=## set minimum AirQuality value (if below - air quality is bad). Maximum value is 99"));
			//Serial.println(F("aq ht=### set time in second to heating-up AQ sensor before measuring. Maximum time - 255 sec"));
			Serial.println(F("t=## set target temperature in celsius"));
			Serial.println(F("dt=## set temperature accuracy in celsius/10"));
			Serial.println(F("dh=## set maximum humidity difference [1...99]"));
			Serial.println(F("mv=### set movement sensitivity value [1...100]"));
			isCommand = true;
		}
		if (inputString.equalsIgnoreCase(F("status"))) { StatusToSerial(); isCommand = true; }	
		if (inputString.equalsIgnoreCase(F("status-web"))) { StatusToWEB(); isCommand = true; }		
		if (inputString.equals(F("enable"))) { isEnable = 1; isCommand = true; }
		if (inputString.equals(F("disable"))) {	isEnable = 0;	isCommand = true;	}
		if (inputString.startsWith("t=")) {
			if ( inputString.substring(2, 4).toInt() < 15 || inputString.substring(2, 4).toInt() > 33) {
						Serial.println(F("Invalid target temperature value [15...33]")); 
      } else { 
				T = (inputString.substring(2, 4).toInt()); 
				Eeprom.writeByte(ADR_T, T);
			}
			isCommand = true;
		}
		if (inputString.startsWith("dt=")) {
			if ( inputString.substring(3, 6).toInt() < 5 || inputString.substring(3, 6).toInt() > 50) {
						Serial.println(F("Invalid temperature accuracy value [5...50]")); 
      } else { 
				dT = (inputString.substring(3, 6).toInt()); 
				Eeprom.writeByte(ADR_dT, dT);
			}
			isCommand = true;
		}
		if (inputString.startsWith("dh=")) {
			if ( inputString.substring(3, 6).toInt() < 5 || inputString.substring(3, 6).toInt() > 50) {
						Serial.println(F("Invalid maximum humidity difference value [1...99]")); 
      } else { 
				dRh = (inputString.substring(3, 6).toInt()); 
				Eeprom.writeByte(ADR_dRh, dRh);
			}
			isCommand = true;
		}
		if (inputString.startsWith("mv=")) {
			if ( inputString.substring(3, 6).toInt() < 1 || inputString.substring(3, 6).toInt() > 255) {
						Serial.println(F("Invalid movement sensitivity value [1...255]")); 
      } else { 
				MoveMax = (inputString.substring(3, 6).toInt()); 
				Eeprom.writeByte(ADR_MOVE_MAX, MoveMax);
			}
			isCommand = true;
		}
		/*if (inputString.startsWith("aq=")) {
			if ( inputString.substring(3, 6).toInt() < 0 || inputString.substring(3, 6).toInt() > 1023) {
						Serial.println(F("Invalid air quality value - valid value 0...1023")); 
      } else { 
				AQ.MinValue = (uint8_t)(inputString.substring(3, 6).toInt()); 
				Eeprom.writeByte(ADR_AQ_MIN, 	 (uint8_t)(AQ.MinValue));
			}
			isCommand = true;
		}
		if (inputString.startsWith(F("aq ht="))) {
			if ( inputString.substring(6, 9).toInt() < 0 || inputString.substring(6, 9).toInt() > 255) {
						Serial.print(F("Invalid value heating-up AQ sensor alid value 0...255")); 
						Serial.print(REG_INT, DEC); 
      } else { 
				AQ.PreheatTime = (uint8_t)(inputString.substring(6, 9).toInt()); 
				Eeprom.writeByte(ADR_AQ_PREHEAT_TIME, AQ.PreheatTime);
			}
			isCommand = true;
		}*/
		if (inputString.startsWith(F("test vent"))) {
			
			//Vent1.Test();
			//Vent2.Test();
			
			int pos;
			
			digitalWrite(AV_EN, HIGH);			
			
			delay(3000);
			
			for(pos = 15; pos < 120; pos ++)                                          
				{ aqua_fan.write(pos); SoftwareServo::refresh(); delay(50); }
			delay(2000);
			
			for(pos = 120; pos >15;  pos --)                                    
				{ aqua_fan.write(pos); SoftwareServo::refresh(); delay(50); }
			delay(50);
			
			digitalWrite(AV_EN, LOW);			
			
			Serial.println(F("Test complete"));
      isCommand = true;
		}
		if (inputString.startsWith(F("av="))) {
			delay(2000);
			digitalWrite(AV_EN, HIGH);	
			delay(50);
			aqua_fan.write((uint8_t)(inputString.substring(3, 6).toInt())); 
			Serial.println((uint8_t)(inputString.substring(3, 6).toInt()));
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			SoftwareServo::refresh(); delay(50);
			digitalWrite(AV_EN, LOW);			
			isCommand = true;
		}
		if (inputString.startsWith("set ")) {
			//проверим какой датчик будем привязывать
			if (inputString.indexOf("t1")>0) { tmpi = inputString.substring(7, 9).toInt(); Room1_DST.Link(tmpi, Searching1Wire[tmpi], true); }
			if (inputString.indexOf("t2")>0) { tmpi = inputString.substring(7, 9).toInt(); Room2_DST.Link(tmpi, Searching1Wire[tmpi], true); }
			//if (inputString.indexOf("tk")>0) { tmpi = inputString.substring(7, 9).toInt(); Kitchen_DST.Link(tmpi, Searching1Wire[tmpi], true); }
			if (inputString.indexOf("ta")>0) { tmpi = inputString.substring(7, 9).toInt(); Aqua_DST.Link(tmpi, Searching1Wire[tmpi], true); }
			if (inputString.indexOf("th")>0) { tmpi = inputString.substring(7, 9).toInt(); HeatTransfer_DST.Link(tmpi, Searching1Wire[tmpi], true); }
			if (inputString.indexOf("tc")>0) { tmpi = inputString.substring(7, 9).toInt(); Freon_DST.Link(tmpi, Searching1Wire[tmpi], true); }
			Search1W();
			isCommand = true;
		}
		if (inputString.startsWith(F("clear "))) {
			//проверим какой датчик будем отвязывать
			if (inputString.indexOf("t1")>0) { Room1_DST.UnLink(true); }
			if (inputString.indexOf("t2")>0) { Room2_DST.UnLink(true); }
			//if (inputString.indexOf("tk")>0) { Kitchen_DST.UnLink(true); }
			if (inputString.indexOf("ta")>0) { Aqua_DST.UnLink(true);}
			if (inputString.indexOf("th")>0) { HeatTransfer_DST.UnLink(true);}
			if (inputString.indexOf("tc")>0) { Freon_DST.UnLink(true); }
			if ( !(tmpi >-1 && tmpi < numberOfDevices)) { Serial.print(F("Invalid device member use 0...")); Serial.println(numberOfDevices-1, DEC); }
      isCommand = true;
		}
		if (inputString.equalsIgnoreCase(F("search1w")) or inputString.equalsIgnoreCase(F("search 1w")) or inputString.equalsIgnoreCase(F("s1w"))) {
			Search1W();
			Print1WDev();
			isCommand = true;
		}
		if (inputString.equalsIgnoreCase(F("print1w")) or inputString.equalsIgnoreCase(F("print 1w")) or inputString.equalsIgnoreCase(F("p1w"))) {
			Print1WDev();
			isCommand = true;
		}
    if (inputString.startsWith(F("date="))) { 
			//date=yyyy.mm.dd
      if( inputString.substring(5,   9).toInt() && 
          inputString.substring(10, 12).toInt() && (inputString.substring(10, 12).toInt() < 13) &&
          inputString.substring(13, 15).toInt() && (inputString.substring(13, 15).toInt() < 31)
        ){ 
				tmpDateTime = RTC.now();
        RTC.adjust(DateTime(inputString.substring(5, 9).toInt(), 
                            inputString.substring(10, 12).toInt(),
                            inputString.substring(13, 15).toInt(), 
                            tmpDateTime.hour(),tmpDateTime.minute(),tmpDateTime.second()
                            )); 
      } else { Serial.println(F("Invalid date format (use yyyy.mm.dd)")); }
        isCommand = true;
    }
    if (inputString.startsWith(F("time="))){
			//time=hh.mi.ss
      if( inputString.substring(5,  7).toInt() && (inputString.substring(5,  7).toInt() < 25) &&
          inputString.substring(8, 10).toInt() && (inputString.substring(8, 10).toInt() < 60) &&
          inputString.substring(11,12).toInt() && (inputString.substring(11,12).toInt() < 60)
        ){ 
				tmpDateTime = RTC.now();
        RTC.adjust(DateTime(tmpDateTime.year(), tmpDateTime.month(), tmpDateTime.day(), 
                            inputString.substring(5,  7).toInt(),
                            inputString.substring(8, 10).toInt(),
                            inputString.substring(11,12).toInt()
                            )); 
      } else { Serial.println(F("Invalid time format (use hh24.mi.ss, not 00:00:00)")); }
        isCommand = true;
    }
    if (inputString.startsWith(F("sp1=")) or inputString.startsWith(F("sp2="))) { 
			//spx=234444333333355555555522
      if( inputString.length() == 28 ){ 
        for (i = 4; i < 28; i++){ 
					if ( inputString.substring(i, i+1).toInt() < 0 || inputString.substring(i, i+1).toInt() > VENT_MAX_SPEED ) {
						Serial.println(F("Invalid speed profile - valid value 0...5")); 
            i = 100;
            break;
          }
        }
        if ( i < 100 ) { 
				  if ( inputString.substring(2, 3).equals("1") ){
					  Serial.println(inputString.substring(4, 28));
					  Vent1.SetMaxSpeed(&(inputString.substring(4, 28)));
					}
					if ( inputString.substring(2, 3).equals("2") ){
						Vent2.SetMaxSpeed(&(inputString.substring(4, 28)));
					}                   
				}                          
			} else { 
				Serial.println(F("Invalid speed profile - use \"sp#=########################\" - 24 speed value / 24 hour")); 
			}
    isCommand = true;
  }
      
  if (!isCommand) { Serial.println(F("Invalid command"));  }
    
  // clear the string:
  inputString = "";
  stringComplete = false;
} 
  
  if(!isConfig_1W || !isConfig_Vent || !isConfig_EEPROM || !isConfig_DHT) {
		MODE = MODE_ERROR;
	} else 
		if (!isEnable) {
			MODE = MODE_STOP;
		} else {	
			//время до регулирования меньше время разогрева датчика 
			/*if (prevMillis2 + REG_INT - ((unsigned long)AQ.PreheatTime)*1000 < curMillis) 
				// датчик не включали еще
				if (AQ.CurrentHeatingTime == 0 && !digitalRead(AQ_EN)) {
					digitalWrite(AQ_EN, HIGH); 					// включить датчик
				}
				//else AQ.CurrentHeatingTime = curMillis - (prevMillis2 + REG_INT - ((unsigned long)AQ.PreheatTime)*1000);
			*/
			if(curMillis - prevMillis2 > REG_INT) {
				/*#ifdef DEBUG 
					Serial.print(F("REG_INT")); Serial.print(" "); Serial.println(curMillis);
				#endif*/
		
				// Снимем показания с датчика кач-ва воздуха если он грелся сколько надо (если цикл регулирования форсирован - возможно датчик не прогрет)
				/*if (AQ.CurrentHeatingTime >= AQ.PreheatTime*1000) 
					AQ.Value = 102.3-analogRead(AQ_IN)/10.23 ;*/
				
			/*	#ifdef DEBUG 
				  Serial.print(AQ.CurrentHeatingTime); Serial.print(" "); Serial.println(curMillis);
					Serial.print(AQ.Value); Serial.print(" "); Serial.println(curMillis);
				#endif*/
				AQ.CurrentHeatingTime = 0;		
				//digitalWrite(AQ_EN, LOW);		
				/*#ifdef DEBUG 
					Serial.print(F("AQH=OFF")); Serial.print(" "); Serial.println(curMillis);
				#endif*/
				// Вычисляем параметры вентиляторов, нагревателей и заслонок
		
				NeedAddHeat = 0;
				//Нагреватели
				//Если холодно включить, тепло - выключить
				if (T-dT/10 > (float)Room1_DST.temp )	NeedAddHeat+=H1.On((int)HeatTransfer_DST.temp);
				else H1.Off();
				if (T-dT/10 > (float)Room2_DST.temp )	NeedAddHeat+=H2.On((int)HeatTransfer_DST.temp);
				else H2.Off();			
				if (T-dT/10 > (float)Room3_DHT.temp)	NeedAddHeat=NeedAddHeat+H3.On((int)HeatTransfer_DST.temp);
				else H3.Off();			
				//если NeedAddHeat>3 - источник тепла нужен другой
				if (NeedAddHeat>2){
					MODE = MODE_HEATING_ADD;
					//Serial.println(F("Need additional heating"));
				}

				//Обновим максимальные скорости
				Vent1.UpdateMaxSpeed(tmpDateTime.hour());
				Vent2.UpdateMaxSpeed(tmpDateTime.hour());
				
				tmpi = CalcVentSpeed((float)Room3_DHT.temp);
				tmpj = CalcVentSpeed(min((float)Room1_DST.temp, (float)Room2_DST.temp));
				
				Vent1.SetSpeedSafe(tmpi);
				Serial.print("VS1="); Serial.println(Vent1.Speed, DEC);
				Vent2.SetSpeedSafe(tmpj);
				Serial.print("VS2="); Serial.println(Vent2.Speed, DEC);
				
				//Дуть на аквариум ?
				//Если теплее 24х - и на улице 24 и менее - дуть
				// 13 - закрыто 
				// 120- открыто
				pos = aqua_fan.read();
				Serial.print("Aqua fan valve ="); Serial.println(pos);
				if((float)Aqua_DST.temp > 24 && (float)External_DHT.temp <= 24) {
					if(pos!=120 ) {
						digitalWrite(AV_EN, HIGH);	
						delay(50);
						if(pos > 120) //decrease
							{ aqua_fan.write(120); SoftwareServo::refresh(); delay(50); }
						else //increase
							for(; pos < 120; pos ++)                                    
							{ aqua_fan.write(pos); SoftwareServo::refresh(); delay(50); }
						delay(50);
						digitalWrite(AV_EN, LOW);
						Serial.println("Aqua fan opened");
					}
				}
				else {
					if(pos!=13 ) {
						digitalWrite(AV_EN, HIGH);	
						delay(50);
						if(pos > 13) //decrease
							for(; pos>13; pos --)                                          
								{ aqua_fan.write(pos); SoftwareServo::refresh(); delay(50); }
						else //increase
							{ aqua_fan.write(13); SoftwareServo::refresh(); delay(50); }
						delay(50);
						digitalWrite(AV_EN, LOW);
						Serial.println("Aqua fan closed");
					}
				}		
				
				StatusToWEB();
				//Сбрасываем счетчики
				moveCounter = 0;
				prevMillis2 = curMillis;
			}
		}
		
  if(curMillis - prevMillis1 > POOL_INT){ 
		#ifdef DEBUG 
			Serial.print(F("POOL_INT")); Serial.print(" "); Serial.println(curMillis); 
		#endif
				
		prevMillis1 = curMillis;
    tmpDateTime = RTC.now();  
		//Обновим температуры 
		Refresh1WData();
		//Снимем влажность в комнате и вытяжке (250msec/call)
		//Вытяжка
		if(!Room3_DHT.ReadSensor() || !Exhaust_DHT.ReadSensor()	|| !External_DHT.ReadSensor()) isConfig_DHT = false;
		else isConfig_DHT = true;
		
		//Проверка не включить ли вытяжку - если в кухне над плитой теплее чем в комнате на 3 и более градусов
		/*if (TS[_Tk].temp - Room3_DHT.t > 0.5){
			Serial.println(F("Exshaust ON"));
		}	else {
			Serial.println(F("Exshaust OFF"));
		}*/    
  }
	
	/*if (digitalRead(AV_EN))
		SoftwareServo::refresh();	*/
}
