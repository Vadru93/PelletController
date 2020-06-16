#include <iostream>
#include "therm.h"
#include <stdio.h>
#include <time.h>
#include <iostream>
#include <csignal>
#include <string.h>
#include <sstream> 
#include <cstdlib>
#include <math.h>
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <iomanip>
#include "json.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <sched.h>
#include "Header.h"
#include <algorithm>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>

using json = nlohmann::json;

enum commands
{
	C_CONTINUE = 0,
	C_REBOOT = 1,
	C_EXIT = 2,
};


#define MOTOR_INC 40
#define MOTOR_DEC 38
#define DIST_TRIGG 12
#define DIST_ECHO 18
#define ONE_WIRE 7
#define MAX_STEP_WAITTIME 11000
#define TEMP_THRESHOLD 0.4f
#define TEMP_THRESHOLD_MOVING 0.12f
#define MAX_POS 200000
#define TARGET_TEMP_WAITTIME 12000
#define TWENTYFOURHOURS 86400000
#define FOURMINUTES 240000
#define MAX_MAX_STEP 20
#define MAX_TEMP 45
#define MIN_TEMP 20
#define TWOMINUTES 120000
#define TENMINUTES 600000
#define TARGET_THRESHOLD 55000
#define TARGET_THRESHOLD2 5000
#define EIGHTMINUTES 480000
#define STARTPOS 54920
#define UPDATEFREQ 10//in seconds
#define SUMMERTEMP 10

#define SECONDS(x) x*1000
#define MINUTES(x) x*SECONDS(60)
#define HOURS(x) x*MINUTES(60)

#define SONICSPEED 0.01705f//speed of sound @ ~ 1bar pressure & ~ 15°C in cm/μs/2

bool reallyMovingValve = false;
float oldTarget = DEVICE_DISCONNECTED_C;
bool targetSet = false;
bool decreasing = false;
bool force = false;
unsigned char command = 0;
unsigned int summer_reboot = 0;

#define NUM_DIST 8
float distances[NUM_DIST];
unsigned int idx = 0;

int currentday = 0;
int currenthour = 0;

/*unsigned int hours = 0;
//unsigned int minutes = 0;

double h = 0.0f;
double m = 0.0f;*/


class ExtraInfo
{
public:
	unsigned int summerStart;
	int days;
};

unsigned int MAX_STEP = 10;
unsigned int days = 1;

static  const float _map(float value,
	float start1,
	float stop1,
	float start2,
	float stop2) {
	return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
}


using namespace std;


static const DeviceAddress RadiatorTemp = { 0x28, 0x2E, 0xB5, 0x93, 0xB, 0x0, 0x0, 0x8E };
static const DeviceAddress OutsideTemp = { 0x28, 0xC4, 0x92, 0x81, 0xB, 0x0, 0x0, 0xFE };

bool cleanupDone = false;
OneWire* cleanupWire = NULL;

int listenfd = 0, client = 0;
bool connected = false;
PI_THREAD(CheckForConnections);
void SendJSON(json j);


template<typename... Args> void _printf(const char* text, Args... args) {
	if (connected)
	{
		char msg[256] = "";

		sprintf(msg, text, args...);
		json j = {
			{"type", "print"},
			{"text", msg}
		};
		SendJSON(j);
	}
	printf(text, args...);
}



class TIMESTAMP
{
	const char* C_TYPE[5] = { "Position", "TargetTemp", "OutsideTemp", "RadTemp" };
public:
	enum TYPE
	{
		POS = 0,
		TAR = 1,
		OUT = 2,
		RAD = 3
	};
	TYPE type;
	union
	{
		float value;
		unsigned int pos;
	};


	int hour;
	int min;

	TIMESTAMP()
	{

	}


	bool compare(float v)
	{

		if (v > value)
			return value + 0.25f <= v;
		return value - 0.25f >= v;
	}

	bool compare(unsigned int v)
	{
		return pos != v;
	}


	/*float average_int(tm* timeinfo)
	{
		double multiplier = 0.0f;

		if (timeinfo->tm_hour)
			multiplier = (double)timeinfo->tm_hour / (double)hours;
		int diff = 60 - min + timeinfo->tm_min;
		multiplier += (double)diff / (double)minutes;

		float pos = (float)*(unsigned int*)&value;

		return pos * multiplier;
	}*/

	/*float average(tm* timeinfo)
	{
		double multiplier = 0.0f;
		if (timeinfo->tm_hour)
			multiplier = (double)timeinfo->tm_hour / (double)hours;
		int diff = 60 - min + timeinfo->tm_min;
		multiplier += (double)diff / (double)minutes;

		if (!type != POS)
			return value * multiplier;
		else
		{
			float pos = (float)*(unsigned int*)&value;

			return pos * multiplier;
		}
	}*/

	bool compare(tm* timeinfo)
	{
		return (timeinfo->tm_min > min + 9 || timeinfo->tm_hour > hour);
	}
	TIMESTAMP(float v, int h, int m, TYPE t)
	{
		type = t;
		value = v;
		hour = h;
		min = m;

		_printf("TS created\n");

		int fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);
		char test[256] = "";
		sprintf(test, "Sensor,Value,Time\n%s,%.2f,%d:%d\n\n", C_TYPE[type], value, hour, min);
		write(fd, &test, strlen(test));
		close(fd);
		chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);
	}
	TIMESTAMP(unsigned int v, int h, int m)
	{
		type = POS;
		this->value = v;
		this->hour = h;
		this->min = m;
		_printf("TS created_int %u %d:%d\n", v, h, m);

		int fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);
		char test[256] = "";
		sprintf(test, "Sensor,Value,Time\n%s,%u,%d:%d\n\n", C_TYPE[type], v, h, m);
		write(fd, &test, strlen(test));
		close(fd);
		chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);
	}
};

//function to calculate the average of data in a time period.
double average(std::vector<TIMESTAMP>& data, const struct tm* const timeinfo)
{

	unsigned int max = data.size() - 1;//-1 to be able to use as index to access the data.
	TIMESTAMP::TYPE type = data[0].type;//if the type is pos the data is stored as unsigned int, else it's stored as float.

	if (max == 0)//only 1 value, return it
		return type != TIMESTAMP::POS ? data[0].value : (float)data[0].pos;

	unsigned int hours = 24 - data[0].hour + timeinfo->tm_hour;//One day is 24 hours, check what time we started the recordings of data and compare to time now.
	if (timeinfo->tm_hour > data[0].hour)// if the hour now is greater than the hour we started to record, it means we are still on the same day.
		hours = timeinfo->tm_hour - data[0].hour;
	double m, h = 0.0f;


	if (data[0].min == timeinfo->tm_min)
	{
		h = (double)hours;
		m = (double)(hours * 60);
		_printf("EQUAL\n");
	}
	else if (timeinfo->tm_min > data[0].min)
	{
		m = (double)(hours * 60 + timeinfo->tm_min - data[0].min);
		h = m / 60.0f;//(double)hours + (double)(timeinfo->tm_min - data[0].min) / 60.0f;
		_printf("BIGGER\n");
	}
	else
	{
		unsigned int tmp = hours - 1;
		m = (double)((tmp * 60) + (60 - data[0].min + timeinfo->tm_min));
		h = m / 60.0f;//(double)(tmp)+(double)(60 - data[0].min + timeinfo->tm_min) / 60.0f;
		_printf("SMALLER\n");
	}


	unsigned int i = 0;
	double dHour = h;
	double dMin = m;
	double average = 0.0f;
	double multiplier = 0.0f;

	_printf("Max %d [Max].val %2.f [Max].Hours %u Hours %.2f Minutes %.2f\n", max, type != TIMESTAMP::POS ? data[max].value : data[max].pos, data[max].hour+1, h, m);


	for (; i < max; i++)
	{
		unsigned int value = data[i].pos;
		if (value != 0)
		{
			unsigned int j = i + 1;
			int hdiff = data[j].hour - data[i].hour;

			if (hdiff)
			{
				if (data[j].min == data[i].min)
				{
					multiplier = (double)hdiff / dHour;
				}
				else if (data[j].min > data[i].min)
				{
					multiplier = (double)hdiff / dHour;
					multiplier += (double)(data[j].min - data[i].min) / dMin;
				}
				else
				{
					multiplier = (double)(60 - data[i].min + data[j].min) / dMin;
					hdiff--;
					if (hdiff > 0)
					{
						multiplier += (double)hdiff / dHour;
					}
				}
			}
			else
				multiplier = (double)(data[j].min - data[i].min) / dMin;


			if (type != TIMESTAMP::POS)
				average += data[i].value * multiplier;
			else
				average += (double)value * multiplier;
		}

	}

	
	unsigned int value = data[max].pos;
	if (value != 0)
	{
		int hdiff = timeinfo->tm_hour - data[max].hour;

		if (hdiff)
		{
			if (timeinfo->tm_min == data[max].min)
			{
				multiplier = (double)hdiff / dHour;
			}
			else if (timeinfo->tm_min > data[max].min)
			{
				multiplier = (double)hdiff / dHour;
				multiplier += (double)(timeinfo->tm_min - data[max].min) / dMin;
			}
			else
			{
				multiplier = (double)(60 - data[max].min + timeinfo->tm_min) / dMin;
				hdiff--;
				if (hdiff > 0)
				{
					multiplier += (double)hdiff / dHour;
				}
			}
		}
		else
			multiplier = (double)(timeinfo->tm_min - data[max].min) / dMin;


		if (type != TIMESTAMP::POS)
			average += data[max].value * multiplier;
		else
			average += (double)value * multiplier;
	}
	return average;
}



/*float average(std::vector<TIMESTAMP>& data, struct tm* timeinfo)
{
	unsigned int max = data.size()-1;
	unsigned int i = 0;
	double dHour = (double)hours;

	TIMESTAMP::TYPE type = data[0].type;
	std::pair<double, unsigned int> avg[24];


	for (; i < max; i++)
	{

		unsigned int j = i + 1;
		if (data[i].hour == data[j].hour)
		{
			avg[data[i].hour].second += data[j].min - data[i].min;

			if (type != TIMESTAMP::POS)
			{
				double value = (double)data[i].value;
				if (value != 0.0f)
					avg[data[i].hour].first += (double)(data[j].min - data[i].min) / 60.0f * value;
			}
			else
			{
				unsigned int v = *(unsigned int*)&data[i].value;
				if(v != 0)
				    avg[data[i].hour].first += (double)(data[j].min - data[i].min) / 60.0f * (double)v;
			}
		}
		else if (data[i].hour == data[j].hour - 1 && data[i].min <= data[j].min)
		{
			avg[data[i].hour].second += 60 - data[i].min + data[j].min;

			if (type != TIMESTAMP::POS)
			{
				double value = (double)data[i].value;
				if (value != 0.0f)
					avg[data[i].hour].first += (double)(60 - data[i].min + data[j].min) / 60.0f * value;
			}
			else
			{
				unsigned int v = *(unsigned int*)&data[i].value;
				if (v != 0)
				    avg[data[i].hour].first += (double)(60 - data[i].min + data[j].min) / 60.0f * (double)v;
			}
		}

	}

	if (timeinfo->tm_hour == data[max].hour)
	{
		avg[data[max].hour].second += timeinfo->tm_hour - data[max].hour;

		if (type != TIMESTAMP::POS)
		{
			double value = (double)data[max].value;

			if (value != 0.0f)
				avg[data[max].hour].first += (double)(timeinfo->tm_min - data[max].min) / 60.0f * value;
		}
		else
		{
			unsigned int v = *(unsigned int*)&data[max].value;
			if (v != 0)
				avg[data[i].hour].first += (double)(timeinfo->tm_min - data[i].min) / 60.0f * (double)v;
		}
	}
	else
	{
		avg[data[max].hour].second += 60 - data[max].min + timeinfo->tm_min;

		if (type != TIMESTAMP::POS)
		{
			double value = (double)data[max].value;
			if (value != 0.0f)
				avg[data[max].hour].first += (double)(60 - data[max].min + timeinfo->tm_min) / 60.0f * value;
		}
		else
		{
			unsigned int v = *(unsigned int*)&data[max].value;
			if (v != 0)
				avg[data[max].hour].first += (double)(60 - data[max].min + timeinfo->tm_min) / 60.0f * (double)v;
		}

	}

 

	float average = 0.0f;
	//unsigned int hour = data[max].hour - data[0].hour;

	for (i = data[0].hour; i < dHour + data[0].hour; i++)
	{
		if (i <= data[max].hour)
		{
			if (avg[i].second < 60)
			{
				unsigned int v = 0;
				double value = 0.0f;
				if (type != TIMESTAMP::POS)
				{
					value = (double)data[i].value;
				}
				else
				{
					v = *(unsigned int*)&data[i].value;
				}

				unsigned int j = max;
				for (; j > 0; j--)
				{
					if (data[j].hour == i)
						break;
				}
				if (type != TIMESTAMP::POS)
				{
					if (value != 0.0f)
						avg[i].first += (double)(60 - avg[i].second) / 60.0f * value;
				}
				else if (v != 0)
					avg[i].first += (double)(60 - avg[i].second) / 60.0f * (double)v;

			}


			if (avg[i].first != 0.0f)
				average += avg[i].first / dHour;
		}
		else
		{

		}
	}


	return average;
}*/

std::vector<TIMESTAMP> AVG_POS;
std::vector<TIMESTAMP> AVG_RAD;
std::vector<TIMESTAMP> AVG_OUT;
std::vector<TIMESTAMP> AVG_TAR;



void SendJSON(json j)
{
	vector <uint8_t> bson = json::to_bson(j);
	auto s = bson.size();

	vector <uint8_t> final;
	final.push_back(s >> 0);
	final.push_back(s >> 8);
	final.push_back(s >> 16);
	final.push_back(s >> 24);

	for (int i = 0; i < s; i++)
	{
		final.push_back(bson[i]);
	}

	if (write(client, &final.front(), final.size()) != final.size())
	{
		connected = false;
		printf("connection failed\n");
		close(client);


		pthread_t thread_conn;
		pthread_attr_t thread_attr;
		sched_param sp;
		memset(&sp, 0, sizeof(sp));
		pthread_attr_init(&thread_attr);


		pthread_attr_setschedpolicy(&thread_attr, SCHED_IDLE);
		pthread_attr_setschedparam(&thread_attr, &sp);
		pthread_create(&thread_conn, &thread_attr, CheckForConnections, NULL);
		pthread_attr_destroy(&thread_attr);
		//piThreadCreate(CheckForConnections);
	}

}



void Cleanup()
{
	if (cleanupDone)
		return;

	timeval curTime;
	gettimeofday(&curTime, NULL);
	struct tm* timeinfo = localtime(&curTime.tv_sec);

	int fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);
	char test[256] = "";
	sprintf(test, "Event,Date,Time\nC_CLEANUP,%d/%d/%d,%d:%d\n\n", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min);
	write(fd, &test, strlen(test));
	close(fd);
	chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);

	delayMicroseconds(10);
	digitalWrite(MOTOR_INC, LOW);
	delayMicroseconds(10);
	digitalWrite(MOTOR_DEC, LOW);
	delayMicroseconds(10);
	digitalWrite(DIST_TRIGG, LOW);
	if (cleanupWire)
		cleanupWire->depower();

	cleanupDone = true;
	shutdown(client, SHUT_RDWR);
	close(client);
}

bool pumpStarted = true;
void StartPump()
{
	if (!pumpStarted)
	{
		pumpStarted = true;

		if (connected)
		{
			json j = {
					{"type", "data"},
				{"pump_enabled", true}
			};
			SendJSON(j);
		}
	}
}

void StopPump()
{
	if (pumpStarted)
	{
		pumpStarted = false;

		if (connected)
		{
			json j = {
						{"type", "data"},
					{"pump_enabled", false}
			};
			SendJSON(j);
		}
	}
}


bool movingValve = false;

float targetTemp = DEVICE_DISCONNECTED_C;
float curRadTemp = targetTemp;
float curOutsideTemp = 0.0f;
bool update = true;
unsigned int targetPos = 0;
unsigned int stage = 0;

static int curStep = 0;
static int valveTimer = 0;
#define valvePosition valveTimer
unsigned int valvePos = 0;
static unsigned int increaseStart = 0;
static unsigned int decreaseStart = 0;

bool summerMode = false;


DallasTemperature temp;

unsigned int GetTimePassed(unsigned int past)
{
	unsigned int now = millis();
	if (summerMode && summer_reboot)
		return (now + summer_reboot - past);
	else if (now < past)
		return (0xFFFFFFFF - past);
	else
		return (now - past);
}

class WaitTimer
{
public:
	unsigned int start;
	unsigned int time;
	bool waiting;
	bool interrupted;
	char func[30];

	WaitTimer()
	{
		time = 0;
		start = 0;
		waiting = false;
		interrupted = false;
		func[0] = 0x0;

	}

	void Print()
	{
		if (waiting)
		{
			if (!decreasing)
			{
				unsigned int _time = GetTimePassed(this->start);
				_printf("Waiting %.2f%% %s T %.2f R %.2f O %.2f valvePosition %d\n",
					_map(_time, 0, (float)this->time, 0, 100),
					func,
					targetTemp,
					curRadTemp,
					curOutsideTemp,
					reallyMovingValve ? (valvePosition + _time) : valvePosition);
				valvePos = reallyMovingValve ? (valvePosition + _time) : valvePosition;
			}
			else
			{
				unsigned int _time = GetTimePassed(this->start);
				_printf("Waiting %.2f%% %s T %.2f R %.2f O %.2f valvePosition %d\n",
					_map(_time, 0, (float)this->time, 0, 100),
					func,
					targetTemp,
					curRadTemp,
					curOutsideTemp,
					reallyMovingValve ? (!decreasing ? (valvePosition + _time) : valvePosition - _time) : valvePosition);
				valvePos = reallyMovingValve ? (valvePosition + _time) : valvePosition;
			}
		}
	}
};


WaitTimer waitTimer[] = { WaitTimer(), WaitTimer(), WaitTimer(), WaitTimer() };

void _delay(unsigned int time, const char* _func)
{
	unsigned int timers = 0;
	while (waitTimer[timers].waiting)
		timers++;
	strcpy(waitTimer[timers].func, _func);
	waitTimer[timers].waiting = true;
	waitTimer[timers].start = millis();
	waitTimer[timers].time = time;
	delay(time);
	waitTimer[timers].waiting = false;
}
#define _delay(x) _delay(x, __FUNCTION__)
#define PRINT_THREADERROR() _printf("THREAD ERROR: %s", __FUNCTION__)


/*PI_THREAD(requestTemps)
{
	while (true)
	{

		temp.requestTemperatures();
		delay(500);
	}
}
*/
float getRadTemp()
{
	if (!temp.requestTemperaturesByAddress(RadiatorTemp))
		temp.requestTemperaturesByAddress(RadiatorTemp);
	float _temp = temp.getTempC(RadiatorTemp);
	if (_temp != DEVICE_DISCONNECTED_C && _temp != 85.0f)
		return _temp;
	else
		return curRadTemp;
}



float getOutsideTemp()
{
	if (!temp.requestTemperaturesByAddress(OutsideTemp))
		temp.requestTemperaturesByAddress(OutsideTemp);
	float _temp = temp.getTempC(OutsideTemp);
	if (_temp != DEVICE_DISCONNECTED_C && _temp != 85.0f)
		return _temp;
	else
		return curOutsideTemp;
}

class TempMap
{
public:
	float outTemp;
	float targetTemp;
	unsigned int valvePos;


	TempMap(float _temp1, float _temp2, unsigned int _pos)
	{
		outTemp = _temp1;
		targetTemp = _temp2;
		valvePos = _pos;
	}
};


static const DistMap pelletsLeft[] = {
	DistMap(100.0f, 2.0f),
	DistMap(104.5f, 1.75f),
	DistMap(109.0f, 1.5f),
	DistMap(113.5f, 1.25f),
	DistMap(118.0f, 1.0f),
	DistMap(121.0f, 0.75f),
	DistMap(124.0f, 0.5f),
	DistMap(127.0f, 0.25f),
	DistMap(130.0f, 0.0f),
};

bool initing = false;

void DistMap::init()
{
	printf("going to init\n");
	idx = 0;
	initing = true;
	distance();
	//printf("Hmm?!\n");
	initing = false;
	while (idx < NUM_DIST)
	{
		//printf("ok?!\n");
		distances[idx] = this->dist;
		idx++;
	}

	/*for (int i = 0; i < 5; i++)
	{
		this->distance();
	}

	initing = false;
	for (int i = 0; i < 5; i++)
	{
		this->distance();
	}*/
}

#define AVG_SIZE 6

unsigned int stop[AVG_SIZE];
unsigned int start[AVG_SIZE];
float dt[AVG_SIZE];

void DistMap::distance()
{
	for (int i = 0; i < AVG_SIZE; i++)
	{
		digitalWrite(DIST_TRIGG, HIGH);
		delayMicroseconds(10);
		digitalWrite(DIST_TRIGG, LOW);

		unsigned int now = micros();
		while (digitalRead(DIST_ECHO) == 0 && micros() - now < 30000);
		start[i] = micros();
		while (digitalRead(DIST_ECHO) == 1);
		stop[i] = micros();
		stop[i] -= 10;

		dt[i] = ((float)(stop[i] - start[i]) * SONICSPEED);
		delayMicroseconds(10);
	}


	std::pair<float*, float*> minmax = std::minmax_element(std::begin(dt), std::end(dt));

	this->dist = dt[0] + dt[1] + dt[2] + dt[3] + dt[4] + dt[5];
	this->dist -= *minmax.first + *minmax.second;
	this->dist *= 0.25f;


	if (!initing)
	{
		if (idx == NUM_DIST)
			idx = 0;

		distances[idx] = this->dist;
		printf("real %.1f ", this->dist);

		this->dist = 0.0f;
		for (int i = 0; i < NUM_DIST; i++)
		{
			this->dist += distances[i];
		}

		minmax = std::minmax_element(std::begin(distances), std::end(distances));
		this->dist -= *minmax.first + *minmax.second;
		float avg_array[6];
		float* it = std::begin(distances);

		for (int i = 0; i < 6; i++)
		{
			while (it == minmax.first || it == minmax.second)
				it++;
			avg_array[i] = *it;
			it++;
		}
		minmax = std::minmax_element(std::begin(avg_array), std::end(avg_array));
		this->dist -= *minmax.first + *minmax.second;
		this->dist *= 0.25f;
		idx++;
	}


	signed int i = -1;
	unsigned char interpolated = 0;
	const DistMap* otherMap;
	while (i < (signed int)(sizeof(pelletsLeft) / sizeof(DistMap) - 1))
	{
		//printf("next itm %d\n", i);
		i++;
		if (i + 1 < (sizeof(pelletsLeft) / sizeof(DistMap)))
		{
			if (this->dist < pelletsLeft[i + 1].dist)
			{
				//_printf("i+1 really %d\n", i + 1);
				float diff = pelletsLeft[i + 1].dist - this->dist;
				float diff2 = this->dist - pelletsLeft[i].dist;
				if (diff > diff2)
				{
					if (diff2 < 0.06f)
						break;
					interpolated = 1;
					otherMap = &pelletsLeft[i + 1];
					break;
				}
				else
				{
					if (diff < 0.06f)
						break;
					interpolated = 2;
					otherMap = &pelletsLeft[i];
					i++;
					break;
				}
			}
		}
	}
	if (i == -1)
		_printf("WTF?\n");

	//printf("Yaaa\n");
	this->bags = pelletsLeft[i].bags;
	/*if (!interpolated)
	{
		//printf("not int i %d\n", i);

		if (this->dist != pelletsLeft[i].dist && ((i == 0 && this->dist < pelletsLeft[i].dist) || (i == sizeof(pelletsLeft) / sizeof(DistMap) - 1 && this->dist > pelletsLeft[i].dist)))
		{
			//printf("too big or small?\n");
			this->dist = pelletsLeft[i].dist;
			this->bags = pelletsLeft[i].bags;
		}
		//printf("dist %f bags %f\n", this->dist, this->bags);
	}
	else if (interpolated == 1)
	{
		//printf("int 1\n");
		this->bags = _map(this->dist, pelletsLeft[i].dist, otherMap->dist, pelletsLeft[i].bags, otherMap->bags);
		//printf("dst%f idst%f odst%f ibag%f dbag%f\n", this->dist, pelletsLeft[i].dist, otherMap->dist, pelletsLeft[i].bags, otherMap->bags);
	}
	else
	{
		//printf("int 2\n");
		this->bags = _map(this->dist, otherMap->dist, pelletsLeft[i].dist, otherMap->bags, pelletsLeft[i].bags);
		//printf("dst%f idst%f odst%f ibag%f dbag%f\n", this->dist, pelletsLeft[i].dist, otherMap->dist, pelletsLeft[i].bags, otherMap->bags);
	}*/
	//if (!initing)
	_printf("pellet dist %.1f bags %.2f raw %u %u %u %u %u %u\n", this->dist, this->bags, stop[0] - start[0], stop[1] - start[1], stop[2] - start[2], stop[3] - start[3], stop[4] - start[4], stop[5] - start[5]);
	/*else
	{
		distances[idx] = this->dist;
		idx++;
		if (idx == 5)
			_printf("pellet dist %.1f bags %.2f raw %u %u %u %u %u %u\n", this->dist, this->bags, stop - start, stop1 - start1, stop2 - start2, stop3 - start3, stop4 - start4, stop5 - start5);
	}*/
}



static const TempMap targets[] = {
TempMap(-4.0f, 35.0f, 73000),
TempMap(0.0f, 32.5f, 50000),
TempMap(4.0f, 30.7f, 23000),
TempMap(5.0f, 30.5f, 21000),
TempMap(5.5f, 30.2f, 19000),
TempMap(6.0f, 29.5f, 15000),
TempMap(7.0f, 29.0f, 0) };
//TempMap(10.0f, 25.0f, 0) };

TempMap const* target = &targets[3];

bool insideSetValvePos = false;
void SetValvePos(unsigned int pos)
{
	movingValve = true;
	_printf("Going to set valvePos %d\n", pos);
	while (reallyMovingValve);
	movingValve = true;
	//StartPump();

	delay(1000);

	if (pos == valvePosition || ((int)pos >= (int)valvePosition - 2000 && pos <= valvePosition + 2000))
	{
		_printf("Target Pos Set R %.2f T %.2f\n", curRadTemp, targetTemp);

		_delay(5000);
		targetSet = true;
		movingValve = false;
		force = false;
		_printf("returning...\n");
		//piLock(0);
		insideSetValvePos = false;
		//piUnlock(0);

		return;
	}


	if (pos > valvePosition)
	{
		unsigned int p = 0;
		while (targetTemp <= curRadTemp - 0.12f)
		{

			if (p < 4)
				_delay(MINUTES(2));
			else
			{
				_printf("Target Temp Set R %.2f T %.2f\n", curRadTemp, targetTemp);

				_delay(5000);
				targetSet = true;
				movingValve = false;
				force = false;
				_printf("returning...\n");
				//piLock(0);
				insideSetValvePos = false;
				//piUnlock(0);
				return;
			}
			p++;
			/*movingValve = false;
			return;*/
		}
		decreasing = false;
		pos -= valvePosition;
		reallyMovingValve = true;
		digitalWrite(MOTOR_INC, HIGH);
		_delay(pos);
		digitalWrite(MOTOR_INC, LOW);
		reallyMovingValve = false;
		valvePosition += pos;
		valvePos = valvePosition;
	}
	else if (curRadTemp >= targetTemp - 0.25f)
	{
		unsigned int p = 0;
		while (targetTemp <= curRadTemp - 0.12f)
		{
			if (p < 4)
				_delay(MINUTES(2));
			else
			{
				_printf("Target Temp Set R %.2f T %.2f\n", curRadTemp, targetTemp);

				_delay(5000);
				targetSet = true;
				movingValve = false;
				force = false;
				_printf("returning...\n");
				//piLock(0);
				insideSetValvePos = false;
				//piUnlock(0);
				return;
			}
			p++;
		}
		decreasing = true;
		pos = valvePosition - pos;
		reallyMovingValve = true;
		digitalWrite(MOTOR_DEC, HIGH);
		_delay(pos);
		digitalWrite(MOTOR_DEC, LOW);
		reallyMovingValve = false;
		valvePosition -= pos;
		valvePos = valvePosition;
		decreasing = false;
	}
	_printf("set pos %d\n", valvePosition);
	if (curRadTemp >= targetTemp - 0.25f)
		_delay(MINUTES(2));
	else
		_delay(5000);
	targetSet = true;
	movingValve = false;
	force = false;
	_printf("wait finished setvalvepos\n");
	//piLock(0);
	insideSetValvePos = false;
	//piUnlock(0);
}
float oldTempTrue = DEVICE_DISCONNECTED_C;

bool UpdateTarget(bool update = false)
{
	//_printf("Update?\n");
	if (!update && oldTarget == DEVICE_DISCONNECTED_C && oldTempTrue == DEVICE_DISCONNECTED_C)
		return false;
	_printf("Update\n");
	signed int i = -1;
	unsigned char interpolated = 0;
	TempMap const* otherMap;
	//_printf("i %d size %d beginning\n",
		/*i,
		sizeof(targets) / sizeof(TempMap) - 1);*/
	while (i < (signed int)(sizeof(targets) / sizeof(TempMap) - 1))
	{

		i++;
		/*_printf("i %d size %d loopa\n",
			i,
			sizeof(targets) / sizeof(targets[0]) - 1);*/
		if (curOutsideTemp <= targets[i].outTemp)
			break;
		else if (i + 1 < (sizeof(targets) / sizeof(TempMap)))
		{
			//_printf("i+1 %d\n", i+1);
			/*target = dummyTarget;
			target->outTemp = _temp;
			target->targetTemp = map(_temp,
				targets[i].outTemp,
				targets[i + 1].outTemp,
				targets[i].targetTemp,
				targets[i + 1].targetTemp);
			target->valvePos = map(_temp,
				targets[i].outTemp,
				targets[i + 1].outTemp,
				targets[i].valvePos,
				targets[i + 1].valvePos);
			goto skip;*/
			if (curOutsideTemp < targets[i + 1].outTemp)
			{
				//_printf("i+1 really %d\n", i + 1);
				float diff = targets[i + 1].outTemp - curOutsideTemp;
				float diff2 = curOutsideTemp - targets[i].outTemp;
				if (diff > diff2)
				{
					if (diff2 < 0.06f)
						break;
					interpolated = 1;
					otherMap = &targets[i + 1];
					break;
				}
				else
				{
					if (diff < 0.06f)
						break;
					interpolated = 2;
					otherMap = &targets[i];
					i++;
					break;
				}
			}
		}
	}
	if (i == -1)
		_printf("WTF?\n");
	//_printf("i %d elo\n", i);

	target = &targets[i];
	if (!interpolated)
	{
		if (target->targetTemp == targetTemp)
			return false;
		targetPos = target->valvePos;
		if (i == 0 && curOutsideTemp < target->outTemp)
			targetTemp = MAX_TEMP;
		else if (i == sizeof(targets) / sizeof(TempMap) - 1 && curOutsideTemp > target->outTemp)
		{
			targetTemp = MIN_TEMP;
			targetPos = 0;
		}
		else
			targetTemp = target->targetTemp;
	}
	else if (interpolated == 1)
	{
		targetTemp = _map(curOutsideTemp, target->outTemp, otherMap->outTemp, target->targetTemp, otherMap->targetTemp);
		targetPos = _map(curOutsideTemp, target->outTemp, otherMap->outTemp, target->valvePos, otherMap->valvePos);
	}
	else
	{
		targetTemp = _map(curOutsideTemp, otherMap->outTemp, target->outTemp, otherMap->targetTemp, target->targetTemp);
		targetPos = _map(curOutsideTemp, otherMap->outTemp, target->outTemp, otherMap->valvePos, target->valvePos);
	}

	_printf("Using Target%d O %.2f T %.2f valvePosition %d\n", i, target->outTemp, targetTemp, targetPos);
	if (update)
	{
		if (targetTemp == oldTarget)
			return false;
		oldTarget = targetTemp;
	}
	return true;
}

bool updateTemperatures = true;

bool OutsideTempChanged(float oldTemp)
{
	if (curOutsideTemp == oldTemp)
		return false;
	if (curOutsideTemp < oldTemp)
	{
		if (curOutsideTemp < oldTemp - 0.24f)
		{
			if (curOutsideTemp < oldTempTrue - 0.24f)
			{
				UpdateTarget();
				oldTempTrue = curOutsideTemp;
			}
		}
		return curOutsideTemp <= oldTemp - 0.5f;
	}
	else
	{
		if (curOutsideTemp > oldTemp + 0.24f)
		{
			if (curOutsideTemp > oldTempTrue + 0.24f)
			{
				UpdateTarget();
				oldTempTrue = curOutsideTemp;
			}
		}
		return curOutsideTemp >= oldTemp + 0.5f;
	}
}

PI_THREAD(SetValvePosThread)
{
	//piLock(0);
	insideSetValvePos = true;
	//piUnlock(0);
	SetValvePos(targetPos);
	return NULL;
}


DistMap pellet(0.0f, 0.0f);

PI_THREAD(CheckForConnections)
{

	//related with the server
	struct sockaddr_in serv_addr;

	/*unsigned char buf[256] = "";

	memset(&buf, '0', sizeof(buf));*/
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	unsigned int VAL = 1;
	if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, &VAL, sizeof(int)) == -1)
		printf("Sock err\n");

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(1337);

	bind(listenfd, (struct sockaddr*) & serv_addr, sizeof(serv_addr));
	_printf("binding\n");

	if (listen(listenfd, SOMAXCONN) == -1)
		printf("listen err\n");
	_printf("listening\n");
	client = accept(listenfd, (struct sockaddr*)NULL, NULL);

	int x = fcntl(client, F_GETFL, 0);
	fcntl(client, F_SETFL, x | TCP_NODELAY);
	_printf("Connected\n");
	close(listenfd);
	connected = true;
	signal(SIGPIPE, SIG_IGN);
	json j = {
					{"type", "data"},
				{"rad_temp", curRadTemp},
				{"boiler_temp", 65},
				{"target_temp", targetTemp},
				{"outside_temp", curOutsideTemp},
				{"valve_pos", valvePos},
				{"pump_enabled", pumpStarted}
	};


	pellet.init();
	//pellet.distance();
	j.push_back(json::object_t::value_type("dist", pellet.dist));
	j.push_back(json::object_t::value_type("bags", pellet.bags));

	SendJSON(j);
	return NULL;
}


float oldRad = -127.0f;
float oldTar = -127.0f;
float oldOut = -127.0f;
unsigned int oldPos = MAX_POS;


inline bool ValuesChanged()
{
	return oldRad != curRadTemp ||
		oldTar != targetTemp ||
		oldOut != curOutsideTemp || oldPos != valvePos;
}

inline void CreateJSON(json& j)
{
	if (oldRad != curRadTemp)
		j.push_back(json::object_t::value_type("rad_temp", curRadTemp));
	if (oldTar != targetTemp)
		j.push_back(json::object_t::value_type("target_temp", targetTemp));
	if (oldOut != curOutsideTemp)
		j.push_back(json::object_t::value_type("outside_temp", curOutsideTemp));
	if (oldPos != valvePos)
		j.push_back(json::object_t::value_type("valve_pos", valvePos));
}


void SendAndFlushAVG(const struct tm* const timeinfo)
{

	int fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);
	char test[256] = "";

	sprintf(test, "Event,Date,Time\nC_FLUSHAVG,%d/%d/%d,%d:%d\n\n", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min);
	
	struct tm* yesterday;

	yesterday = timeinfo;
	yesterday->tm_mday--;
	mktime(yesterday);

	write(fd, &test, strlen(test));
	close(fd);
	chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);

	double avg = 0.0f;

	fd = open("/home/pi/data/avg.csv", O_WRONLY | O_CREAT | O_APPEND);

	int max = AVG_POS.size() - 1;
	if (max >= 0)
	{
		avg = average(AVG_POS, timeinfo);
		_printf("AVG POS %f\n", avg);

		sprintf(test, "Date,Average Position\n%d/%d/%d,%.2f\n", yesterday->tm_mday, yesterday->tm_mon + 1, yesterday->tm_year + 1900, avg);
		write(fd, &test, strlen(test));

		AVG_POS.clear();
	}

	max = AVG_RAD.size() - 1;
	if (max >= 0)
	{
		avg = average(AVG_RAD, timeinfo);

		sprintf(test, "Date,Average RadiatorTemp\n%d/%d/%d,%.2f\n", yesterday->tm_mday, yesterday->tm_mon + 1, yesterday->tm_year + 1900, avg);
		write(fd, &test, strlen(test));

		_printf("AVG RAD %f\n", avg);
		AVG_RAD.clear();
	}

	max = AVG_OUT.size() - 1;
	if (max >= 0)
	{
		avg = average(AVG_OUT, timeinfo);

		sprintf(test, "Date,Average OutsideTemp\n%d/%d/%d,%.2f\n", yesterday->tm_mday, yesterday->tm_mon + 1, yesterday->tm_year + 1900, avg);
		write(fd, &test, strlen(test));

		_printf("AVG OUT %f\n", avg);
		AVG_OUT.clear();
	}

	max = AVG_TAR.size() - 1;
	if (max >= 0)
	{
		avg = average(AVG_TAR, timeinfo);

		sprintf(test, "Date,Average TargetTemp\n%d/%d/%d,%.2f\n", yesterday->tm_mday, yesterday->tm_mon + 1, yesterday->tm_year + 1900, avg);
		write(fd, &test, strlen(test));

		_printf("AVG TAR %f\n", avg);
		AVG_TAR.clear();
	}

	close(fd);
	chmod("/home/pi/data/avg.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);


	_printf("going to SYNC\n");
	system("./home/pi/gdrive --service-account gdrive-278220-818cb96807d5.json sync upload /home/pi/data 1kGEkztil-ERX-YFK1m_yzD7ZPjr8-VeT");

}

bool second = true;
/*
DistMap distance()
{
	printf("going to dist\n");
	digitalWrite(DIST_TRIGG, HIGH);
	delayMicroseconds(10);
	digitalWrite(DIST_TRIGG, LOW);

	unsigned int now = micros();
	while (digitalRead(DIST_ECHO) == 0 && micros() - now < 30000);
	unsigned int start = micros();
	while (digitalRead(DIST_ECHO) == 1);
	unsigned int stop = micros();

	return DistMap((float)(stop - start) * SONICSPEED);
}
*/

PI_THREAD(UpdateTemperatures)
{
	_printf("inside updatetemp thread\n");
	//float oldTemp = DEVICE_DISCONNECTED_C;
	unsigned int timer = 0;
	unsigned int disconnectCounter = 0;

	while (updateTemperatures)
	{
		/*if (!temp.requestTemperaturesByAddress(OutsideTemp))
		{
			if (!temp.requestTemperaturesByAddress(OutsideTemp))
				_printf("dis out\n");
		}

		if (!temp.requestTemperaturesByAddress(RadiatorTemp))
		{
			if (!temp.requestTemperaturesByAddress(RadiatorTemp))
				_printf("dis rad\n");
		}*/


		temp.requestTemperatures();

		delayMicroseconds(200);
		float _temp = temp.getTempC(RadiatorTemp);
		if (_temp != DEVICE_DISCONNECTED_C && _temp != 85.0f)
		{
			disconnectCounter = 0;
			curRadTemp = _temp;
			/*_temp = temp.getTempC(RadiatorTemp);
			if (_temp != DEVICE_DISCONNECTED_C)
			  curRadTemp = _temp;
			else
				_printf("disonnected rad\n");*/
		}
		else
		{
			//_printf("disconnected rad\n");
			disconnectCounter++;
		}
		/*else
		{
			unsigned int disconnectCounter = 0;
			while (disconnectCounter < 4)
			{
				_temp = temp.getTempC(RadiatorTemp);
				if (_temp != DEVICE_DISCONNECTED_C && _temp != 85.0f)
				{

					curRadTemp = _temp;
					disconnectCounter = 4;
					break;
				}
				else if (disconnectCounter == 3)
				{
					_printf("disconnected scheduling reboot....\n");
					command = C_REBOOT;
					disconnectCounter = 4;
					break;
				}
				else
				{
					disconnectCounter++;
					_printf("disconnected rad\n");
					delayMicroseconds(200);
					temp.requestTemperaturesByAddress(RadiatorTemp);
				}
			}
		}*/
		delayMicroseconds(300);
		_temp = temp.getTempC(OutsideTemp);
		if (_temp != DEVICE_DISCONNECTED_C && _temp != 85.0f)
		{
			disconnectCounter = 0;
			curOutsideTemp = _temp;
			/*
			_temp = temp.getTempC(OutsideTemp);
			if (_temp != DEVICE_DISCONNECTED_C)
				curOutsideTemp = _temp;
			else
				_printf("disconnected outside\n");*/
		}
		else
		{
			//_printf("disconnected outside\n");
			disconnectCounter++;
		}
		if (disconnectCounter >= 8)
			command = C_REBOOT;
		/*else
		{
			unsigned int disconnectCounter = 0;
			while (disconnectCounter < 4)
			{
				_temp = temp.getTempC(OutsideTemp);
				if (_temp != DEVICE_DISCONNECTED_C && _temp != 85.0f)
				{

					curOutsideTemp = _temp;
					disconnectCounter = 4;
					break;
				}
				else if (disconnectCounter == 3)
				{
					_printf("disconnected scheduling reboot....\n");
					command = C_REBOOT;
					disconnectCounter = 4;
					break;
				}
				else
				{
					disconnectCounter++;
					_printf("disconnected outside\n");
					delayMicroseconds(200);
					temp.requestTemperaturesByAddress(OutsideTemp);
				}
			}
		}*/
		//_printf("Rad %.2f Out %.2f\n", curRadTemp, curOutsideTemp);
		bool changed = ValuesChanged();

		if (changed && targetTemp != -127.0f && curOutsideTemp != -127.0f)
		{
			timeval curTime;
			gettimeofday(&curTime, NULL);
			struct tm* timeinfo = localtime(&curTime.tv_sec);
			if (currentday != timeinfo->tm_wday)
			{
				printf("new day?!\n");
				currentday = timeinfo->tm_wday;
				currenthour = timeinfo->tm_hour;

				SendAndFlushAVG(timeinfo);
			}

			if (!AVG_POS.size() || (AVG_POS.back().compare(valvePos) && AVG_POS.back().compare(timeinfo)))
			{
				AVG_POS.push_back(TIMESTAMP(valvePos, timeinfo->tm_hour, timeinfo->tm_min));
			}

			if (!AVG_RAD.size() || (AVG_RAD.back().compare(curRadTemp) && AVG_RAD.back().compare(timeinfo)))
			{
				AVG_RAD.push_back(TIMESTAMP(curRadTemp, timeinfo->tm_hour, timeinfo->tm_min, TIMESTAMP::TYPE::RAD));
			}

			if (!AVG_OUT.size() || (AVG_OUT.back().compare(curOutsideTemp) && AVG_OUT.back().compare(timeinfo)))
			{
				AVG_OUT.push_back(TIMESTAMP(curOutsideTemp, timeinfo->tm_hour, timeinfo->tm_min, TIMESTAMP::TYPE::OUT));
			}

			if (!AVG_TAR.size() || (AVG_TAR.back().compare(targetTemp) && AVG_TAR.back().compare(timeinfo)))
			{
				AVG_TAR.push_back(TIMESTAMP(targetTemp, timeinfo->tm_hour, timeinfo->tm_min, TIMESTAMP::TYPE::TAR));
			}


			if (currenthour != timeinfo->tm_hour)
			{
				currenthour = timeinfo->tm_hour;

				if (AVG_POS.size() != 0)
					_printf("AVG_POS %.2f\n", average(AVG_POS, timeinfo));
				if (AVG_RAD.size() != 0)
					_printf("AVG_RAD %.2f\n", average(AVG_RAD, timeinfo));
				if (AVG_OUT.size() != 0)
					_printf("AVG_OUT %.2f\n", average(AVG_OUT, timeinfo));
				if (AVG_TAR.size() != 0)
					_printf("AVG_TAR %.2f\n", average(AVG_TAR, timeinfo));
			}
		}

		if (connected)
		{
			/*unsigned int dummmy;
			if (read(client, &dummy, 4) == -1)
			{
				printf("disconnected\n");
				connected = false;
				close(client);
				piThreadCreate(CheckForConnections);
			}
			else
			{*/
			json j = {
				{"type", "data"}
			}; /*,
				{"rad_temp", curRadTemp},
				{"boiler_temp", 65},
				{"target_temp", targetTemp},
				{"outside_temp", curOutsideTemp},
				{"valve_pos", valvePos}
				};*/

			if (changed)
			{
				CreateJSON(j);
				oldRad = curRadTemp;
				oldTar = targetTemp;
				oldOut = curOutsideTemp;
				oldPos = valvePos;
			}

			if (timer < UPDATEFREQ)
				timer++;
			else
			{
				timer = 0;
				pellet.distance();
				j.push_back(json::object_t::value_type("dist", pellet.dist));
				j.push_back(json::object_t::value_type("bags", pellet.bags));
			}


			SendJSON(j);
			//}

		}
		//second = !second;
		/*int x;
		x = fcntl(client, F_GETFL, 0);
		fcntl(client, F_SETFL, x | O_NONBLOCK);
		int r = read(client, &command, 1);
		fcntl(client, F_SETFL, x)*/

		//_printf("Scheduling reboot...\n");
		delay(200);
	}

	return NULL;
}


enum moveValveDirect
{
	VALVE_INC,
	VALVE_DEC
};


moveValveDirect ValveDirection;

void increaseRadTemp(unsigned int past)
{
	unsigned int tempValvePosition = valvePosition;
	_printf("Increasing rad temp vPos %d temp pos %d\n", valvePosition, tempValvePosition);
	while (curRadTemp < targetTemp + TEMP_THRESHOLD_MOVING && curStep < MAX_STEP && tempValvePosition < MAX_POS && tempValvePosition < targetPos + TARGET_THRESHOLD)
	{
		//_printf("curRad %d is samall\n", curStep);
		curStep++;

		delay(800);
		tempValvePosition = valvePosition + GetTimePassed(past);
		valvePos = tempValvePosition;
		_printf("T %.2f R %.2f valvePosition %d\n", targetTemp, curRadTemp, tempValvePosition);
		//curRadTemp = getRadTemp();
	}

}

void decreaseRadTemp(unsigned int past)
{
	unsigned int tempValvePosition = valvePosition;
	_printf("Decreasing rad temp vPos %d temp pos %d\n", valvePosition, tempValvePosition);
	while (curRadTemp >= targetTemp - 0.10f && curStep < MAX_STEP && tempValvePosition > 0)
	{
		//_printf("curRad %d is samall\n", curStep);
		curStep++;
		//_printf("Now it's time %.2f\n", curRadTemp);
		delay(2000);
		tempValvePosition = valvePosition - GetTimePassed(past);
		valvePos = tempValvePosition;
		_printf("T %.2f R %.2f valvePosition %d\n", targetTemp, curRadTemp, tempValvePosition);
		//curRadTemp = getRadTemp();
	}
}

bool moveValveThreadLaunched = false;


PI_THREAD(MoveValve)
{
	decreasing = ValveDirection == VALVE_DEC;
	moveValveThreadLaunched = true;
	curStep = 0;
	_printf("inside thread\n");
	delayMicroseconds(10);
	/*if (ValveDirection == VALVE_INC)
		digitalWrite(MOTOR_DEC, LOW);
	else
		digitalWrite(MOTOR_INC, LOW);
	delayMicroseconds(10);*/
	if (ValveDirection == VALVE_INC)
		digitalWrite(MOTOR_INC, HIGH);
	else
		digitalWrite(MOTOR_DEC, HIGH);
	reallyMovingValve = true;
	unsigned int past = millis();
	delay(500);



	float difference = ((targetTemp - curRadTemp));
	if (difference < 0.0f)
		difference *= -1.0f;

	MAX_STEP = difference;
	MAX_STEP += 2;
	if (MAX_STEP > MAX_MAX_STEP)
		MAX_STEP = MAX_MAX_STEP;
	if (ValveDirection == VALVE_DEC)
		MAX_STEP += 2;

	if (ValveDirection == VALVE_INC)
		increaseRadTemp(past);
	else
		decreaseRadTemp(past);

	if (ValveDirection == VALVE_INC)
	{
		_printf("turning off INC\n");
		digitalWrite(MOTOR_INC, LOW);
		reallyMovingValve = false;
		valveTimer += GetTimePassed(past);
	}
	else
	{
		_printf("turning off DEC\n");
		digitalWrite(MOTOR_DEC, LOW);
		reallyMovingValve = false;
		valveTimer -= GetTimePassed(past);
	}

	if (curStep < MAX_STEP && valvePosition > 0 && valvePosition < MAX_POS)
	{
		_printf("target temp set in thread\n");
		/*MAX_STEP /= 10;
		if (MAX_STEP == 0)
			MAX_STEP = 1;*/
		if (ValveDirection == VALVE_DEC)
			MAX_STEP -= 2;
		_delay(TARGET_TEMP_WAITTIME * MAX_STEP);
	}
	else
	{

		_printf("max is set\n");

		if (curStep == MAX_STEP)
		{
			/*MAX_STEP /= 10;
			if (MAX_STEP == 0)
				MAX_STEP = 1;*/
			if (ValveDirection == VALVE_DEC)
				MAX_STEP -= 2;
			_delay(MAX_STEP_WAITTIME * MAX_STEP);
		}
	}
	if (curRadTemp >= targetTemp - 0.06f)
		targetSet = true;
	/*else if (curRadTemp <= targetTemp - TEMP_THRESHOLD_MOVING)
		targetSet = true;*/
	movingValve = false;
	moveValveThreadLaunched = false;
	return NULL;
}

void moveValve()
{

	_printf("Not increasing\n");

	int x = piThreadCreate(MoveValve);
	if (x != 0)
		PRINT_THREADERROR();

}

void ServicePump()
{
	StartPump();
	delay(60000);
	StopPump();

}

unsigned int summerStart = 0;

void ServiceValve()
{
	movingValve = true;
	decreasing = false;
	valvePosition = 0;
	digitalWrite(MOTOR_DEC, LOW);
	delayMicroseconds(1000);
	digitalWrite(MOTOR_INC, HIGH);
	summerStart = millis();
	_delay(MINUTES(2));
	digitalWrite(MOTOR_INC, LOW);
	delayMicroseconds(1000);
	digitalWrite(MOTOR_DEC, HIGH);
	summerStart = millis();
	decreasing = true;
	_delay(MINUTES(2));
	digitalWrite(MOTOR_DEC, LOW);
	movingValve = false;
	valvePosition = 0;
	decreasing = false;
}


PI_THREAD(CheckForService)
{
	_printf("SummerModeThread\n");
	timeval curTime;
	gettimeofday(&curTime, NULL);
	struct tm* timeinfo = localtime(&curTime.tv_sec);
	int fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);
	char test[256] = "";

	sprintf(test, "Event,Date,Time\nC_ENTERSUMMER,%d/%d/%d,%d:%d\n\n", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min);
	write(fd, &test, strlen(test));
	close(fd);
	chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);

	digitalWrite(MOTOR_INC, LOW);
	digitalWrite(MOTOR_DEC, HIGH);
	_delay(5000);
	digitalWrite(MOTOR_DEC, LOW);

	while (curOutsideTemp >= SUMMERTEMP && summerMode)
	{
		if (days != 8)
		{
			unsigned long int timeLapsed = millis();

			if (summer_reboot)
			{
				timeLapsed += summer_reboot;
			}
			else if (timeLapsed < summerStart)
				timeLapsed = 0xFFFFFFFF - summerStart + timeLapsed;
			else
				timeLapsed -= summerStart;
			if (timeLapsed >= HOURS(24))
			{
				days++;
				summerStart = millis();
				summer_reboot = 0;
			}
		}
		else
		{
			_printf("going to service\n");
			ServicePump();
			ServiceValve();
			summerStart = millis();
			days = 1;
			summer_reboot = 0;
		}
	}

	summer_reboot = 0;
	summerMode = false;
	StartPump();

	fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);

	sprintf(test, "Event,Date,Time\nC_LEAVESUMMER,%d/%d/%d,%d:%d\n\n", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min);
	write(fd, &test, strlen(test));
	close(fd);
	chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);
	return NULL;
}

void EnterSummerMode()
{
	_printf("EnteringSummerMode\n");
	StopPump();
	summerMode = true;
	summerStart = millis();
	days = 1;
	int x = piThreadCreate(CheckForService);
	if (x != 0)
		PRINT_THREADERROR();
}

static const char* const BOOL[] = { "FALSE", "TRUE" };

void LeaveSummerMode()
{
	summerMode = false;
}

float oldTemp = DEVICE_DISCONNECTED_C;
unsigned char times = 0;

void MainLoop()
{
	times = 0;
	while (command == C_CONTINUE)
	{
		////printf("WTF?\n");


		/*if (setVal)
			printf("true");
		else
			printf("false");*/
		if (curOutsideTemp < SUMMERTEMP)
		{
			bool setVal = insideSetValvePos;
			//piLock(0);

			//piUnlock(0);
			if (!setVal && (OutsideTempChanged(oldTemp) || force))
			{
				if (UpdateTarget(true) || force)
				{
					force = false;
					setVal = true;
					int x = piThreadCreate(SetValvePosThread);
					if (x != 0)
						PRINT_THREADERROR();

					while (!insideSetValvePos) printf("WTF?!\n");
				}
				oldTemp = curOutsideTemp;
				oldTempTrue = oldTemp;
			}


			if (!movingValve && !moveValveThreadLaunched && targetTemp != DEVICE_DISCONNECTED_C && !setVal)// && !summerMode)
			{
				//printf("Ok...\n");
				//StartPump();
				//curOutsideTemp = getOutsideTemp();
				if (curRadTemp < (targetTemp <= 30.0f ? (targetTemp - TEMP_THRESHOLD * 2) : (targetTemp - TEMP_THRESHOLD)))
				{
					//printf("currad smaller\n");
					summerMode = false;
					targetSet = false;

					if (valvePosition < MAX_POS && valvePosition < targetPos + TARGET_THRESHOLD)
					{
						_printf("going to increase\n");
						ValveDirection = VALVE_INC;
						movingValve = true;
						moveValve();
					}
					else
					{
						_printf("valvePosition %d bigger than MAX %d\n", valvePosition, targetPos + TARGET_THRESHOLD);
						if (!movingValve)
						{
							_printf("T %.2f R %.2f O %.2f\n", targetTemp, curRadTemp, curOutsideTemp);
						}
					}
				}
				else if (curRadTemp >= targetTemp + 0.1f)
				{
					//printf("currad bigger\n");
					targetSet = true;

					if (valvePosition > 0)
					{

						/*if (valvePosition > targetPos - TARGET_THRESHOLD2)
						{*/
						_printf("going to decrease\n");
						ValveDirection = VALVE_DEC;
						movingValve = true;
						moveValve();
					}
					else
					{
						//curRadTemp = getRadTemp();
						if (valvePosition < 0)
						{
							valvePosition = 0;
							valvePos = 0;
						}

						if (curOutsideTemp > 6)
						{
							StopPump();
						}
						else if (!summerMode)
						{

							if (!force)
							{
								force = true;
							}
						}
						_printf("T %.2f R %.2f O %.2f\n", targetTemp, curRadTemp, curOutsideTemp);
						/*else
							_printf("T %.2f R %.2f O %.2f waiting for service... %.2f%%\n ", targetTemp, curRadTemp, curOutsideTemp, _map(GetTimePassed(summerStart) * days, 0.0f, TWENTYFOURHOURS * 7, 0.0f, 100.0f));
							*/

					}
				}
				else
				{
					//printf("currad equal\n");
					//curRadTemp = getRadTemp();
					if (!targetSet)
					{
						summerMode = false;
						if (valvePosition < MAX_POS && valvePosition < targetPos + TARGET_THRESHOLD)
						{
							_printf("going to increase\n");
							ValveDirection = VALVE_INC;
							movingValve = true;
							moveValve();
						}
						else
						{
							_printf("valvePosition %d bigger than MAX %d\n", valvePosition, targetPos + TARGET_THRESHOLD);
							if (!movingValve)
							{
								_printf("T %.2f R %.2f O %.2f\n", targetTemp, curRadTemp, curOutsideTemp);
							}
						}
					}
					else
					{
						StartPump();
						_printf("Target set T %.2f R %.2f valvePosition %d O %.2f\n", targetTemp, curRadTemp, valvePosition, curOutsideTemp);
					}
					//curRadTemp = getRadTemp();
				}
			}
			else
			{
				//printf("or???\n");
				/*_printf("moving %s thread %s pos %d\n", BOOL [movingValve], BOOL [insideSetValvePos], valvePosition);
					_printf("Rad %.2f Pod %d Out %.2f\n", curRadTemp, valvePosition, curOutsideTemp);*/
				bool waiting = false;
				for (unsigned int i = 0; i < sizeof(waitTimer) / sizeof(WaitTimer); i++)
				{
					if (waitTimer[i].waiting)
						waiting = true;
					waitTimer[i].Print();
				}
				if (!waiting && !movingValve)
				{
					_printf("not mov thread %s valvePosition %d\n", BOOL[setVal], valvePosition);
					_printf("T %.2f R %.2f O %.2f\n", targetTemp, curRadTemp, curOutsideTemp);
					if (times == 15)
						command = C_REBOOT;
					times++;
					/*command = C_REBOOT;
					return;*/
				}
			}
		}
		else if (!summerMode)
		{
			while (reallyMovingValve || insideSetValvePos) printf("omg?!");

			if (UpdateTarget(true) || force)
			{
				force = false;
				delay(1000);
			}
			oldTemp = curOutsideTemp;
			oldTempTrue = oldTemp;

			if (valvePosition != 0)
			{
				digitalWrite(MOTOR_DEC, HIGH);
				delay(valvePosition);
				digitalWrite(MOTOR_DEC, LOW);
			}

			targetTemp = 0.0f;
			EnterSummerMode();
		}
		else
			_printf("T %.2f R %.2f O %.2f waiting for service... %.2f%%\n ", targetTemp, curRadTemp, curOutsideTemp, _map((float)(GetTimePassed(summerStart) * days), 0.0f, HOURS(24) * 7, 0.0f, 100.0f));
		/*else if (summerMode)
		{
			if (movingValve)
			{
				//curRadTemp = getRadTemp();
				if (!decreasing)
					valvePosition = millis() - summerStart;
				else
					valvePosition = -(millis() - summerStart) + TWOMINUTES;
				//curRadTemp = getRadTemp();
				_printf("valvePosition %d R %.2f\n", valvePosition, curRadTemp);
			}
			else
			{
				curRadTemp = getRadTemp(temp);
				_printf("Too high temp %.2f, waiting for service... %.2f%%\n ", curRadTemp, map((float)(millis()-summerStart), 0.0f, TWENTYTWOHOURS, 0.0f, 100.0f));
			}*/
			//}



		if (connected)
		{
			int r = recv(client, &command, 1, MSG_DONTWAIT);
			if (r == 1 && command == C_REBOOT)
				_printf("Scheduling reboot...\n");
		}
		//printf("ending\n");
		delay(3000);
	}
}

int main(int argc, char* argv[])
{
	std::atexit(Cleanup);
	auto lam =
		[](int i) { cout << "aborting" << endl; exit(-1); };
	signal(SIGINT, lam);
	//abort()
	signal(SIGABRT, lam);
	//sent by "kill" command
	signal(SIGTERM, lam);
	//^Z
	signal(SIGTSTP, lam);
	delay(100);
	while (wiringPiSetupPhys())
		_printf("failed\n");

	pinMode(MOTOR_INC, OUTPUT);
	delayMicroseconds(10);
	pinMode(MOTOR_DEC, OUTPUT);
	delayMicroseconds(10);
	pinMode(DIST_TRIGG, OUTPUT);
	delayMicroseconds(10);
	pinMode(DIST_ECHO, INPUT);
	digitalWrite(DIST_TRIGG, LOW);
	delayMicroseconds(500);
	digitalWrite(MOTOR_INC, LOW);
	delayMicroseconds(100);
	if (argc <= 1)
	{
		printf("going to wait 4 minutes...\n");
		digitalWrite(MOTOR_DEC, HIGH);
		delay(MINUTES(4));
	}
	else
	{
		_printf("argc %d\n", argc);

		int pos = atoi(argv[1]);
		_printf("argv[1] %d\n", pos);
		valvePosition = pos;
		valvePos = valvePosition;
	}
	digitalWrite(MOTOR_DEC, LOW);
	delayMicroseconds(100);
	/*digitalWrite(MOTOR_INC, HIGH);
	delay(3243);
	valvePosition = 3243;
	digitalWrite(MOTOR_INC, LOW);*/
	/*delay(5000);
	digitalWrite(MOTOR_DEC, HIGH);*/
	_printf("turning on or off?\n");
	OneWire oneWire(ONE_WIRE);
	cleanupWire = &oneWire;

	delay(100);

	temp.setOneWire(&oneWire); //DallasTemperature temp(&oneWire);
	delay(1000);
	temp.begin();
	delay(1000);
	/*piThreadCreate(requestTemps);
	delayMicroseconds(10);*/
	/*_printf("OutsideTemp valid %s\n", BOOL[temp.validAddress(OutsideTemp)]);
	_printf("RadiatorTemp valid %s\n", BOOL[temp.validAddress(RadiatorTemp)]);
	delay(5000);*/
	while (temp.getDeviceCount() != 2)
	{

		_printf("not dev count %d\n", temp.getDeviceCount());
		temp.begin();
		/*_printf("OutsideTemp exists %s\n", BOOL [temp.getAddress((uint8_t*)OutsideTemp, 2)]);
		_printf("RadiatorTemp exists %s\n", BOOL[temp.getAddress((uint8_t*)RadiatorTemp, 2)]);*/
		delay(500);
	}

	temp.setCheckForConversion(false);
	//curRadTemp = getRadTemp();
	/*unsigned char addr[8];
	if(temp.getAddress(addr , 0))
	  _printf(" addr 1 %X %X %X %X %X %X %X %X\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5],addr[6],addr[7]);
	if(temp.getAddress(addr , 1))
	  _printf("addr 2 %X %X %X %X %X %X %X %X\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5],addr[6],addr[7]);
	*/


	delay(1000);
	//curRadTemp = getRadTemp();
	delay(500);
	temp.setResolution(12);
	_printf("RES %d\n parasite %d\n", temp.getResolution(), temp.isParasitePowerMode());
	delay(1500);
	temp.requestTemperatures();
	temp.getTempC(OutsideTemp);
	temp.requestTemperatures();
	temp.getTempC(OutsideTemp);
	temp.requestTemperatures();
	temp.getTempC(OutsideTemp);
	delay(500);

	timeval curTime;
	gettimeofday(&curTime, NULL);
	struct tm* timeinfo = localtime(&curTime.tv_sec);

	currentday = timeinfo->tm_wday;
	currenthour = timeinfo->tm_hour;

	int fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);
	char test[256] = "";
	sprintf(test, "Event,Date,Time\nC_STARTUP,%d/%d/%d,%d:%d\n\n", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min);
	write(fd, &test, strlen(test));
	close(fd);
	chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);


	system("./home/pi/gdrive --service-account gdrive-278220-818cb96807d5.json sync upload /home/pi/data 1kGEkztil-ERX-YFK1m_yzD7ZPjr8-VeT");


	fd = open("avg.inc", O_RDONLY);
	unsigned int s = 0;

	if (!(fd | ENOENT))
	{

		read(fd, &s, 4);
		if (s)
		{
			AVG_POS.resize(s);
			read(fd, &AVG_POS.front(), sizeof(TIMESTAMP) * s);
		}

		read(fd, &s, 4);
		if (s)
		{
			AVG_RAD.resize(s);
			read(fd, &AVG_RAD.front(), sizeof(TIMESTAMP) * s);
		}

		read(fd, &s, 4);
		if (s)
		{
			AVG_OUT.resize(s);
			read(fd, &AVG_OUT.front(), sizeof(TIMESTAMP) * s);
		}

		read(fd, &s, 4);
		if (s)
		{
			AVG_TAR.resize(s);
			read(fd, &AVG_TAR.front(), sizeof(TIMESTAMP) * s);
		}

		close(fd);
	}

	pthread_t thread_temp, thread_conn;
	pthread_attr_t thread_attr;
	sched_param sp;
	memset(&sp, 0, sizeof(sp));
	pthread_attr_init(&thread_attr);

	pthread_attr_setschedpolicy(&thread_attr, SCHED_FIFO);
	pthread_attr_setschedparam(&thread_attr, &sp); // Set attributes to priority 30
	pthread_create(&thread_temp, &thread_attr, UpdateTemperatures, NULL);

	pthread_attr_setschedpolicy(&thread_attr, SCHED_IDLE);
	pthread_attr_setschedparam(&thread_attr, &sp);
	pthread_create(&thread_conn, &thread_attr, CheckForConnections, NULL);
	pthread_attr_destroy(&thread_attr);


	//piThreadCreate(UpdateTemperatures);


	//piThreadCreate(CheckForConnections);
	//sched_setscheduler(0, SCHED_OTHER, &sp);
	delay(500);


	//while (curRadTemp == DEVICE_DISCONNECTED_C || curRadTemp == 85.0f);

	if (argc > 3)
	{
		summer_reboot = atoi(argv[2]);
		days = atoi(argv[3]);


		if (UpdateTarget(true) || force)
		{
			force = false;
		}
		oldTemp = curOutsideTemp;
		oldTempTrue = oldTemp;
		summerMode = true;
		StopPump();
		targetTemp = 0.0f;
		int x = piThreadCreate(CheckForService);
		if (x != 0)
			PRINT_THREADERROR();
	}
	delay(2000);

LOOP:
	MainLoop();

	bool waiting = false;
	unsigned timeWaited = 0;

	switch (command)
	{
	case C_REBOOT:
		for (unsigned int i = 0; i < sizeof(waitTimer) / sizeof(WaitTimer); i++)
		{
			if (waitTimer[i].waiting)
				waiting = true;
		}

		while ((insideSetValvePos || waiting) && timeWaited < MINUTES(4))
		{
			waiting = false;
			for (unsigned int i = 0; i < sizeof(waitTimer) / sizeof(WaitTimer); i++)
			{
				if (waitTimer[i].waiting)
					waiting = true;
				waitTimer[i].Print();
			}
			if (!waiting)
				_printf("waiting...\n");
			delay(3000);
			timeWaited += 3000;
		}
		_printf("going to reboot...\n\n");
		delay(1000);
		Cleanup();



		gettimeofday(&curTime, NULL);
		timeinfo = localtime(&curTime.tv_sec);

		fd = open("/home/pi/data/test.csv", O_WRONLY | O_CREAT | O_APPEND);

		sprintf(test, "Event,Date,Time\nC_REBOOT,%d/%d/%d,%d:%d\n\n", timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min);
		write(fd, &test, strlen(test));
		close(fd);
		chmod("/home/pi/data/test.csv", S_IRWXU | S_IRWXG | S_IROTH | S_IWOTH);


		if (summerMode)
		{
			valvePos &= 0x100000;
			fd = open("extras.inc", O_WRONLY | O_CREAT);
			ExtraInfo inf;
			inf.summerStart = millis() + summer_reboot;
			inf.days = days;
			write(fd, &inf, sizeof(ExtraInfo));
			close(fd);
		}

		fd = open("avg.inc", O_WRONLY | O_CREAT);
		s = AVG_POS.size();

		write(fd, &s, 4);
		if (s)
		{
			write(fd, &AVG_POS.front(), sizeof(TIMESTAMP) * s);
		}

		s = AVG_RAD.size();
		write(fd, &s, 4);
		if (s)
		{
			write(fd, &AVG_RAD.front(), sizeof(TIMESTAMP) * s);
		}

		s = AVG_OUT.size();
		write(fd, &s, 4);
		if (s)
		{
			write(fd, &AVG_OUT.front(), sizeof(TIMESTAMP) * s);
		}

		s = AVG_TAR.size();
		write(fd, &s, 4);
		if (s)
		{
			write(fd, &AVG_TAR.front(), sizeof(TIMESTAMP) * s);
		}

		close(fd);

		if (valvePos == 35584)
			valvePos = 35585;
		else if (valvePos == 2)
			valvePos = 3;
		return valvePos;

	case C_EXIT:
		break;

	default:
		goto LOOP;
	}
	printf("EXIIIIT\n\n");
	return -1;
}