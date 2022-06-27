#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <vector>
//#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

#include <windows.h>

#include "serial.h"

using namespace std;
using boost::thread;
using boost::mutex;

mutex serial_lock;

union float2byte
{
	float f;
	char c[4];
};

//send target velocity & bridge state via wireless serial
int send_data_n(double* vel_arr, SerialPort* serial)
{
	//package head
	string str_sent = "s";

	//v1, v2, v3, v4, coil1, coil2
	for (int s = 0; s < 6; s++)
	{
		stringstream ss_tmp;
		ss_tmp << fixed << setprecision(1) << vel_arr[s];
		string num_tmp = ss_tmp.str();

		str_sent = str_sent + num_tmp + "\n";
	}

	cout << str_sent << endl;

	int w_state = 0;
	
	if (serial->isConnected()) 
	{
		char* char_arr;
		char_arr = &str_sent[0];

		w_state = serial->writeSerialPort(char_arr, str_sent.length());
	}

	return w_state;
}

//serial thread
void serial_fun(double* vel_arr, SerialPort* serial)
{
	while (1)
	{
		serial_lock.lock();
		int w_state = send_data_n(vel_arr, serial);
		cout << "serial write - " << 1 << endl;

		serial_lock.unlock();
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	}
}


int main() 
{
	//wheel velocity in rad/s, bridge heating coil state 1-heating 0-sleep
	//v1, v2, v3, v4, coil1, coil2
	double mb_ctr_arr[6] = { 0,0,0,0,0,0 };

	//port ID of wireless serial can be found in device manager
	char* port_name = "\\\\.\\COM4";

	SerialPort* mb_serial = new SerialPort(port_name);
	//start serial thread
	thread *serial_thread = new thread(serial_fun, mb_ctr_arr, mb_serial);

	//wheel motion test
	serial_lock.lock();
	mb_ctr_arr[0] = 4.5;
	mb_ctr_arr[2] = -4.5;
	cout << "whell a1 b1 in motion " << 1 << endl;
	serial_lock.unlock();
	boost::this_thread::sleep_for(boost::chrono::seconds(3));
	
	serial_lock.lock();
	mb_ctr_arr[0] = 0;
	mb_ctr_arr[2] = 0;
	mb_ctr_arr[1] = 4.5;
	mb_ctr_arr[3] = -4.5;
	cout << "whell a2 b2 in motion " << 1 << endl;
	serial_lock.unlock();
	boost::this_thread::sleep_for(boost::chrono::seconds(3));

	//bridge heating test
	serial_lock.lock();
	mb_ctr_arr[1] = 0;
	mb_ctr_arr[3] = 0;
	mb_ctr_arr[4] = 1;
	mb_ctr_arr[5] = 1;
	cout << "bridge sge1 seg2 heating up " << 1 << endl;
	serial_lock.unlock();
	boost::this_thread::sleep_for(boost::chrono::seconds(30));

	serial_lock.lock();
	mb_ctr_arr[4] = 0;
	mb_ctr_arr[5] = 0;
	serial_lock.unlock();
	boost::this_thread::sleep_for(boost::chrono::seconds(5));

	//other controller can be deployed in main thread

	//clean up
	serial_thread->join();
	delete serial_thread;
	delete mb_serial;

	return 0;
}