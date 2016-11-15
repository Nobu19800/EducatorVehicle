// -*- C++ -*-
/*!
 * @file  controlEV3.cpp
 * @brief レゴ　マインドストーム EV3のEducator Vehicle操作基本クラス
 * @date $Date$
 *
 * @author 宮本信彦
 * n-miyamoto@aist.go.jp
 * 産業技術総合研究所　
 * ロボットイノベーション研究センター　
 * ロボットソフトウェアプラットフォーム研究チーム
 *
 * LGPL
 *
 * $Id$
 */

#include <fstream>
#include <iostream>
#include <string.h>
#include "controlEV3.h"


/**
* @brief コンストラクタ
*/
controlEV3::controlEV3()
{
	right_touch_sensor = NULL;
	left_touch_sensor = NULL;
	color_sensor = NULL;
	ultrasonic_sensor = NULL;
	gyro_sensor = NULL;
	left_large_motor = NULL;
	right_large_motor = NULL;
	medium_motor = NULL;

	gettimeofday(&prev_clock, NULL);
	gettimeofday(&current_clock, NULL);
	diff_time = 0;
	m_vx = 0;
	m_vy = 0;
	m_va = 0;
	m_px = 0;
	m_py = 0;
	m_a = 0;
}

/**
* @brief デストラクタ
*/
controlEV3::~controlEV3()
{
	if(right_touch_sensor)delete right_touch_sensor;
	if(left_touch_sensor)delete left_touch_sensor;
	if(color_sensor)delete color_sensor;
	if(ultrasonic_sensor)delete ultrasonic_sensor;
	if(gyro_sensor)delete gyro_sensor;
	if(left_large_motor)delete left_large_motor;
	if(right_large_motor)delete right_large_motor;
	if(medium_motor)delete medium_motor;
}

/**
* @brief 初期化
* @param wheelRadius 車輪の直径
* @param wheels_distance 車輪間の距離
* @param px 初期位置X
* @param py 初期位置Y
* @param a 初期姿勢
* @return true
*/
bool controlEV3::init(float wheelRadius, float wheelDistance, float px, float py, float a)
{
	set_param(wheelRadius, wheelDistance);
	reset_position(px, py, a);
	if(right_touch_sensor)delete right_touch_sensor;
	if(left_touch_sensor)delete left_touch_sensor;
	if(color_sensor)delete color_sensor;
	if(ultrasonic_sensor)delete ultrasonic_sensor;
	if(gyro_sensor)delete gyro_sensor;
	if(left_large_motor)delete left_large_motor;
	if(right_large_motor)delete right_large_motor;
	if(medium_motor)delete medium_motor;

	right_touch_sensor = new ev3dev::touch_sensor(RIGHT_TOUCH_SENSOR_ADDRESS);
	if(!right_touch_sensor->connected())
	{
		delete right_touch_sensor;
		right_touch_sensor = new ev3dev::touch_sensor();
		
	}

	left_touch_sensor = new ev3dev::touch_sensor(LEFT_TOUCH_SENSOR_ADDRESS);


	color_sensor = new ev3dev::color_sensor();


	ultrasonic_sensor = new ev3dev::ultrasonic_sensor();


	gyro_sensor = new ev3dev::gyro_sensor();


	left_large_motor = new ev3dev::large_motor(LEFT_LARGE_MOTOR_ADDRESS);
	if(left_large_motor->connected())
	{
		left_large_motor->set_stop_command(ev3dev::motor::stop_command_hold);
	}
	right_large_motor = new ev3dev::large_motor(RIGHT_LARGE_MOTOR_ADDRESS);
	if(right_large_motor->connected())
	{
		right_large_motor->set_stop_command(ev3dev::motor::stop_command_hold);
	}
	medium_motor = new ev3dev::medium_motor();
	if(medium_motor->connected())
	{
		medium_motor->set_stop_command(ev3dev::motor::stop_command_hold);
	}
	return true;
}

/**
* @brief 右タッチセンサのオンオフを取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return タッチセンサがオンの場合はtrue、オフの場合はfalse
*/
bool controlEV3::right_touch_sensor_is_pressed(bool &ret)
{
	if(right_touch_sensor->connected())
	{
		ret = true;
		return right_touch_sensor->is_pressed();
	}
	ret = false;
	return false;
}

/**
* @brief 左タッチセンサのオンオフを取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return タッチセンサがオンの場合はtrue、オフの場合はfalse
*/
bool controlEV3::left_touch_sensor_is_pressed(bool &ret)
{
	if(left_touch_sensor->connected())
	{
		ret = true;
		return left_touch_sensor->is_pressed();
	}
	ret = false;
	return false;
}

/**
* @brief カラーセンサで色を取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 計測した色
*/
std::string controlEV3::get_color_sensor(bool &ret)
{
	if(color_sensor->connected())
	{
		ret = true;
		int color = color_sensor->color();
		if(color == 0)
		{
			return "None";
		}
		else if(color == 1)
		{
			return "Black";
		}
		else if(color == 2)
		{
			return "Blue";
		}
		else if(color == 3)
		{
			return "Green";
		}
		else if(color == 4)
		{
			return "Yellow";
		}
		else if(color == 5)
		{
			return "RED";
		}
		else if(color == 6)
		{
			return "White";
		}
		else if(color == 7)
		{
			return "Brown";
		}
		
	}
	ret = false;
	return "ERROR";
}

/**
* @brief カラーセンサで反射光の強さを取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 反射光の強さ
*/
float controlEV3::get_light_reflect(bool &ret)
{
	if(color_sensor->connected())
	{
		ret = true;
		return (float)color_sensor->reflected_light_intensity();
	}
	ret = false;
	return 0;
}

/**
* @brief ジャイロセンサで角度を取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 角度
*/
float controlEV3::get_angle_gyro_sensor(bool &ret)
{
	if(gyro_sensor->connected())
	{
		ret = true;
		float data = (float)gyro_sensor->angle();
		return data/180*M_PI;
	}
	ret = false;
	return 0;
}

/**
* @brief 超音波センサで距離を取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 距離
*/
float controlEV3::get_distance_ultrasonic_sensor(bool &ret)
{
	if(ultrasonic_sensor->connected())
	{
		ret = true;
		float data = ultrasonic_sensor->distance_centimeters();
		return data/100.0;
	}
	ret = false;
	return -1;
}

/**
* @brief モーターの速度設定
* @param m モーター
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_speed_motor(ev3dev::motor *m, float v, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		
		m->set_speed_regulation_enabled("on");
		m->set_speed_sp(rot_to_count(v, m, ret));
		m->run_forever();
		return;
	}
	ret = false;
}

/**
* @brief モーターの位置設定
* @param m モーター
* @param p 位置
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_position_motor(ev3dev::motor *m, float p, float v, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		m->set_speed_regulation_enabled("on");
		m->set_speed_sp(rot_to_count(v, m, ret));
		m->set_position_sp(p*180/M_PI);
		m->run_to_abs_pos();
		return;
	}
	ret = false;
}

/**
* @brief モーターの速度取得
* @param m モーター
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 速度
*/
float controlEV3::get_speed_motor(ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return count_to_rot(m->speed(), m, ret);
	}
	ret = false;
	return 0;
}

/**
* @brief モーターの位置取得
* @param m モーター
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 位置
*/
float controlEV3::get_position_motor(ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return (float)m->position()*M_PI/180.0;
	}
	ret = false;
	return 0;
}

/**
* @brief 左モーターの速度設定
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_speed_left_large_motor(float v, bool &ret)
{
	set_speed_motor(left_large_motor, v, ret);
}

/**
* @brief 右モーターの速度設定
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_speed_right_large_motor(float v, bool &ret)
{
	set_speed_motor(right_large_motor, v, ret);
}

/**
* @brief 左モーターの位置設定
* @param p 位置
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_position_left_large_motor(float p, float v, bool &ret)
{
	set_position_motor(left_large_motor, p, v, ret);
}

/**
* @brief 右モーターの位置設定
* @param p 位置
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_position_right_large_motor(float p, float v, bool &ret)
{
	set_position_motor(right_large_motor, p, v, ret);
}

/**
* @brief 左モーターの速度取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 速度
*/
float controlEV3::get_speed_left_large_motor(bool &ret)
{
	return get_speed_motor(left_large_motor, ret);
}

/**
* @brief 右モーターの速度取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 速度
*/
float controlEV3::get_speed_right_large_motor(bool &ret)
{
	return get_speed_motor(right_large_motor, ret);
}

/**
* @brief 右モーターの位置取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 位置
*/
float controlEV3::get_position_left_large_motor(bool &ret)
{
	return get_position_motor(left_large_motor, ret);
}

/**
* @brief 左モーターの位置取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 位置
*/
float controlEV3::get_position_right_large_motor(bool &ret)
{
	return get_position_motor(right_large_motor, ret);
}


/**
* @brief 左モーターをリセット
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::reset_left_large_motor(bool &ret)
{
	if(left_large_motor->connected())
	{
		ret = true;
		left_large_motor->reset();
		return;
	}
	ret = false;
}

/**
* @brief 右モーターをリセット
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::reset_right_large_motor(bool &ret)
{
	if(right_large_motor->connected())
	{
		ret = true;
		right_large_motor->reset();
		return;
	}
	ret = false;
}

/**
* @brief 左モーターを停止
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::stop_left_large_motor(bool &ret)
{
	if(left_large_motor->connected())
	{
		ret = true;
		left_large_motor->stop();
		return;
	}
	ret = false;
}

/**
* @brief 右モーターを停止
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::stop_right_large_motor(bool &ret)
{
	if(right_large_motor->connected())
	{
		ret = true;
		right_large_motor->stop();
		return;
	}
	ret = false;
}


/**
* @brief Mモーターの速度設定
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_speed_medium_motor(float v, bool &ret)
{
	set_speed_motor(medium_motor, v, ret);
}

/**
* @brief Mモーターの位置設定
* @param p 位置
* @param v 速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_position_medium_motor(float p, float v, bool &ret)
{
	set_position_motor(medium_motor, p, v, ret);
}

/**
* @brief Mモーターの速度取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 速度
*/
float controlEV3::get_speed_medium_motor(bool &ret)
{
	return get_speed_motor(medium_motor, ret);
}

/**
* @brief Mモーターの位置取得
* @param ret 取得できた場合はtrue、失敗した場合はfalse
* @return 位置
*/
float controlEV3::get_position_medium_motor(bool &ret)
{
	return get_position_motor(medium_motor, ret);
}

/**
* @brief Mモーターをリセット
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::reset_medium_motor(bool &ret)
{
	if(medium_motor->connected())
	{
		ret = true;
		medium_motor->reset();
		return;
	}
	ret = false;
}

/**
* @brief Mモーターを停止
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::stop_medium_motor(bool &ret)
{
	if(medium_motor->connected())
	{
		ret = true;
		medium_motor->stop();
		return;
	}
	ret = false;
}


/**
* @brief LCDの操作
* @param file_name 表示する画像データ
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_image_lcd_filename(std::string file_name, bool &ret)
{
	ev3dev::lcd lcd = ev3dev::lcd();


	unsigned char *fb = lcd.frame_buffer();
	int buf_size = lcd.frame_buffer_size();
	//int width = lcd.line_length();
	//int height = buf_size/bits_per_pixel/width;
	
	if(lcd.bits_per_pixel() == 1)
	{
		std::ifstream ifs(file_name.c_str(), std::ios::in | std::ios::binary);
		if (!ifs)
		{
			ret = false;
			return;
		}
		ifs.seekg(0, std::fstream::end);
		int eofPos = ifs.tellg();
		ifs.clear();
		ifs.seekg(0, std::fstream::beg);
		int begPos = ifs.tellg();
		int file_size = eofPos - begPos;
		if(file_size < buf_size)
		{
			ifs.close();
			ret = false;
			return;
		}
		//char *temp_file_data = new char[file_size];
		//ifs.read(temp_file_data, file_size);
		//memcpy(&fb[0], &temp_file_data[0], sizeof(unsigned char)*buf_size);
		//delete temp_file_data;
		ifs.read((char*)&fb[0], file_size);
		ifs.close();

		ret = true;
	}
	
	else
	{
		ret = false;
	}
}

/**
* @brief LCDの操作
* @param data 表示する画像データ
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_image_lcd(unsigned char* data, bool &ret)
{
	ev3dev::lcd lcd = ev3dev::lcd();


	unsigned char *fb = lcd.frame_buffer();
	int buf_size = lcd.frame_buffer_size();
	
	if(lcd.bits_per_pixel() == 1)
	{
		memcpy(&fb[0], &data[0], sizeof(unsigned char)*buf_size);
		ret = true;
	}
	else
	{
		ret = false;
	}
}

/**
* @brief Educator Vehicleの速度(X軸)取得
* @return 速度(X軸)
*/
float controlEV3::get_speed_x()
{
	return m_vx;
}

/**
* @brief Educator Vehicleの速度(Y軸)取得
* @return 速度(Y軸)
*/
float controlEV3::get_speed_y()
{
	return m_vy;
}

/**
* @brief Educator Vehicleの回転速度取得
* @return 回転速度
*/
float controlEV3::get_speed_a()
{
	return m_va;
}

/**
* @brief Educator Vehicleの位置(X軸)取得
* @return 位置(X軸)
*/
float controlEV3::get_position_x()
{
	return m_px;
}

/**
* @brief Educator Vehicleの位置(Y軸)取得
* @return 位置(Y軸)
*/
float controlEV3::get_position_y()
{
	return m_py;
}

/**
* @brief Educator Vehicleの姿勢取得
* @return 姿勢
*/
float controlEV3::get_position_a()
{
	return m_a;
}

/**
* @brief オドメトリ更新
* @param ret 更新できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::update(bool &ret)
{
	ret = true;
	gettimeofday(&current_clock, NULL);
	diff_time = (current_clock.tv_sec - prev_clock.tv_sec) + (current_clock.tv_usec - prev_clock.tv_usec) * 1.0E-6;
	if(diff_time <= 0.0)
	{
		gettimeofday(&prev_clock, NULL);
		ret = false;
		return;
	}
	if(diff_time > 0.5)
	{
		gettimeofday(&prev_clock, NULL);
		ret = false;
		return;
	}
	
	gettimeofday(&prev_clock, NULL);



	float r = m_wheelRadius/2.0;
	float d = m_wheelDistance/2.0;
	float left_wheel_speed = get_speed_left_large_motor(ret) * r;
	if(!ret)return;
	float right_wheel_speed = get_speed_right_large_motor(ret) * r;
	if(!ret)return;


	


	float o = (right_wheel_speed - left_wheel_speed)/(2.0*d);
	float v = (right_wheel_speed + left_wheel_speed)/2.0;
	
	
	m_vx = v * cos(m_a);
	m_vy = v * sin(m_a);
	m_va = o;

	m_px += m_vx*diff_time;
	m_py += m_vy*diff_time;
	m_a += m_va*diff_time;

	
	
}


/**
* @brief Educator Vehicleの速度設定
* @param vx 直進速度
* @param va 回転速度
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::set_speed(float vx, float va, bool &ret)
{
	ret = true;
	float r = m_wheelRadius/2.0;
	float d = m_wheelDistance/2.0;

	float right_motor_speed = (vx + va*d)/r;
	float left_motor_speed = (vx - va*d)/r;

	

	set_speed_left_large_motor(left_motor_speed, ret);
	if(!ret)return;
	set_speed_right_large_motor(right_motor_speed, ret);
	if(!ret)return;
}


/**
* @brief 角度をモーターのカウントに変換
* @param rot 角度
* @param m モーター
* @param ret 変換できた場合はtrue、失敗した場合はfalse
* @return カウント
*/
int controlEV3::rot_to_count(float rot, ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return rot*(180/M_PI)*(m->count_per_rot()/(float)360.0);
	}
	ret = false;
	return 0;
}

/**
* @brief モーターのカウントを角度に変換
* @param count カウント
* @param m モーター
* @param ret 変換できた場合はtrue、失敗した場合はfalse
* @return 角度
*/
float controlEV3::count_to_rot(int count, ev3dev::motor *m, bool &ret)
{
	if(m->connected())
	{
		ret = true;
		return (float)count/(180/M_PI)/((float)m->count_per_rot()/360.0);
	}
	ret = false;
	return 0;
}

/**
* @brief Educator Vehicleの位置初期化
* @param px 位置(X)
* @param py 位置(Y)
* @param a 姿勢
*/
void controlEV3::reset_position(float px, float py, float a)
{
	m_px = px;
	m_py = py;
	m_a = a;
}

/**
* @brief 車輪の直径、車輪間の距離設定
* @param wheelRadius 車輪の直径
* @param wheels_distance 車輪間の距離
*/
void controlEV3::set_param(float wheelRadius, float wheelDistance)
{
	m_wheelRadius = wheelRadius;
	m_wheelDistance = wheelDistance;
}


/**
* @brief 全てのモーター停止
* @param ret 設定できた場合はtrue、失敗した場合はfalse
*/
void controlEV3::stop(bool &ret)
{
	ret = true;
	stop_left_large_motor(ret);
	if(!ret)return;
	stop_right_large_motor(ret);
	if(!ret)return;
	stop_medium_motor(ret);
	if(!ret)return;
}
