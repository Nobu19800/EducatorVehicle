// -*- C++ -*-
/*!
 * @file  controlEV3.h
 * @brief レゴ　マインドストーム EV3のEducator Vehicle操作基本クラス
 * @date  $Date$
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

#ifndef CONTROLEV3_H
#define CONTROLEV3_H

#define RIGHT_TOUCH_SENSOR_ADDRESS ev3dev::INPUT_3
#define LEFT_TOUCH_SENSOR_ADDRESS ev3dev::INPUT_1
#define RIGHT_LARGE_MOTOR_ADDRESS ev3dev::OUTPUT_C
#define LEFT_LARGE_MOTOR_ADDRESS ev3dev::OUTPUT_B

#define DEFAULT_WHEEL_RADIUS 0.056
#define DEFAULT_WHEEL_DISTANCE 0.1185

#include <ev3dev.h>
#include <string>

#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/time.h>


/*!
 * @class controlEV3
 * @brief レゴ　マインドストーム EV3のEducator Vehicle操作基本クラス
 *
 * 
 *
 */
class controlEV3
{
 public:
	/**
	* @brief コンストラクタ
	*/
	controlEV3();
	/**
	* @brief デストラクタ
	*/
	~controlEV3();
	/**
	* @brief 初期化
	* @param wheelRadius 車輪の直径
	* @param wheels_distance 車輪間の距離
	* @param px 初期位置X
	* @param py 初期位置Y
	* @param a 初期姿勢
	* @return true
	*/
	bool init(float wheelRadius=DEFAULT_WHEEL_RADIUS, float wheels_distance=DEFAULT_WHEEL_DISTANCE, float px=0, float py=0, float a=0);
	/**
	* @brief 右タッチセンサのオンオフを取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return タッチセンサがオンの場合はtrue、オフの場合はfalse
	*/
	bool right_touch_sensor_is_pressed(bool &ret);
	/**
	* @brief 左タッチセンサのオンオフを取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return タッチセンサがオンの場合はtrue、オフの場合はfalse
	*/
	bool left_touch_sensor_is_pressed(bool &ret);
	/**
	* @brief カラーセンサで色を取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 計測した色
	*/
	std::string get_color_sensor(bool &ret);
	/**
	* @brief カラーセンサで反射光の強さを取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 反射光の強さ
	*/
	float get_light_reflect(bool &ret);
	/**
	* @brief ジャイロセンサで角度を取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 角度
	*/
	float get_angle_gyro_sensor(bool &ret);
	/**
	* @brief 超音波センサで距離を取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 距離
	*/
	float get_distance_ultrasonic_sensor(bool &ret);

	/**
	* @brief モーターの速度設定
	* @param m モーター
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_speed_motor(ev3dev::motor *m, float v, bool &ret);
	/**
	* @brief モーターの位置設定
	* @param m モーター
	* @param p 位置
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_position_motor(ev3dev::motor *m, float p, float v, bool &ret);
	/**
	* @brief モーターの速度取得
	* @param m モーター
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 速度
	*/
	float get_speed_motor(ev3dev::motor *m, bool &ret);
	/**
	* @brief モーターの位置取得
	* @param m モーター
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 位置
	*/
	float get_position_motor(ev3dev::motor *m, bool &ret);

	/**
	* @brief 左モーターの速度設定
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_speed_left_large_motor(float v, bool &ret);
	/**
	* @brief 右モーターの速度設定
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_speed_right_large_motor(float v, bool &ret);

	/**
	* @brief 左モーターの位置設定
	* @param p 位置
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_position_left_large_motor(float p, float v, bool &ret);
	/**
	* @brief 右モーターの位置設定
	* @param p 位置
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_position_right_large_motor(float p, float v, bool &ret);
	/**
	* @brief 左モーターの速度取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 速度
	*/
	float get_speed_left_large_motor(bool &ret);
	/**
	* @brief 右モーターの速度取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 速度
	*/
	float get_speed_right_large_motor(bool &ret);
	/**
	* @brief 右モーターの位置取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 位置
	*/
	float get_position_left_large_motor(bool &ret);
	/**
	* @brief 左モーターの位置取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 位置
	*/
	float get_position_right_large_motor(bool &ret);
	/**
	* @brief 左モーターをリセット
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void reset_left_large_motor(bool &ret);
	/**
	* @brief 右モーターをリセット
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void reset_right_large_motor(bool &ret);
	/**
	* @brief 左モーターを停止
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void stop_left_large_motor(bool &ret);
	/**
	* @brief 右モーターを停止
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void stop_right_large_motor(bool &ret);
	/**
	* @brief Mモーターの速度設定
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_speed_medium_motor(float v, bool &ret);
	/**
	* @brief Mモーターの位置設定
	* @param p 位置
	* @param v 速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_position_medium_motor(float p, float v, bool &ret);
	/**
	* @brief Mモーターの速度取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 速度
	*/
	float get_speed_medium_motor(bool &ret);
	/**
	* @brief Mモーターの位置取得
	* @param ret 取得できた場合はtrue、失敗した場合はfalse
	* @return 位置
	*/
	float get_position_medium_motor(bool &ret);
	/**
	* @brief Mモーターをリセット
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void reset_medium_motor(bool &ret);
	/**
	* @brief Mモーターを停止
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void stop_medium_motor(bool &ret);
	/**
	* @brief LCDの操作
	* @param file_name 表示する画像データ
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_image_lcd(std::string file_name, bool &ret);
	/**
	* @brief Educator Vehicleの速度(X軸)取得
	* @return 速度(X軸)
	*/
	float get_speed_x();
	/**
	* @brief Educator Vehicleの速度(Y軸)取得
	* @return 速度(Y軸)
	*/
	float get_speed_y();
	/**
	* @brief Educator Vehicleの回転速度取得
	* @return 回転速度
	*/
	float get_speed_a();
	/**
	* @brief Educator Vehicleの位置(X軸)取得
	* @return 位置(X軸)
	*/
	float get_position_x();
	/**
	* @brief Educator Vehicleの位置(Y軸)取得
	* @return 位置(Y軸)
	*/
	float get_position_y();
	/**
	* @brief Educator Vehicleの姿勢取得
	* @return 姿勢
	*/
	float get_position_a();
	/**
	* @brief 位置更新
	* @param ret 更新できた場合はtrue、失敗した場合はfalse
	*/
	void update(bool &ret);
	/**
	* @brief Educator Vehicleの速度設定
	* @param vx 直進速度
	* @param va 回転速度
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void set_speed(float vx, float va, bool &ret);
	/**
	* @brief 角度をモーターのカウントに変換
	* @param rot 角度
	* @param m モーター
	* @param ret 変換できた場合はtrue、失敗した場合はfalse
	* @return カウント
	*/
	int rot_to_count(float rot, ev3dev::motor *m, bool &ret);
	/**
	* @brief モーターのカウントを角度に変換
	* @param count カウント
	* @param m モーター
	* @param ret 変換できた場合はtrue、失敗した場合はfalse
	* @return 角度
	*/
	float count_to_rot(int count, ev3dev::motor *m, bool &ret);
	/**
	* @brief Educator Vehicleの位置初期化
	* @param px 位置(X)
	* @param py 位置(Y)
	* @param a 姿勢
	*/
	void reset_position(float px=0, float py=0, float a=0);
	/**
	* @brief 車輪の直径、車輪間の距離設定
	* @param wheelRadius 車輪の直径
	* @param wheels_distance 車輪間の距離
	*/
	void set_param(float wheelRadius=DEFAULT_WHEEL_RADIUS, float wheels_distance=DEFAULT_WHEEL_DISTANCE);
	/**
	* @brief 全てのモーター停止
	* @param ret 設定できた場合はtrue、失敗した場合はfalse
	*/
	void stop(bool &ret);
 private:
	ev3dev::touch_sensor *right_touch_sensor;
	ev3dev::touch_sensor *left_touch_sensor;
	ev3dev::color_sensor *color_sensor;
	ev3dev::ultrasonic_sensor *ultrasonic_sensor;
	ev3dev::gyro_sensor *gyro_sensor;
	ev3dev::large_motor *left_large_motor;
	ev3dev::large_motor *right_large_motor;
	ev3dev::medium_motor *medium_motor;
	struct timeval prev_clock;
	struct timeval current_clock;
	float diff_time;
	float m_vx, m_vy, m_va;
	float m_px, m_py, m_a;
	float m_wheelRadius, m_wheelDistance;
};




#endif
