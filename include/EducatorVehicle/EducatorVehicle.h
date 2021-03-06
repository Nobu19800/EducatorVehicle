// -*- C++ -*-
/*!
 * @file  EducatorVehicle.h
 * @brief Educator Vehicle
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

#ifndef EDUCATORVEHICLE_H
#define EDUCATORVEHICLE_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

using namespace RTC;

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include "controlEV3.h"

/*!
 * @class EducatorVehicle
 * @brief Educator Vehicle
 *
 * レゴ　マインドストーム EV3の制御コンポーネント
 *
 */
class EducatorVehicle
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  EducatorVehicle(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~EducatorVehicle();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * タイヤの半径
   * - Name: wheelRadius wheelRadius
   * - DefaultValue: 0.028
   * - Unit: m
   */
  double m_wheelRadius;
  /*!
   * タイヤ間距離の1/2
   * - Name: wheelDistance wheelDistance
   * - DefaultValue: 0.05925
   * - Unit: m
   */
  double m_wheelDistance;
  /*!

   * Mモーターの速度
   * - Name: medium_motor_speed medium_motor_speed
   * - DefaultValue: 1.6

   * - Unit: rad/s
   */
  double m_medium_motor_speed;
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedVelocity2D m_velocity2D;
  /*!
   * 速度指令 (v_x, v_y, v_θ)
   * - Type: RTC::TimedVelocity32D
   * - Unit: m/s, m/s, rad/s
   */
  InPort<RTC::TimedVelocity2D> m_velocity2DIn;
  RTC::TimedDouble m_angle;
  /*!
   * モーターMの角度
   * - Type: RTC::TImedDouble
   * - Unit: rad
   */
  InPort<RTC::TimedDouble> m_angleIn;
  RTC::CameraImage m_lcd;
  /*!
   * LCDに表示する画像ファイル名
   * - Type: RTC::CamerImage
   */
  InPort<RTC::CameraImage> m_lcdIn;
  RTC::TimedString m_sound;
  /*!
   * 出力する音声を設定します。
   * 以下のコマンドを使用できます。
   * beep：ビープ音
   * tone,周波数:指定周波数の音
   * コマンドを使用しない場合は入力文字列を発音します。
   */
  InPort<RTC::TimedString> m_soundIn;
  RTC::TimedPose2D m_pos_update;
  /*!
  * 現在位置の更新
  * - Type: RTC::TimedPose2D
  * - Unit: m,rad
  */
  InPort<RTC::TimedPose2D> m_pos_updateIn;

  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedPose2D m_odometry;
  /*!
   * 現在の位置・姿勢（角度） (x, y, θ)
   * - Type: RTC::TimedPose2D
   * - Unit: m, m, rad
   */
  OutPort<RTC::TimedPose2D> m_odometryOut;
  RTC::RangeData m_ultrasonic;
  /*!
   * 超音波センサをレンジセンサと仮定し、要素1の距離データを格納
   * - Type: RTC::RangeData
   * - Unit: m
   */
  OutPort<RTC::RangeData> m_ultrasonicOut;
  RTC::TimedDouble m_gyro;
  /*!
   * ジャイロセンサをTimedDoubleにて出力
   * - Type: RTC::TimedDouble
   * - Unit: rad
   */
  OutPort<RTC::TimedDouble> m_gyroOut;
  RTC::TimedString m_color;
  /*!
   * カラーセンサの値を色名 (none, black, white, blue, green,
   * red, yellow, brown) で出力
   * - Type: RTC::TimedString
   */
  OutPort<RTC::TimedString> m_colorOut;
  RTC::TimedDouble m_light_reflect;
  /*!
   * 反射光の強さ
   * - Type: RTC::TimedDouble
   */
  OutPort<RTC::TimedDouble> m_light_reflectOut;
  RTC::TimedBooleanSeq m_touch;
  /*!
   * タッチセンサの値をBoolean[2] で出力
   * - Type: RTC::TimedBooleanSeq
   */
  OutPort<RTC::TimedBooleanSeq> m_touchOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
	controlEV3 robot;
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void EducatorVehicleInit(RTC::Manager* manager);
};

#endif // EDUCATORVEHICLE_H
