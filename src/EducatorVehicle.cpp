// -*- C++ -*-
/*!
 * @file  EducatorVehicle.cpp
 * @brief Educator Vehicle
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

#include "EducatorVehicle.h"

// Module specification
// <rtc-template block="module_spec">
static const char* educatorvehicle_spec[] =
  {
    "implementation_id", "EducatorVehicle",
    "type_name",         "EducatorVehicle",
    "description",       "Educator Vehicle",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "MobileRobot",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.wheelRadius", "0.028",
    "conf.default.wheelDistance", "0.054",
    "conf.default.medium_motor_speed", "1.6",
    // Widget
    "conf.__widget__.wheelRadius", "text",
    "conf.__widget__.wheelDistance", "text",
    "conf.__widget__.medium_motor_speed", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
EducatorVehicle::EducatorVehicle(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_velocity2DIn("velocity2D", m_velocity2D),
    m_angleIn("angle", m_angle),
    m_lcdIn("lcd", m_lcd),
    m_soundIn("sound", m_sound),
    m_odometryOut("odometry", m_odometry),
    m_ultrasonicOut("ultrasonic", m_ultrasonic),
    m_gyroOut("gyro", m_gyro),
    m_colorOut("color", m_color),
    m_light_reflectOut("light_reflect", m_light_reflect),
    m_touchOut("touch", m_touch)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
EducatorVehicle::~EducatorVehicle()
{
}



RTC::ReturnCode_t EducatorVehicle::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("velocity2D", m_velocity2DIn);
  addInPort("angle", m_angleIn);
  addInPort("lcd", m_lcdIn);
  addInPort("sound", m_soundIn);
  
  // Set OutPort buffer
  addOutPort("odometry", m_odometryOut);
  addOutPort("ultrasonic", m_ultrasonicOut);
  addOutPort("gyro", m_gyroOut);
  addOutPort("color", m_colorOut);
  addOutPort("light_reflect", m_light_reflectOut);
  addOutPort("touch", m_touchOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("wheelRadius", m_wheelRadius, "0.028");
  bindParameter("wheelDistance", m_wheelDistance, "0.054");
  bindParameter("medium_motor_speed", m_medium_motor_speed, "1.6");
  // </rtc-template>
  
  
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onFinalize()
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onActivated(RTC::UniqueId ec_id)
{
	bool ret = true;
	robot.init();
	robot.set_param(m_wheelRadius, m_wheelDistance);
	robot.reset_left_large_motor(ret);
	robot.reset_right_large_motor(ret);
	robot.reset_medium_motor(ret);	
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onDeactivated(RTC::UniqueId ec_id)
{
	bool ret = true;
	robot.stop(ret);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onExecute(RTC::UniqueId ec_id)
{
	bool ret = true;
	robot.update(ret);

	if(ret)
	{
		m_odometry.data.position.x = robot.get_position_x();
		m_odometry.data.position.y = robot.get_position_y();
		m_odometry.data.heading = robot.get_position_a();
		setTimestamp(m_odometry);
		m_odometryOut.write();
	}

	if(m_velocity2DIn.isNew())
	{
		m_velocity2DIn.read();
		robot.set_speed(m_velocity2D.data.vx, m_velocity2D.data.va, ret);
	}
	if(m_angleIn.isNew())
	{
		m_angleIn.read();
		robot.set_position_medium_motor(m_angle.data, m_medium_motor_speed, ret);
	}

	
	
	m_ultrasonic.ranges.length(1);
	m_ultrasonic.ranges[0] = robot.get_distance_ultrasonic_sensor(ret);
	if(ret)
	{
		
		m_ultrasonic.config.minAngle = robot.get_position_medium_motor(ret);
		m_ultrasonic.config.maxAngle = robot.get_position_medium_motor(ret);
		m_ultrasonic.config.angularRes = 0.0;
		setTimestamp(m_ultrasonic);
		m_ultrasonicOut.write();
	}
	
	m_gyro.data = robot.get_angle_gyro_sensor(ret);
	if(ret)
	{
		setTimestamp(m_gyro);
		m_gyroOut.write();
	}

	m_color.data = robot.get_color_sensor(ret).c_str();
	if(ret)
	{
		setTimestamp(m_color);
		m_colorOut.write();
	}

	m_light_reflect.data = robot.get_light_reflect(ret);
	if(ret)
	{
		setTimestamp(m_light_reflect);
		m_light_reflectOut.write();
	}

	bool right_ret = true;
	bool touch_right = robot.right_touch_sensor_is_pressed(right_ret);
	if(right_ret)
	{
		bool left_ret = true;
		bool touch_left = robot.left_touch_sensor_is_pressed(left_ret);
		if(left_ret)
		{
			m_touch.data.length(2);
			m_touch.data[1] = touch_left;
		}
		else
		{
			m_touch.data.length(1);
		}
		m_touch.data[0] = touch_right;
		
		setTimestamp(m_touch);
		m_touchOut.write();
	}

	if(m_lcdIn.isNew())
	{
		m_lcdIn.read();
		const char *lcd_data = m_lcd.data;
		robot.set_image_lcd(lcd_data, ret);
	}

	if(m_soundIn.isNew())
	{
		m_soundIn.read();
		std::string sound_data = (const char*)m_sound.data;
		coil::eraseBothEndsBlank(sound_data);
		if(sound_data == "beep")
		{
			ev3dev::sound::beep();
		}
		else if(sound_data.find("tone") != std::string::npos )
		{
			std::vector<std::string> s = coil::split(sound_data,",");
			if(s.size() >= 3)
			{
				int freq = 0;
				float ms = 0;
				if(coil::stringTo<int>(freq,s[1].c_str()) && coil::stringTo<float>(ms,s[2].c_str()))
				{
					ev3dev::sound::tone(freq, ms);
				}
			}
		}
		else
		{
			ev3dev::sound::speak(sound_data.c_str());
		}
	}

  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onAborting(RTC::UniqueId ec_id)
{
	bool ret;
	robot.stop(ret);
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EducatorVehicle::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}



extern "C"
{
 
  void EducatorVehicleInit(RTC::Manager* manager)
  {
    coil::Properties profile(educatorvehicle_spec);
    manager->registerFactory(profile,
                             RTC::Create<EducatorVehicle>,
                             RTC::Delete<EducatorVehicle>);
  }
  
};


