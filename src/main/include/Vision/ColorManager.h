#pragma once
#include <frc/util/color.h>
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/smartdashboard/SmartDashboard.h>

class ColorManager
{
public:

  ColorManager()
  {
    m_ColorMatcher.AddColorMatch(m_cBlueTarget);
    m_ColorMatcher.AddColorMatch(m_cGreenTarget);
    m_ColorMatcher.AddColorMatch(m_cRedTarget);
    m_ColorMatcher.AddColorMatch(m_cSecoundRedTarget);
    m_ColorMatcher.AddColorMatch(m_cYellowTarget); 
  }

  double m_dConfidence = 0.0f;
  std::string m_sColor;
  void GetClosestColor();
  void OutputColor();

  /* i2cPort for the color sensor */
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  /* Sensor Objects */
  rev::ColorSensorV3 m_ColorSensor{i2cPort};
  rev::ColorMatch    m_ColorMatcher;
  frc::Color         m_cDetectedColor;  
  frc::Color         m_cMatchedColor;

  /* Pre- Color defines need to be calorbated */
  static constexpr frc::Color m_cBlueTarget = frc::Color(0.143, 0.427, 0.429);
  static constexpr frc::Color m_cGreenTarget = frc::Color(0.197, 0.561, 0.240);
  static constexpr frc::Color m_cSecoundRedTarget = frc::Color(0.561, 0.232, 0.114);
  static constexpr frc::Color m_cRedTarget = frc::Color(0.862, 0, 0);
  static constexpr frc::Color m_cYellowTarget = frc::Color(0.361, 0.524, 0.113);
};