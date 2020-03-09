#include "Vision/ColorManager.h"

void ColorManager::GetClosestColor()
{
    // Get color
    double dConfidence = 0.0;
    m_cDetectedColor = m_ColorSensor.GetColor();    
    m_cMatchedColor  = m_ColorMatcher.MatchClosestColor(m_cDetectedColor, dConfidence);

    std::string sColor;
    if (m_cMatchedColor == m_cBlueTarget)
        sColor = "Blue";
    else if (m_cMatchedColor == m_cGreenTarget)
        sColor = "Green";
    else if (m_cMatchedColor ==  m_cRedTarget)
        sColor = "Red";
    else if (m_cMatchedColor == m_cSecoundRedTarget)
        sColor = "Secound Red";
    else if (m_cMatchedColor == m_cYellowTarget)
        sColor = "Yellow";
    else
        sColor = "Unknown Color";
}

void ColorManager::OutputColor()
{
    frc::SmartDashboard::PutString("Color",      m_sColor);      
    frc::SmartDashboard::PutNumber("Confidence", m_dConfidence);      
    frc::SmartDashboard::PutNumber("Red",   m_cDetectedColor.red);
    frc::SmartDashboard::PutNumber("Green", m_cDetectedColor.green);
    frc::SmartDashboard::PutNumber("Blue",  m_cDetectedColor.blue);
}