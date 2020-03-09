#ifndef IR_HELPER_HPP
#define IR_HELPER_HPP

#include <frc/AnalogInput.h>

enum InfraredSensorType
{
	GP2Y0A710K0F, // 3 - 18 feet
	GP2Y0A02YK0F, // 7 - 59 inches
	OPB732WZ      // up to 3 inches
};

enum Color
{
	BLACK,
	WHITE,
	GRAY
};

class Infrared
{
public:
	Infrared(int irChannel, InfraredSensorType irType);

	Color checkColor();
	double irRange();
	double voltage();
	int averageRaw();

private:
	static constexpr double COLOR_ERROR = 0.1;
	static constexpr double WHITE_VALUE = 0.0;
	static constexpr double BLACK_VALUE = 1.0;
	static constexpr double GRAY_VALUE = 2.0;
    
	double voltageScaling = 1.0;
	double voltageIntercept = 1.0;

    InfraredSensorType sensorFamily;
	frc::AnalogInput* irSensor;
};

#endif