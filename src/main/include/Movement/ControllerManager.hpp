#ifndef SRC_FRC5572Controller_HPP_
#define SRC_FRC5572Controller_HPP_

#include <frc/Utility.h>
#include <frc/XboxController.h>

#define LEFT_z 2
#define LEFT_x 0
#define LEFT_y 1

#define RIGHT_z 3
#define RIGHT_x 4
#define RIGHT_y 5

#define LEFT_BUTTON 5
#define RIGHT_BUTTON 6

#define X_BUTTON 3
#define Y_BUTTON 4
#define A_BUTTON 1
#define B_BUTTON 2

#define START_BUTTON 8
#define BACK_BUTTON 7

#define LEFT_STICK_BUTTON 9
#define RIGHT_STICK_BUTTON 10

class FRC5572Controller 
{
public:
  bool toggleA = false, togglePressedA = false;
  bool toggleB = false, togglePressedB = false;
  bool toggleX = false, togglePressedX = false;
  bool toggleY = false, togglePressedY = false;
  
  void UpdateToggleA();
  FRC5572Controller(int I);
  ~FRC5572Controller();
  double LT();
  bool LB();
  double RT();
  bool RB();
  bool X();
  bool Y();
  bool A();
  bool B();
  std::pair<double, double> L();
  std::pair<double, double> R();
  int POV();
  bool start();
  bool back();
  bool Lbutton();
  bool Rbutton();
  void rumble(double, double);
private:
  frc::XboxController *pad;
};

#endif