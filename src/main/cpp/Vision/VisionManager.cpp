#include "Vision/VisionManager.hpp"

VisionManager::VisionManager(){
  Update();
}

void VisionManager::Update(){
  tx = table->GetNumber("tx",0.0);
  ty = table->GetNumber("ty",0.0);
  ta = table->GetNumber("ta",0.0);
  tv = table->GetNumber("tv",0.0);

  if (tv < 1.0){
    targetFound = false;
    disX = 0;
    disY = 0;
  }
  else
  {
    targetFound = true;
    disX = tx;
    disY = ty;
  }
}

void TurnOffLights(){
  
}