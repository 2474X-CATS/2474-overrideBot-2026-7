#include "simMechs.h" 
#include "../architecture/telemetry.h" 
#include "../utilities/functools.h" 
#include <memory> 

double ElevatorMech::CHANNEL_WIDTH = 7; 
double ElevatorMech::DEFAULT_X = 65; 
double ElevatorMech::DEFAULT_Y = 115;  
double ElevatorMech::MAX_WIDTH = 100; 
double ElevatorMech::STAGE_HEIGHT = 120;

double ElevatorMech::getHeight(){
    return (Telemetry::inst.getValueAt<double>("elevator", "current_height") / 1000 * 240); 
}

void ElevatorMech::draw(){ 

    double cascadeHeight = getHeight();
    double singleStageHeight = cascadeHeight / 2.0; 
  
    double width = MAX_WIDTH; 
    double yPos = DEFAULT_Y;  
    double xPos = DEFAULT_X; 
    int brightness = 0; 

    for (int i = 0; i <= 2; i ++){  
     
      drawRectangle(xPos, yPos, width, CHANNEL_WIDTH, globalColor.rgb(brightness, brightness, brightness)); 
      drawRectangle(xPos, yPos, CHANNEL_WIDTH, STAGE_HEIGHT, globalColor.rgb(brightness, brightness, brightness)); 
      drawRectangle(xPos + width - CHANNEL_WIDTH, yPos, CHANNEL_WIDTH, STAGE_HEIGHT, globalColor.rgb(brightness, brightness, brightness)); 
      
      width -= (CHANNEL_WIDTH * 2);
      xPos += CHANNEL_WIDTH; 
      yPos -= singleStageHeight; 
      brightness += (100 / 3);
    }
} 

void ElevatorMech::update(){ 
    return;
} 

void ElevatorMech::mousePressed(int mx, int my){ 
    return;
} 

void ElevatorMech::mouseReleased(){ 
    return;
}  

double ElevatorMech::getPivotX(){ 
    return DEFAULT_X + (MAX_WIDTH / 2);
} 

double ElevatorMech::getPivotY(){ 
    return DEFAULT_Y - getHeight() + 30;
} 

//--------------------------------------------------------------------------------------

double ForearmMech::FOREARM_LENGTH = 75; 
double ForearmMech::FOREARM_HEIGHT = 20; 


double ForearmMech::getPivotX(){ 
    return elevRef.getPivotX();
}  

double ForearmMech::getPivotY(){ 
    return elevRef.getPivotY(); 
}

double ForearmMech::getAngle(){ 
    return flipOrientation(angleSum(Telemetry::inst.getValueAt<double>("forearm", "current_angle"), 180));
} 

void ForearmMech::draw(){  
    double originX = elevRef.getPivotX(); 
    double originY = elevRef.getPivotY(); 
    double angle = toRadians(getAngle());
      
    Brain.Screen.setPenWidth(FOREARM_HEIGHT);  
    drawLine(originX, originY, originX + cos(angle) * FOREARM_LENGTH, originY + sin(angle) * FOREARM_LENGTH, globalColor.white); 
    Brain.Screen.setPenWidth(FOREARM_HEIGHT - 1);  
    drawLine(originX, originY, originX + cos(angle) * FOREARM_LENGTH, originY + sin(angle) * FOREARM_LENGTH, globalColor.black);
    
    Brain.Screen.setPenWidth(1); 
    Brain.Screen.setPenColor(globalColor.white);
    drawEllipse(originX, originY, FOREARM_HEIGHT/3*2, globalColor.rgb(50,50,50));
    Brain.Screen.setPenColor(vex::transparent);
    
} 

void ForearmMech::update(){ 
    return;
} 

void ForearmMech::mousePressed(int mx, int my){ 
    return;
} 

void ForearmMech::mouseReleased(){ 
    return;
} 

//-------------------------------------------------------------------------------------- 

double ClawMech::CLAW_WIDTH = 20; 
double ClawMech::CLAW_HEIGHT = 50; 

double ClawMech::getOutwardAngle(){  
    double angle;
    if (Telemetry::inst.getValueAt<bool>("claw","clenched")){ 
       angle = 105;
    } else { 
       angle = 135;
    } 
    return angle;
}

double ClawMech::getPivotX(){  
    double offset; 
    if (Telemetry::inst.getValueAt<bool>("claw", "facing_down")){ 
        offset = 0.25;
    } else {
        offset = -0.25;
    }
    return foreRef.getPivotX() + (ForearmMech::FOREARM_LENGTH * cos(toRadians(foreRef.getAngle()) + offset)) ;
} 

double ClawMech::getPivotY(){  
    double offset; 
    if (Telemetry::inst.getValueAt<bool>("claw", "facing_down")){ 
        offset = 0.25;
    } else {
        offset = -0.25;
    }
    return foreRef.getPivotY() + (ForearmMech::FOREARM_LENGTH * sin(toRadians(foreRef.getAngle()) + offset));
}

void ClawMech::draw(){   
    double forearmAngle = foreRef.getAngle();
    double angle = angleSum(forearmAngle, 90);
    double outwardClawAngle1 = angleSum(angle, getOutwardAngle()); 
    double outwardClawAngle2 = angleSum(angle, -getOutwardAngle());
    
    forearmAngle = toRadians(forearmAngle);
    angle = toRadians(angle); 
    outwardClawAngle1 = toRadians(outwardClawAngle1); 
    outwardClawAngle2 = toRadians(outwardClawAngle2); 

    double midpointX = getPivotX(); 
    double midpointY = getPivotY();  

    double x1; 
    double y1;  

    double x2; 
    double y2;  

    double x3; 
    double y3; 

    double x4; 
    double y4; 

    double x5; 
    double y5;  

    double x6; 
    double y6;

    x1 = midpointX + cos(angle) * (CLAW_WIDTH / 2); 
    x2 = midpointX - cos(angle) * (CLAW_WIDTH / 2);   
    x3 = x1 - cos(outwardClawAngle1) * (CLAW_HEIGHT/3);  
    x4 = x2 + cos(outwardClawAngle2) * (CLAW_HEIGHT/3); 
    x5 = x4 + cos(forearmAngle) * (CLAW_HEIGHT/3); 
    x6 = x3 + cos(forearmAngle) * (CLAW_HEIGHT/3);

    y1 = midpointY + sin(angle) * (CLAW_WIDTH / 2); 
    y2 = midpointY - sin(angle) * (CLAW_WIDTH / 2); 
    y3 = y1 - sin(outwardClawAngle1) * (CLAW_HEIGHT/3);  
    y4 = y2 + sin(outwardClawAngle2) * (CLAW_HEIGHT/3);  
    y5 = y4 + sin(forearmAngle) * (CLAW_HEIGHT/3); 
    y6 = y3 + sin(forearmAngle) * (CLAW_HEIGHT/3);
    
    if (Telemetry::inst.getValueAt<bool>("claw","senses_object")){ 
        drawEllipse(midpointX + cos(forearmAngle) * 20, midpointY + sin(forearmAngle) * 20, 10, globalColor.yellow);
    }

    Brain.Screen.setPenWidth(10); 
    
    drawLine(x1, y1, x2, y2, globalColor.rgb(150, 150, 150));  
    drawLine(x1, y1, x3, y3, globalColor.rgb(150, 150, 150));  
    drawLine(x2, y2, x4, y4, globalColor.rgb(150, 150, 150)); 
    drawLine(x4, y4, x5, y5, globalColor.rgb(150, 150, 150)); 
    drawLine(x3, y3, x6, y6, globalColor.rgb(150, 150, 150));
   
    Brain.Screen.setPenWidth(1);    

} 

void ClawMech::update(){ 
    return;
} 

void ClawMech::mousePressed(int mx, int my){ 
    return;
} 

void ClawMech::mouseReleased(){ 
    return;
}  

//--------------------------------------------------------------------------------------  

void Console::draw(){ 
   drawRectangle(x,y,width,height, globalColor.rgb(100,100,100)); 
   renderText("Super Structure Console", x+5, y+20, globalColor.black, globalColor.rgb(100, 100, 100), vex::fontType::mono20); 
   displayStatus(); 
   displayActions();  
   /*
   if (pressed){ 
     drawEllipse(Brain.Screen.xPosition(), Brain.Screen.yPosition(), 20, globalColor.green);
   } 
   */
   
} 

void Console::displayStatus(){  
   
   std::unique_ptr<char> elevatorHeight(new char[30]);
   std::unique_ptr<char> forearmAngle(new char[30]); 
   std::unique_ptr<char> ssPosition(new char[25]); 
   std::unique_ptr<char> clenching(new char[20]); 

   sprintf(elevatorHeight.get(), "Elevator Height: %.3f", Telemetry::inst.getValueAt<double>("elevator","current_height"));  
   sprintf(forearmAngle.get(), "Forearm Angle: %.3f", Telemetry::inst.getValueAt<double>("forearm","current_angle")); 
   sprintf(clenching.get(), "Claw Clenching: %d", Telemetry::inst.getValueAt<bool>("claw","clenched")); 

   string state;  

   switch (Telemetry::inst.getValueAt<int>("ss_manager", "position")){ 
       case PRIMED: 
         state = "PRIMED";  
         break;
       case GROUND:  
         state = "GROUND"; 
         break;
       case STANDING:  
         state = "STANDING"; 
         break;
       case AUTO:  
         state = "AUTO"; 
         break; 
   }  

   sprintf(ssPosition.get(), "SS State: %s", state.c_str());  
   
   //-------------------------------------------------------------------------------------
   drawLine(x, 30, x + width, 30, globalColor.black); 
   drawLine(x, 50, x + width, 50, globalColor.black);
   renderText("Statuses", x+90,45, globalColor.black, globalColor.rgb(100,100,100), vex::fontType::mono15);
   
   renderText(elevatorHeight.get(), x+10, 65, globalColor.black, globalColor.rgb(100,100,100), vex::fontType::mono12); 
   renderText(forearmAngle.get(), x+10, 80, globalColor.black, globalColor.rgb(100,100,100), vex::fontType::mono12); 
   renderText(ssPosition.get(), x+10, 95, globalColor.black, globalColor.rgb(100,100,100), vex::fontType::mono12); 
   renderText(clenching.get(), x+10, 110, globalColor.black, globalColor.rgb(100,100,100), vex::fontType::mono12);
   
   
} 

void Console::displayActions(){  
   drawLine(x, 120, x + width, 120, globalColor.black);  
   renderText("Actions", x+90, 135, globalColor.black, globalColor.rgb(100,100,100), vex::fontType::mono15); 
   drawLine(x, 140, x + width, 140, globalColor.black);
   //displayPrimingAction();
   //displayClawAction(); 
   displayEnableGameObject(); 
   //displaySwitchPickupMode(); 
   //displayActivateMacro();
   return;
}


void Console::update(){ 
    //maintainPrimingAction();
} 

void Console::mousePressed(int mx, int my){ 
    return;
} 

void Console::mouseReleased(){  
    /*
    if (checkClawAction()){ 
        return;
    }  

    if (checkActivateMacro()){ 
        return;
    } 
    */ 

    if (checkGOEnable()){ 
        return;
    } 
    
    /*
    if (checkSwitchUp()){ 
        return;
    }   
    */

}


void Console::displayPrimingAction(){ 
    drawRectangle(x, 140, width / 3 * 2, 30, globalColor.rgb(125, 125, 125));    

    Brain.Screen.setPenWidth(2); 
    drawLine(x + (width/3), 140, x + (width/3), 170, globalColor.black);
    Brain.Screen.setPenWidth(2); 

    drawLine(x + (width/6)+1, y + 140, x + (width/6)+1, y + 156, globalColor.black);  
    drawLine(x + (width/6), y + 156, x + (width/6) + 5, y + 156 - 5, globalColor.black);  
    drawLine(x + (width/6), y + 156, x + (width/6) - 5, y + 156 - 5, globalColor.black); 
    
    drawLine(x + (width/2)+1, y + 172, x + (width/2)+1, y + 156, globalColor.black); 
    drawLine(x + (width/2), y + 156, x + (width/2) - 5, y + 156 + 5, globalColor.black); 
    drawLine(x + (width/2), y + 156, x + (width/2) + 5, y + 156 + 5, globalColor.black);  
    
    renderText("(Prime down)", x + (width/6)+1 -38, y + 156 + 11, globalColor.black, globalColor.rgb(125, 125, 125), vex::fontType::mono12);
    renderText("(Prime up)", x + (width/2) -30, y + 156 - 5, globalColor.black, globalColor.rgb(125, 125, 125), vex::fontType::mono12); 

    Brain.Screen.setPenWidth(1);
}
 
void Console::displayClawAction(){  
    drawRectangle(x + width / 3 * 2, 140, width - (width / 3 * 2), 30, globalColor.rgb(50,50,50));  
    SuperStructurePosition currentPos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 
    bool rested = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached"); 
    bool sensesObject = Telemetry::inst.getValueAt<bool>("claw", "senses_object"); 
    if (rested && sensesObject){  
       if (currentPos == Telemetry::inst.getValueAt<int>("ss_manager", "pickup_position")){ 
        renderText("(GRAB)", x+width/3*2+10, 160, globalColor.white, globalColor.rgb(50,50,50), vex::fontType::mono20);
       } else { 
        renderText("(RELEASE)", x+width/3*2+5, 160, globalColor.white, globalColor.rgb(50,50,50), vex::fontType::mono15);
       } 
       return;
    }  
    renderText("....", x+width/3*2+5 + 15, 160, globalColor.white, globalColor.rgb(50,50,50), vex::fontType::mono20);
}  

void Console::displaySwitchPickupMode(){  
    drawRectangle(x, 170, width / 2, 30, globalColor.rgb(200,200,200)); 
    renderText("Pickup position to", x + 5, 182, globalColor.black, globalColor.rgb(200,200,200), vex::fontType::mono12);
    
    if (Telemetry::inst.getValueAt<int>("ss_manager", "pickup_position") == SuperStructurePosition::GROUND){ 
        renderText("(STANDING)", x+19, 196, globalColor.black, globalColor.rgb(200,200,200), vex::fontType::mono15);
    } else { 
        renderText("(GROUND)", x+27, 196, globalColor.black, globalColor.rgb(200,200,200), vex::fontType::mono15);
    } 
    
}  

void Console::displayActivateMacro(){ 
    drawRectangle(x, 200, width, 40, globalColor.black);  
    SuperStructurePosition currentPos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 
    bool rested = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached");  
    if (currentPos != SuperStructurePosition::PRIMED || !rested){ 
        renderText(".....", x + 100, 220, globalColor.white, globalColor.black, vex::fontType::mono20);
    } else { 
        renderText("ENABLE SCORING MACRO", x + 20, 220, globalColor.white, globalColor.black, vex::fontType::mono20);
    }
}  

void Console::displayEnableGameObject(){ 
    drawRectangle(x + (width/2), 170, width / 2, 30, globalColor.rgb(75, 75, 75));  
    string action; 
    if (Telemetry::inst.getValueAt<bool>("claw", "senses_object")){ 
        renderText("REMOVE", x + width / 2 + 35, 185, globalColor.black, globalColor.rgb(75, 75, 75), vex::fontType::mono15); 
    } else { 
        renderText("ADD", x + width / 2 + 50, 185, globalColor.black, globalColor.rgb(75, 75, 75), vex::fontType::mono15); 
    } 
    renderText("Game Object", x + width / 2 + 25, 197, globalColor.black, globalColor.rgb(75, 75, 75), vex::fontType::mono12);
} 

bool Console::checkRect(int rectX, int rectY, int rectWidth, int rectHeight){ 
    return (Brain.Screen.xPosition() > rectX && Brain.Screen.xPosition() < rectX + rectWidth) && (Brain.Screen.yPosition() > rectY && Brain.Screen.yPosition() < rectY + rectHeight);
} 


bool Console::checkClawAction(){ 
   bool successful = checkRect(x + width / 3 * 2, 140, width - (width / 3 * 2), 30);  
   
   if (successful){  
      if (Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached") && Telemetry::inst.getValueAt<bool>("claw", "senses_object")){ 
          Telemetry::inst.placeValueAt<bool>(true, "claw", "requesting_act"); 
          if (Telemetry::inst.getValueAt<int>("ss_manager", "position") == SuperStructurePosition::PRIMED){ 
            Telemetry::inst.placeValueAt<bool>(false, "claw", "senses_object");
          }
      }
   }
   return successful;
} 

bool Console::checkSwitchUp(){ 
   bool successful = checkRect(x, 170, width / 2, 30); 
   if (successful){ 
     if (Telemetry::inst.getValueAt<int>("ss_manager","pickup_position") == SuperStructurePosition::GROUND){ 
        Telemetry::inst.placeValueAt<int>(SuperStructurePosition::STANDING, "ss_manager", "pickup_position");
     } else { 
        Telemetry::inst.placeValueAt<int>(SuperStructurePosition::GROUND, "ss_manager", "pickup_position");
     }
   } 
   return successful;
}

bool Console::checkActivateMacro(){ 
   bool successful = checkRect(x, 200, width, 40); 
   if (successful){
     SuperStructurePosition currentPos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position")); 
     bool rested = Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached"); 
     if (currentPos == SuperStructurePosition::PRIMED && rested){ 
         Telemetry::inst.placeValueAt<bool>(true, "ss_manager", "macro_requested"); 
         Telemetry::inst.placeValueAt<bool>(false, "claw", "senses_object");
     }
   }
   return successful;
} 

bool Console::checkGOEnable(){ 
   bool successful = checkRect(x + (width/2), 170, width / 2, 30); 
   if (successful){ 
       bool currentlySensesObject = Telemetry::inst.getValueAt<bool>("claw", "senses_object"); 
       Telemetry::inst.placeValueAt<bool>(!currentlySensesObject, "claw", "senses_object");
   } 
   return successful;
} 

void Console::maintainPrimingAction(){  
   int primingDirection = 0;
   if (pressed && checkRect(x, 140, width / 3 * 2, 30)){ 
      if (Brain.Screen.xPosition() < (x + (width/3)) ){ //prime downwards
         primingDirection = -1;
      } else { 
         primingDirection = 1;
      }
   }   

   Telemetry::inst.placeValueAt<int>(primingDirection, "elevator", "priming_direction");

}