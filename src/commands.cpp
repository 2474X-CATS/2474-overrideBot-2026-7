#include "commands.h" 

////////////////////////////////////////////////////////////  
/*
void DriveForwardBy::start(){   

    driveRef.setSpeedFactor(1);
    startingPoint[0] = driveRef.get<double>("Pos_X"); 
    startingPoint[1] = driveRef.get<double>("Pos_Y");  
    control->setLastTimestamp(Brain.Timer.time());   
};
void DriveForwardBy::periodic(){ 
    double output = control->calculate(getDistTraveled(), Brain.Timer.time());  
    if (!goingForward) 
        output = -output;
    driveRef.manualDriveForward(output);
}; 
bool DriveForwardBy::isOver(){ 
    return control->atSetpoint(getDistTraveled());
};
void DriveForwardBy::end(){ 
    driveRef.stop();  
    driveRef.setSpeedFactor(0.85); 
}; 

double DriveForwardBy::getDistTraveled(){ 
    return hypot(driveRef.get<double>("Pos_X") - startingPoint[0], driveRef.get<double>("Pos_Y") - startingPoint[1]); 
}; 
*/  
void DrivePath::start(){ 
    return; 
}  

void DrivePath::periodic(){ 
    if (isDriving()){ 
        drivePeriodic();
    } else { 
        turnPeriodic();
    }
} 

bool DrivePath::isOver(){ 
    return operationsIndex > numOfOperations;
} 

void DrivePath::end(){  
    return;
}

void DrivePath::drivePeriodic(){ 
   if (!initialized){
    initializeDrive();  
    initialized = true;
   } else { 
    if (isDriveOver()){
      operationsIndex += 1; 
      initialized = false;  
      drivebaseRef.stop(); 
      delete drivePID; 
    } else { 
      drive();
    }
   }
}  

void DrivePath::turnPeriodic(){  
   if (!initialized){
    initializeTurn();  
    initialized = true;
   } else { 
    if (isTurnOver()){
      operationsIndex += 1; 
      initialized = false;  
      drivebaseRef.stop(); 
      delete turnPID;
    } else { 
      turn();
    }
   }
} 

bool DrivePath::isDriving(){ 
   return turningFirst ? operationsIndex % 2 == 1 : operationsIndex % 2 == 0;
} 

bool DrivePath::isTurning(){ 
   return turningFirst ? operationsIndex % 2 == 0 : operationsIndex % 2 == 1;
}

bool DrivePath::isDriveOver(){ 
   return drivePID->atSetpoint(getDrivingError());
} 

void DrivePath::initializeDrive(){  

    isGoingForward = setpoints.at(operationsIndex) > 0;  
  
    startingPoint[0] = drivebaseRef.get<double>("Pos_X");   
    startingPoint[1] = drivebaseRef.get<double>("Pos_Y"); 
    
    drivePID = new pidcontroller(drivebaseRef.getPowerPID(), fabs(setpoints.at(operationsIndex)));  
    drivePID->setLastTimestamp(Brain.Timer.time());
}  

void DrivePath::drive(){ 
    double output = fabs(drivePID->calculate(getDrivingError(), Brain.Timer.time()));  
    if (!isGoingForward)  
       output *= -1; 
    drivebaseRef.manualDriveForward(output);
} 

double DrivePath::getDrivingError(){ 
    return hypot(drivebaseRef.get<double>("Pos_X") - startingPoint[0], drivebaseRef.get<double>("Pos_Y") - startingPoint[1]);
} 

bool DrivePath::isTurnOver(){ 
    return turnPID->atSetpoint(getAngularError());
} 

void DrivePath::initializeTurn(){  
    
    double startAngle = drivebaseRef.get<double>("Angle_Degrees_CCW"); 
    
    double angleSetpoint = setpoints.at(operationsIndex); 
    
    double normalDist = startAngle > angleSetpoint ? (360 - startAngle) + angleSetpoint : angleSetpoint - startAngle;  
    isCounterClockwise = true; 

    if (normalDist > 180){
       isCounterClockwise = false;    
       normalDist = 360 - normalDist;  
    }

    Brain.Screen.print(normalDist); 
    Brain.Screen.newLine(); 
    Brain.Screen.print(isCounterClockwise ? "counter-clockwise" : "clockwise");  
    Brain.Screen.newLine();  

    turnPID = new pidcontroller(drivebaseRef.getTurningPID(), 0);    
    turnPID->setLastTimestamp(Brain.Timer.time());  

} 

void DrivePath::turn(){ 
    double output = turnPID->calculate(getAngularError(), Brain.Timer.time());   
    if (isCounterClockwise){ 
      output *= -1;  
    } 
    drivebaseRef.manualTurnClockwise(-output); 
}  

double DrivePath::getAngularError(){  
    double currentAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");  
    double angleSetpoint = setpoints.at(operationsIndex); 
    
    double dist = currentAngle > angleSetpoint ? (360 - currentAngle) + angleSetpoint : angleSetpoint - currentAngle;  

    if (!isCounterClockwise){   
        dist = (360 - dist); 
    } 
    
    return dist;
} 


void DriveToSetpoint::start(){  

    double startingX = drivebaseRef.get<double>("Pos_X"); 
    double startingY = drivebaseRef.get<double>("Pos_Y");   

    double overallDist = hypot(startingX - setpointX, startingY - setpointY); 
    double overallAngle = fmod((atan2(startingY - setpointY, startingX - setpointX) / M_PI * 180 + 180), 360); 

    double xDist = setpointX - startingX;  
    double yDist = setpointY - startingY; 
    
    switch (pathType){ 
        case EUCLIDEAN:  
          setpoints.push_back(overallAngle);  
          setpoints.push_back(overallDist); 
          break;  
        case MANHATTAN_XY: 
          setpoints.push_back((xDist >= 0 ? 0 : 180)); 
          setpoints.push_back(fabs(xDist)); 
          setpoints.push_back((yDist >= 0 ? 90 : 270)); 
          setpoints.push_back(fabs(yDist)); 
          break;
        case MANHATTAN_YX:  
          setpoints.push_back((yDist >= 0 ? 90 : 270)); 
          setpoints.push_back(fabs(yDist));  
          setpoints.push_back((xDist >= 0 ? 0 : 180)); 
          setpoints.push_back(fabs(xDist));
          break;
    } 

    setpoints.push_back(endingAngle);
    
    numOfOperations = setpoints.size() - 1;  
}

////////////////////////////////////////////////////////////  
/*
void DriveForwardWhileIntaking::start(){  
    hoodRef.close();
    startingPoint[0] = driveRef.get<double>("Pos_X"); 
    startingPoint[1] = driveRef.get<double>("Pos_Y");  
    control->setLastTimestamp(Brain.Timer.time());   
};

void DriveForwardWhileIntaking::periodic(){ 
    double output = control->calculate(getDistTraveled(), Brain.Timer.time()) * 1.5;  
    if (!goingForward) 
        output = -output; 
    intakeRef.intake(); 
    indexerRef.spinOver();
    driveRef.manualDriveForward(output);  
}; 

bool DriveForwardWhileIntaking::isOver(){ 
    return control->atSetpoint(getDistTraveled());
};

void DriveForwardWhileIntaking::end(){ 
    driveRef.stop();   
    intakeRef.stop();   
    indexerRef.stop();
}; 

double DriveForwardWhileIntaking::getDistTraveled(){ 
    return hypot(driveRef.get<double>("Pos_X") - startingPoint[0], driveRef.get<double>("Pos_Y") - startingPoint[1]); 
}; 
*/
////////////////////////////////////////////////////////////

void DriveForwardForTime::start(){ 
    drivebaseRef.setSpeedFactor(1); 
    startingTime = Brain.Timer.time(); 
}; 

void DriveForwardForTime::periodic(){ 
    drivebaseRef.arcadeDrive(percentage * 100, 0); 
};

bool DriveForwardForTime::isOver(){ 
    return Brain.Timer.time() - startingTime >= timeDuration;
} 

void DriveForwardForTime::end(){ 
    drivebaseRef.stop(); 
    drivebaseRef.setSpeedFactor(0.85);
}

void IntakeToHopper::start(){ 
   hoodRef.close(); 
   startingTime = Brain.Timer.time(vex::msec); 
};  

void IntakeToHopper::periodic(){ 
  intakeRef.intake(); 
  indexerRef.spinOver();
};  

bool IntakeToHopper::isOver(){ 
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
} 

void IntakeToHopper::end(){ 
    intakeRef.stop(); 
    indexerRef.stop(); 
    hoodRef.stop();
} 


////////////////////////////////////////////////////////////  

void ScoreOnGoal::start(){ 
   //Top: 1 
   //Mid: 2 
   //Low: 3 
   if (goal == 1)  
     hoodRef.open();  
   hopperRef.dispenseCubes();
   startingTime = Brain.Timer.time(); 
} 

void ScoreOnGoal::periodic(){   
    hopperRef.dispenseCubes(); 
    if (goal != 3){ 
        intakeRef.intake(); 
        if (goal == 1) 
            indexerRef.spinOver();
        else { 
            indexerRef.spinUnder();
        }
    } else { 
        intakeRef.outtake();
    } 
} 

bool ScoreOnGoal::isOver(){ 
    return Brain.Timer.time() - startingTime >= timeDuration; 
} 

void ScoreOnGoal::end(){ 
    intakeRef.stop(); 
    indexerRef.stop(); 
    hopperRef.stop();  
} 



////////////////////////////////////////////////////////////   

void DeployMatchloader::start(){  
    if (isOut) 
       matchLoaderRef.deploy();
    else { 
       matchLoaderRef.retract();
    }
} 

void DeployMatchloader::periodic(){
    return;
} 

bool DeployMatchloader::isOver(){ 
    return true; 
}

void DeployMatchloader::end(){ 
    return;
}   

////////////////////////////////////////////////////////////    

void WaitFor::start(){ 
    startTime = Brain.Timer.time(); 
} 

void WaitFor::periodic(){ 
    return;
} 

bool WaitFor::isOver(){ 
    return Brain.Timer.time() - startTime >= timeDuration;
} 

void WaitFor::end(){ 
    return;
} 



////////////////////////////////////////////////////////////  



/*
CommandInterface* driveForwardByTiles(double tiles){ 
    return DriveForwardBy::getCommand(fabs(tiles * TILE_SIZE_MM), tiles > 0);
};  
*/
/*
CommandInterface* turnToAngle(double goalHeading){ 
    return TurnToHeading::getCommand(goalHeading);
}; 
*/
CommandInterface* scoreOnGoal(Goal_Pos position, double timeDuration){ 
    return ScoreOnGoal::getCommand(static_cast<int>(position), timeDuration);
}; 

CommandInterface* intakeCubes(double timeDuration){ 
    return IntakeToHopper::getCommand(timeDuration); 
}; 

CommandInterface* holdFor(double timeDuration){ 
    return WaitFor::getCommand(timeDuration);
}; 

CommandInterface* extend(){ 
    return DeployMatchloader::getCommand(true);
};

CommandInterface* retract(){ 
    return DeployMatchloader::getCommand(false);
};  
/*
CommandInterface* driveAndIntakeForTiles(double tiles){ 
    return DriveForwardWhileIntaking::getCommand(fabs(tiles * TILE_SIZE_MM), tiles > 0); 
}; 
*/
CommandInterface* ramForwardFor(double percentage, double timeDuration){ 
    return DriveForwardForTime::getCommand(percentage, timeDuration);
}

CommandInterface* driveToPoint(double setX, double setY, PathType pathType){ 
   return DriveToSetpoint::getCommand(setX, setY, pathType);
} 

CommandInterface* alignWithLocation(int locationIndex, double projectedDist, PathType pathType){ 
    return DriveToSetpoint::getCommand((int)locationIndex, projectedDist, pathType);
}

/////////////////////////////////////////////////////////////////////  

/*
string makePreciseDrive(){  
    ostringstream res; 

    double dist = 0;   
    bool intaking;
    while (true){   

       Brain.Screen.printAt(50, 200, "Press "B" for drive without intaking");   
       Brain.Screen.printAt(50, 220, "Press "A" for drive with intaking");    

       if (Controller.ButtonB.pressing()){
         intaking = false;  
         break;
       } 
       if (Controller.ButtonA.pressing()){ 
         intaking = true  
         break;
       }
    }   

    Brain.Screen.clearScreen(); 

    Drivebase::globalRef->setSpeedFactor(0.1); 

    while (!(Controller.ButtonL2.pressing() && Controller.ButtonRight.pressing())){  
        
        Brain.Screen.printAt();

        Drivebase::globalRef->arcadeDrive(Controller.Axis3.position(), 0); 
      
      dist += Drivebase::globalRef->get<double>("Velocity_mm/20ms");   

      Brain.Screen.printAt(50,200, "%f", dist); 

      if (intaking){ 
        
        Intake::globalRef->intake(); 

      }
    }  
    
    Brain.Screen.clearScreen(); 

    Intake::globalRef->stop();   
    Drivebase::globalRef->stop(); 
    Drivebase::globalRef->setSpeedFactor(0.85); 
    
    res << intaking << "," << fabs(dist) << "," << (dist > 0);
    //if (intaking)
        //return DriveForwardWhileIntaking::getCommand(fabs(dist), dist > 0);
    //return DriveForwardBy::getCommand(fabs(dist), dist > 0); 
    return res.str();
}

string makePreciseTurn(){   
    ostringstream res;
    Drivebase::globalRef->setSpeedFactor(0.1);
    while (!(Controller.ButtonL2.pressing() && Controller.ButtonRight.pressing())){   

        Brain.Screen.printAt(50, 150, "Use the right x-axis on the controller to adjust the angle...");
        Brain.Screen.printAt(20, 200, "Turn angle: %f", Drivebase::globalRef->get<double>("Angle_Degrees"));  

        Drivebase::globalRef->arcadeDrive(0, Controller.Axis1.position());  

    }  
    Brain.Screen.clearScreen(); 
    Drivebase::globalRef->stop();  
    Drivebase::globalRef->setSpeedFactor(0.85); 

    res "Drive to pos:"<< Drivebase::globalRef->get<double>("Angle_Degrees");
    return res.str();
    //return TurnToHeading::getCommand(Drivebase::globalRef->get<double>("Angle_Degrees"));
} 

string makeIntakeCubes(){  
     
     ostringstream res; 

     double startTime;   
     double elapsedTime = -1;
     bool holding = false;  

     Hood::globalRef->close(); 

     while ((!(Controller.ButtonL2.pressing() && Controller.ButtonRight.pressing())) || elapsedTime == -1){ 
        
        Brain.Screen.printAt(50, 150, "Hold Y to intake cubes"); 
        Brain.Screen.printAt(50, 150, "Press L2 + Arrow Right to exit");  

        if (!holding){ 
        if (Controller.ButtonY.pressing()){
           holding = true; 
           startTime = Brain.Timer.time();  
        }
       }  
       if (holding){   

         Brain.Screen.printAt(50, 200, "Time holding: %f", Brain.Timer.time() - startTime);   
         Brain.Screen.printAt(50, 250, "Last time elapsed: %f", elapsedTime); 

         Intake::globalRef->intake(); 
         Indexer::globalRef->spinOver();

         if (!Controller.ButtonY.pressing()){ 
            holding = false;   

            Brain.Screen.clearLine(200); 
            Brain.Screen.clearLine(250);

            Intake::globalRef->stop(); 
            Indexer::globalRef->stop(); 

            elapsedTime = Brain.Timer.time() - startTime; 
         } 
       } 
     } 
     res << "Intaking:" << elapsedTime;  
     return res.str();
     //return IntakeToHopper::getCommand(elapsedTime);
}; 

string makeScoreCubes(){ 
    ostringstream res; 

    int pos;   

    double startTime;   
    double elapsedTime = -1;
    bool holding = false;  

    while (true){   
        
        Brain.Screen.printAt(50, 200, "Choose which goal you are scoring at: [Arrow Up : 1 : High], [Arrow Left : 2 : Mid], [Arrow Down : 3 : Low]");
        Brain.Screen.printAt(50, 150, "Press L2 + Arrow Right to exit"); 

        if (Controller.ButtonUp.pressing()){
           pos = 1;   
           break
        }  
        if (Controller.ButtonLeft.pressing()){ 
           pos = 2; 
           break;
        } 
        if (Controller.ButtonDown.pressing()){ 
            pos = 3; 
            break;
        }
    }   

    Brain.Screen.clearLine(200); 
    Brain.Screen.clearLine(150);

    if (pos == 1){ 
        Hood::globalRef->open();
    }
    while ((!(Controller.ButtonL2.pressing() && Controller.ButtonRight.pressing())) || elapsedTime == -1){ 
       
       Brain.Screen.printAt(50,100, "Scoring for goal: %d", pos);  
       Brain.Screen.printAt(50, 125, "Hold Y to score:");   
       Brain.Screen.printAt(50, 150, "Press L2 + Arrow Right to exit"); 
        

       if (!holding){ 
        if (Controller.ButtonY.pressing()){
           holding = true; 
           startTime = Brain.Timer.time();  
        }
       }  
       if (holding){       

         Brain.Screen.printAt(50, 250, "Last time elapsed: %f", elapsedTime); 
         Brain.Screen.printAt(50, 200, "Time holding: %f", Brain.Timer.time() - startTime);   
         

         Hopper::globalRef->dispenseCubes();
         switch (pos){ 
            case 1: 
              Intake::globalRef->intake(); 
              Indexer::globalRef->spinOver(); 
              break  
            case 2: 
              Intake::globalRef->intake(); 
              Indexer::globalRef.->spinUnder();  
              break;
            case 3: 
              Intake::globalRef->outtake(); 
              break; 
         }
         if (!Controller.ButtonY.pressing()){ 
            holding = false;  
            
            Brain.Screen.clearLine(200); 
            Brain.Screen.clearLine(250);

            Intake::globalRef->stop(); 
            Indexer::globalRef->stop();  
            Hopper::globalRef->stop();

            elapsedTime = Brain.Timer.time() - startTime; 
         } 
       } 

    } 
    res << "ScoreOnGoal:" << pos << "," << elapsedTime; 
    return res.str();
    //return ScoreOnGoal::getCommand(pos, elapsedTime);
} 

string makeRamForward(){  
    ostringstream res; 

    double percentSpeed = 0; 
    while (!Controller.ButtonRight.pressing()){ 
        if (Controller.ButtonUp.pressing()){ 
            percentSpeed = percentSpeed < 100 ? percentSpeed + 5 : percentSpeed; 
        } else if (Controller.ButtonDown.pressing()){ 
            percentSpeed = percentSpeed > -100 ? percentSpeed - 5 : percentSpeed;
        }
    } 
    double startTime;  
    double elapsedTime;
    bool holding = false; 
    
    Drivebase::globalRef->setSpeedFactor(1);

    while (true){    

      Brain.Screen.printAt(50, 150, "Hold the R2 button to Ram");   

      if (!holding){ 
         if (Controller.ButtonR2.pressing()){
           holding = true;   
           startTime = Brain.Timer.time(); 
         }
      } else {   

        Brain.Screen.printAt(50, 200, "Last time elapsed: %f", elapsedTime);  
        Brain.Screen.printAt(50, 225, "Time holding: %f", Brain.Timer.time() - startTime);

        Drivebase::globalRef->arcadeDrive(percentSpeed, 0); 

        if (!Controller.ButtonR2.pressing()){   

            Drivebase::globalRef->stop();  

            elapsedTime = Brain.Timer.time() - startTime;  
            break;  

        }
      }
    } 
    res << "Ram forward" << percentSpeed << "," << elapsedTime; 
    return res.str();
    //return DriveForwardForTime::getCommand(percentSpeed, elapsedTime);
}
*/
