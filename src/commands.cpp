#include "commands.h"
#include <sstream>

//--------------------------------------- 
// A COMBINATION OF DRIVING FORWARD AND TURNING COMMANDS 

//bool DrivePath::isCounterClockwise = false;

void DrivePath::start()
{ 
    referenceAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");
    return;
}

void DrivePath::periodic()
{
    if (isDriving())
    {
        drivePeriodic();
    }
    else
    {
        turnPeriodic();
    } 
    intakeRef.periodic(); 
}

bool DrivePath::isOver()
{
    return operationsIndex > numOfOperations;
}

void DrivePath::end()
{
   RobotState::manuallyModifyState("intaking", false); 
   drivebaseRef.stop(); 
   intakeRef.stop();
}

void DrivePath::drivePeriodic()
{
    if (!initialized)
    {
        initializeDrive(); 
        initialized = true;  
        RobotState::manuallyModifyState("intaking", intaking);
    }
    else
    {
        if (isDriveOver())
        {
            operationsIndex += 1;
            initialized = false;
            drivebaseRef.stop();
            delete drivingProfile; 
            RobotState::manuallyModifyState("intaking", false);
        }
        else
        {
            drive();
        }
    }
}

void DrivePath::turnPeriodic()
{
    if (!initialized)
    {
        initializeTurn();
        initialized = true;
    }
    else
    {
        if (isTurnOver())
        {
            operationsIndex += 1; 
            initialized = false;
            drivebaseRef.stop();
            delete turnPID;
        }
        else
        {
            turn();
        }
    }
}

bool DrivePath::isDriving()
{
    return turningFirst ? operationsIndex % 2 == 1 : operationsIndex % 2 == 0;
}

bool DrivePath::isTurning()
{
    return turningFirst ? operationsIndex % 2 == 0 : operationsIndex % 2 == 1;
}

bool DrivePath::isDriveOver()
{ 
    return (Brain.Timer.time(vex::sec) - drivingProfile->getStartTime()) >= drivingProfile->getTotalDuration(); 
}

void DrivePath::initializeDrive()
{
    isGoingForward = setpoints.at(operationsIndex) > 0;
    drivingProfile = new TrapezoidalMotionProfile(drivebaseRef.getMotionConstants(), abs(setpoints.at(operationsIndex)), Brain.Timer.time(vex::sec));
    
    referenceAngle = drivebaseRef.get<double>("Angle_Degrees_CCW"); 

    startingPoint[0] = drivebaseRef.get<double>("Pos_X"); 
    startingPoint[1] = drivebaseRef.get<double>("Pos_Y"); 

    lastPoint[0] = startingPoint[0]; 
    lastPoint[1] = startingPoint[1]; 
}

void DrivePath::drive()
{  
    double output;  
    
    double currentDist = hypot( 
       lastPoint[0] - startingPoint[0], 
       lastPoint[1] - startingPoint[1]
    ); 

    TrapezoidalSetpoint setpoint;  

    setpoint = drivingProfile->generateSetpoint(Brain.Timer.time(vex::sec));   

    output = setpoint.velocity; 

    double positionError = setpoint.position - currentDist;
    output += (positionError * 0.03); 

    if (!isGoingForward) 
        output *= -1; 

    drivebaseRef.manualDriveForward(output, referenceAngle);  

    lastPoint[0] = drivebaseRef.get<double>("Pos_X"); 
    lastPoint[1] = drivebaseRef.get<double>("Pos_Y"); 


}


bool DrivePath::isTurnOver()
{
    return turnPID->atSetpoint(getAngularError()) || setpoints.at(operationsIndex) == -1;
}

void DrivePath::initializeTurn()
{  
    turnPID = new pidcontroller(drivebaseRef.getTurningPID(), 0);
    turnPID->setLastTimestamp(Brain.Timer.time());  
}

void DrivePath::turn()
{
    double output = turnPID->calculate(getAngularError(), Brain.Timer.time());  
    
    if (RobotState::getStateOf("is_counterclockwise"))
       drivebaseRef.manualTurnClockwise(output); 
    else { 
       drivebaseRef.manualTurnClockwise(-output);
    }
}

double DrivePath::getAngularError()
{  
    double currentAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");   

    double angleSetpoint = setpoints.at(operationsIndex); 

    double dist; 
    
    dist = angleSetpoint - currentAngle; 
    if (dist > 180){  
        dist = -(360 - dist);
    } else if (dist < -180){ 
        dist = (360 + dist);
    } 
    
    return dist;
} 

string DrivePath::repr(){ 
    stringstream ss; 
    ss << "Path " << turningFirst << "," << intaking << "["; 
    for (double setpoint : setpoints){ 
       ss << setpoint << ","; 
    } 
    ss << "]"; 
    return ss.str(); 
}

//--------------------------------------- 
// PURSUE A CERTAIN (X,Y) POSITION USING ONE OF THREE PATH TYPES

void DriveToSetpoint::start()
{

    double startingX = drivebaseRef.get<double>("Pos_X");
    double startingY = drivebaseRef.get<double>("Pos_Y");

    double overallDist = hypot(startingX - setpointX, startingY - setpointY); 
    
    double overallAngle = fmod((atan2(startingY - setpointY, startingX - setpointX) / M_PI * 180 + 180), 360); 

    double xDist = setpointX - startingX;
    double yDist = setpointY - startingY;

    switch (pathType)
    {
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

    if (endingAngle != -1 && pathType != PathType::EUCLIDEAN)
        setpoints.push_back(endingAngle);

    numOfOperations = setpoints.size() - 1;
} 

string DriveToSetpoint::repr(){ 
   stringstream ss;   
   ss << "Reach " << setpointX << "," << setpointY << "," << pathType;
   return ss.str();
}

//--------------------------------------- 
// TURN THE ROBOT SO THAT IT S FACING A CERTAIN (X,Y) POSITION

void TurnToSetpoint::start(){ 
   DriveToSetpoint::start(); 
   setpoints.pop_back(); 
   numOfOperations--;
};   

string TurnToSetpoint::repr(){ 
   stringstream ss;  
   ss << "Face " << setpointX << "," << setpointY;
   return ss.str();
}

//  

void FlatAlignWithX::start(){  

   DriveToSetpoint::start(); 
   double angleDiff = fabs(drivebaseRef.get<double>("Angle_Degrees_CCW") - setpoints.at(0)); 
    
   if (angleDiff > 180){  
     angleDiff = 360 - angleDiff;
   } 
   
   if (angleDiff > 90){ 
     setpoints[0] = fmod(setpoints[0] + 180, 360); 
     setpoints[1] = setpoints[1] * -1;
   }
   
   setpoints.pop_back();  
   setpoints.pop_back();
   numOfOperations -= 2; 

};    


void FlatAlignWithY::start(){   

   DriveToSetpoint::start();  
   double angleDiff = fabs(drivebaseRef.get<double>("Angle_Degrees_CCW") - setpoints.at(0)); 
    
   if (angleDiff > 180){  
     angleDiff = 360 - angleDiff;
   } 
   
   if (angleDiff > 90){ 
     setpoints[0] = fmod(setpoints[0] + 180, 360); 
     setpoints[1] = setpoints[1] * -1;
   }  

   setpoints.pop_back();  
   setpoints.pop_back();
   numOfOperations -= 2;
};

// 

void SlantedAlignWithX::start(){   
   double heading = drivebaseRef.get<double>("Angle_Degrees_CCW") / 360.0 * (2 * M_PI);  
   double xPos = drivebaseRef.get<double>("Pos_X");  

   double xDiff = setpointX - xPos;    
   double dist = xDiff / cos(heading);   
   
   setpoints.push_back(dist);
   numOfOperations += 1;
}  

void SlantedAlignWithY::start(){   
   double heading = drivebaseRef.get<double>("Angle_Degrees_CCW") / 360.0 * (2 * M_PI);  
   double yPos = drivebaseRef.get<double>("Pos_Y");   

   double yDiff = setpointY - yPos;  
   double dist = yDiff / sin(heading);  
   
   setpoints.push_back(dist);
   numOfOperations += 1;
} 

//

void CloseDistance::start(){ 
   DriveToSetpoint::start();  
   setpoints[1] = setpoints[1] - offset;
   operationsIndex++;
};   

//--------------------------------------- 
// DRIVES THE ROBOT FORWARD OR BACKWARD AT A CERTAIN SPEED PERCENTAGE

void DriveForwardForTime::start()
{ 
    RobotState::manuallyModifyState("intaking", intaking);
    startingTime = Brain.Timer.time(vex::msec);     
};

void DriveForwardForTime::periodic()
{
    drivebaseRef.manualPercentageDrive(percentage * 100);  
    intakeRef.periodic();
};

bool DriveForwardForTime::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void DriveForwardForTime::end()
{  
    RobotState::manuallyModifyState("intaking", false);
    drivebaseRef.stop(); 
    intakeRef.stop();
}   

string DriveForwardForTime::repr(){ 
    stringstream ss; 
    ss << "DFFT " << percentage <<  "," << timeDuration; 
    return ss.str();
}  

//----------------------------------------------------- 

void Calibrate::start()
{ 
    RobotState::manuallyModifyState("calibrating", true); //Asuume facing somewhat towards wall
    drivebaseRef.setCalibratingStructure(wall);  
    startingTime = Brain.Timer.time(vex::msec);   
    
};

void Calibrate::periodic()
{
    drivebaseRef.manualPercentageDrive(percentage * 100);  
};

bool Calibrate::isOver()
{
    return (Brain.Timer.time(vex::msec) - startingTime) >= durationMilliseconds; //Set off by drivebase's internal systems
}

void Calibrate::end()
{   
    RobotState::manuallyModifyState("k_calibrating", true);
    drivebaseRef.stop();
}   





//--------------------------------------- 
// INTAKE CUBES FOR STORAGE FOR A CERTAIN AMOUNT OF TIME

void IntakeCubes::start()
{ 
    RobotState::manuallyModifyState("intaking", true); 
    startingTime = Brain.Timer.time(vex::msec);
};

void IntakeCubes::periodic()
{
    intakeRef.periodic();
};

bool IntakeCubes::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void IntakeCubes::end()
{
    RobotState::manuallyModifyState("intaking", false); 
    intakeRef.stop();
} 

string IntakeCubes::repr(){ 
    stringstream ss;  
    ss << "Intake " << timeDuration;
    return ss.str(); 
}

//--------------------------------------- 
// SCORE ON EITHER A HIGH, MID, OR LOW GOAL FOR A CERTAIN AMOUNT OF TIME

void ScoreOnGoal::start()
{
    switch (goal)
    {
    case 1:
        RobotState::manuallyModifyState("scoring_high", true);
        break;
    case 2:
        RobotState::manuallyModifyState("scoring_mid", true);
        break;
    case 3:
        RobotState::manuallyModifyState("scoring_low", true);
        break;
    default:
        break;
    }
    startingTime = Brain.Timer.time();
};

void ScoreOnGoal::periodic()
{
    indexerRef.periodic();
    intakeRef.periodic();
}

bool ScoreOnGoal::isOver()
{
    return Brain.Timer.time() - startingTime >= timeDuration;
}

void ScoreOnGoal::end()
{
    switch (goal)
    {
    case 1:
        RobotState::manuallyModifyState("scoring_high", false);
        break;
    case 2:
        RobotState::manuallyModifyState("scoring_mid", false);
        break;
    case 3:
        RobotState::manuallyModifyState("scoring_low", false);
        break;
    default:
        break;
    } 
    intakeRef.stop(); 
    indexerRef.stop(); 
    
} 

string ScoreOnGoal::repr(){ 
    stringstream ss;  
    ss << "Score " << goal << "," << timeDuration; 
    return ss.str();
}

//------------------------------------ 

bool DiscriminateScoreOnHighGoal::isOver(){  
    //300 ms grace period
    return (Brain.Timer.time(vex::msec) - startingTime > 300) && ( (!indexerRef.get<bool>("detects_block")) || (!indexerRef.get<bool>("detects_correct_color")) );
}


//--------------------------------------- 
// EXTENDS OR RETRACTS THE MATCHLOADER MECHANISM

void DeployMatchloader::start()
{
    RobotState::manuallyModifyState("matchloader_out", isOut);
}

void DeployMatchloader::periodic()
{
    matchLoaderRef.periodic();
    ran = true;
}

bool DeployMatchloader::isOver()
{
    return ran;
}

void DeployMatchloader::end()
{
    return;
} 

string DeployMatchloader::repr(){ 
   stringstream ss;  
   ss << "ML" << "," << isOut; 
   return ss.str(); 
}; 

// EXTENDS OR RETRACTS THE DESCORE MECHANISM

void DeployDescore::start()
{
    RobotState::manuallyModifyState("descore_in", isOut);
}

void DeployDescore::periodic()
{
    hooksRef.periodic();
    ran = true;
}

bool DeployDescore::isOver()
{
    return ran;
}

void DeployDescore::end()
{
    return;
} 


//--------------------------------------- 
// BLOCKS THE ROUTINE FROM RUNNING FOR A CERTAIN AMOUNT OF TIME

void WaitFor::start()
{
    startTime = Brain.Timer.time();
}

void WaitFor::periodic()
{
    return;
}

bool WaitFor::isOver()
{
    return Brain.Timer.time() - startTime >= timeDuration;
}

void WaitFor::end()
{
    return;
}  

string WaitFor::repr(){ 
   stringstream ss; 
   ss << "Wait " << timeDuration; 
   return ss.str();
}
//--------------------------------------- 
/*
void AnchorHeading::start(){ 
  Drivebase::globalRef->anchorAngle();
};  

void AnchorHeading::periodic(){ 
  return;
} 

bool AnchorHeading::isOver(){ 
  return true;
} 

void AnchorHeading::end(){ 
  return;
}  
*/
//--------------------------------------- 

void DisengageHighGoal::start()
{ 
    RobotState::manuallyModifyState("scoring_high", true);
    startingTime = Brain.Timer.time(vex::msec);  
};

void DisengageHighGoal::periodic()
{
    drivebaseRef.manualPercentageDrive(-percentage * 100);  
    intakeRef.periodic(); 
    indexerRef.periodic();
};

bool DisengageHighGoal::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void DisengageHighGoal::end()
{  
    RobotState::manuallyModifyState("scoring_high", false);
    drivebaseRef.stop(); 
    intakeRef.stop(); 
    indexerRef.stop();
}   


//--------------------------------------- 
// MODIFIES THE STATE OF THE ROBOT 

void ModifyRobotState::start()
{
   RobotState::manuallyModifyState(entryKey, value); 
   //Drivebase::globalRef->anchorAngle();
}

void ModifyRobotState::periodic()
{
    return;
}

bool ModifyRobotState::isOver()
{
    return true;
}

void ModifyRobotState::end()
{
    return;
}

string ModifyRobotState::repr(){ 
    stringstream ss; 
    ss << "Transform " << entryKey << "," << value;
    return ss.str();
} 


CommandInterface* DriveToLocation(int zoneIndex, double dist, PathType pathType, bool intaking){ 
  std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(dist); 
  return DriveToSetpoint::getCommand(setpoint[0], setpoint[1], -1, pathType, intaking);
}

CommandInterface* TurnToLocation(int zoneIndex){ 
  std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(0); 
  return TurnToSetpoint::getCommand(setpoint[0], setpoint[1]);
}  

CommandInterface* TurnToLocation(int zoneIndex, double dist){ 
  std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(dist); 
  return TurnToSetpoint::getCommand(setpoint[0], setpoint[1]);
};

CommandInterface* CloseDistanceBetween(int zoneIndex, double dist, double offset, bool intaking){
  std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(dist);  
  return CloseDistance::getCommand(setpoint[0], setpoint[1], intaking, offset);
};



