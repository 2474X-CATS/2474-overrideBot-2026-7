#include "commands.h"
#include <sstream>

//---------------------------------------
// A COMBINATION OF DRIVING FORWARD AND TURNING COMMANDS

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
    indexerRef.periodic();
}

bool DrivePath::isOver()
{
    return operationsIndex > numOfOperations;
}

void DrivePath::end()
{
    RobotState::manuallyModifyState("intaking", false); 
    //RobotState::manuallyModifyState("odometry_enabled", true);
    drivebaseRef.stop();
    intakeRef.stop(); 
    indexerRef.stop();
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
    return (Brain.Timer.time() - drivingProfile->getStartTime()) >= drivingProfile->getTotalDuration();
}

void DrivePath::initializeDrive()
{ 

    double timestamp = Brain.Timer.time(); 

    isGoingForward = setpoints.at(operationsIndex) > 0;
    drivingProfile = new TrapezoidalMotionProfile(drivebaseRef.getMotionConstants(), abs(setpoints.at(operationsIndex))); 

    if (alphaController == nullptr){ 
        alphaController = new errorcontroller(drivebaseRef.getCorrectiveLinearPID()); 
    } else { 
        alphaController->deactivate();
    }

    referenceAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");

    startingPoint[0] = drivebaseRef.get<double>("Pos_X");
    startingPoint[1] = drivebaseRef.get<double>("Pos_Y");

    lastPoint[0] = startingPoint[0];
    lastPoint[1] = startingPoint[1]; 
    
    //RobotState::manuallyModifyState("odometry_enabled", true);

    drivingProfile->setLastTimestamp(timestamp);  
    alphaController->setLastTimestamp(timestamp); 

}

void DrivePath::drive()
{
    TrapezoidalSetpoint setpoint = drivingProfile->generateSetpoint(Brain.Timer.time());
    
    double correctiveAlpha = alphaController->calculate(drivebaseRef.get<double>("Linear_Velocity"), Brain.Timer.time());
    double output = setpoint.velocity;
    
    if (!isGoingForward)
        output *= -1; 
    
    alphaController->setReference(output);
    output += correctiveAlpha; 

    drivebaseRef.manualDriveForward(output, -1);

    lastPoint[0] = drivebaseRef.get<double>("Pos_X");
    lastPoint[1] = drivebaseRef.get<double>("Pos_Y");
}

bool DrivePath::isTurnOver()
{
    return turnPID->atSetpoint(getAngularError()) || setpoints.at(operationsIndex) == -1;
}

void DrivePath::initializeTurn()
{ 
    //RobotState::manuallyModifyState("odometry_enabled", false); 

    turnPID = new pidcontroller(drivebaseRef.getTurningPID(), 0);
    turnPID->setLastTimestamp(Brain.Timer.time());
}

void DrivePath::turn()
{
    double output = turnPID->calculate(getAngularError(), Brain.Timer.time());

    if (RobotState::getStateOf("is_counterclockwise"))
        drivebaseRef.manualTurnCounterclockwise(output);
    else
    {
        drivebaseRef.manualTurnCounterclockwise(-output);
    }
}

double DrivePath::getAngularError()
{
    double currentAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");
    double angleSetpoint = setpoints.at(operationsIndex);
    double dist;
    dist = angleSetpoint - currentAngle;
    if (dist > 180)
    {
        dist = -(360 - dist);
    }
    else if (dist < -180)
    {
        dist = (360 + dist);
    }

    return dist;
}

string DrivePath::repr()
{
    stringstream ss;
    ss << "Path " << turningFirst << "," << intaking << "[";
    for (double setpoint : setpoints)
    {
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

string DriveToSetpoint::repr()
{
    stringstream ss;
    ss << "Reach " << setpointX << "," << setpointY << "," << pathType;
    return ss.str();
}

//---------------------------------------
// TURN THE ROBOT SO THAT IT S FACING A CERTAIN (X,Y) POSITION

void TurnToSetpoint::start()
{
    DriveToSetpoint::start();
    setpoints.pop_back();
    numOfOperations--;
};

string TurnToSetpoint::repr()
{
    stringstream ss;
    ss << "Face " << setpointX << "," << setpointY;
    return ss.str();
}

//

void FlatAlignWithX::start()
{

    DriveToSetpoint::start();
    double angleDiff = fabs(drivebaseRef.get<double>("Angle_Degrees_CCW") - setpoints.at(0));

    if (angleDiff > 180)
    {
        angleDiff = 360 - angleDiff;
    }

    if (angleDiff > 90)
    {
        setpoints[0] = fmod(setpoints[0] + 180, 360);
        setpoints[1] = setpoints[1] * -1;
    }

    setpoints.pop_back();
    setpoints.pop_back();
    numOfOperations -= 2;
};

void FlatAlignWithY::start()
{

    DriveToSetpoint::start();
    double angleDiff = fabs(drivebaseRef.get<double>("Angle_Degrees_CCW") - setpoints.at(0));

    if (angleDiff > 180)
    {
        angleDiff = 360 - angleDiff;
    }

    if (angleDiff > 90)
    {
        setpoints[0] = fmod(setpoints[0] + 180, 360);
        setpoints[1] = setpoints[1] * -1;
    }

    setpoints.pop_back();
    setpoints.pop_back();
    numOfOperations -= 2;
};

// 

double ParkClear::PARK_ZONE_WIDTH = 19 * 25.4; 

void ParkClear::start(){   

    RobotState::manuallyModifyState("odometry_enabled", false); 
    double currentX = drivebaseRef.get<double>("Pos_X"); 
    double currentY = drivebaseRef.get<double>("Pos_Y"); 
    double heading = toRadians(drivebaseRef.get<double>("Angle_Degrees_CCW"));
    
    if (stay){ 
        endingPoint[0] = currentX; 
        endingPoint[1] = currentY; 
    } else { 
        endingPoint[0] = currentX + (PARK_ZONE_WIDTH * cos(heading)); 
        endingPoint[1] = currentY + (PARK_ZONE_WIDTH * sin(heading));
    } 
    
    startingTimestamp = Brain.Timer.time();
} 

void ParkClear::periodic(){ 
    if (Brain.Timer.time() - startingTimestamp < phases.at(index).duration){  
       if (!phaseInitialized){  
          RobotState::manuallyModifyState("intaking", phases.at(index).intaking);
          phaseInitialized = true;
       }
       drivebaseRef.manualPercentageDrive(phases.at(index).output * 100);   
       intakeRef.periodic();
    } else {  
       intakeRef.stop();
       index++;  
       phaseInitialized = false; 
       startingTimestamp = Brain.Timer.time();
    }
}

bool ParkClear::isOver(){ 
    return index == numPhases;
} 

void ParkClear::end(){  
    drivebaseRef.stop();  
    drivebaseRef.scheduleSetpoint(endingPoint[0], endingPoint[1]);   
    RobotState::manuallyModifyState("odometry_enabled", true);
} 

//

void SlantedAlignWithX::start()
{
    double heading = drivebaseRef.get<double>("Angle_Degrees_CCW") / 360.0 * (2 * M_PI);
    double xPos = drivebaseRef.get<double>("Pos_X");

    double xDiff = setpointX - xPos;
    double dist = xDiff / cos(heading);

    setpoints.push_back(dist);
    numOfOperations += 1;
}

void SlantedAlignWithY::start()
{
    double heading = drivebaseRef.get<double>("Angle_Degrees_CCW") / 360.0 * (2 * M_PI);
    double yPos = drivebaseRef.get<double>("Pos_Y");

    double yDiff = setpointY - yPos;
    double dist = yDiff / sin(heading);

    setpoints.push_back(dist);
    numOfOperations += 1;
}

//

void CloseDistance::start()
{
    DriveToSetpoint::start();
    setpoints[1] = setpoints[1] - offset;
    operationsIndex++;
};

//--------------------------------------- 

FollowCirclePath::FollowCirclePath(Drivebase& drivebase, Intake& intake, Indexer& indexer, vector<BiarcEnum> setpoints, bool intaking) : 
Command<Drivebase, Intake, Indexer>(drivebase, intake, indexer), 
drivebaseRef(drivebase),  
intakeRef(intake),  
indexerRef(indexer),
setpoints(setpoints), 
intaking(intaking)
{ 
  for (BiarcEnum biarc : setpoints){  
    segments.push_back(CirclePath(biarc)); 
    nSegments++;
  } 
} 


void FollowCirclePath::start(){ 
    
    double maxVelocity = drivebaseRef.getMotionConstants().maxVelocity;
    for (int i = 0; i < nSegments; i++){ 
        
       if (i > 0){ 
         segments[i-1].setEndingVelocity(maxVelocity); 
         segments[i].setStartingVelocity(maxVelocity); 
       } 
    }  
    RobotState::manuallyModifyState("intaking", intaking); 
} 

void FollowCirclePath::periodic(){  
    double timestamp = Brain.Timer.time();
    if (!initialized){  
        segments.at(index).activate(drivebaseRef.getPathMetadata()); 
        segments.at(index).init(timestamp);   
        initialized = true;
    } else { 
        if (segments.at(index).completed(timestamp)){ 
            initialized = false; 
            index++;
        } else { 
            PathFrameOutput output = segments.at(index).calculateFrameOutput(  
                drivebaseRef.getFrameData()
            ); 
            drivebaseRef.manualDriveWithCurvature(output.linearVelocity, output.angularVelocity); 
        }
    }  
    intakeRef.periodic(); 
    indexerRef.periodic();
} 

bool FollowCirclePath::isOver(){ 
    return index >= nSegments;
} 

void FollowCirclePath::end(){
    RobotState::manuallyModifyState("intaking", false);  
    intakeRef.stop();  
    indexerRef.stop();
    drivebaseRef.stop();
}


//--------------------------------------- 

void FollowSplinePath::start(){ 
    path = new HomingPath(waypoints, drivebaseRef.getPathMetadata()); 
    path->init(Brain.Timer.time()); 
} 

void FollowSplinePath::periodic(){  

    PathFrameOutput output = path->calculateFrameOutput( 
        drivebaseRef.getFrameData()
    ); 

    drivebaseRef.manualDriveWithCurvature(output.linearVelocity, output.angularVelocity);
} 

bool FollowSplinePath::isOver(){ 
    return path->completed(drivebaseRef.get<double>("Pos_X"), drivebaseRef.get<double>("Pos_Y"));
} 

void FollowSplinePath::end(){ 
    drivebaseRef.stop();
}


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

string DriveForwardForTime::repr()
{
    stringstream ss;
    ss << "DFFT " << percentage << "," << timeDuration;
    return ss.str();
}

//-----------------------------------------------------

void Calibrate::start()
{
    RobotState::manuallyModifyState("calibrating", true); // Asuume facing somewhat towards wall
    drivebaseRef.setCalibratingStructure(wall);
    startingTime = Brain.Timer.time(vex::msec);
};

void Calibrate::periodic()
{
    drivebaseRef.manualPercentageDrive(percentage * 100);
};

bool Calibrate::isOver()
{
    return (Brain.Timer.time(vex::msec) - startingTime) >= durationMilliseconds; // Set off by drivebase's internal systems
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
    indexerRef.periodic();
};

bool IntakeCubes::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void IntakeCubes::end()
{
    RobotState::manuallyModifyState("intaking", false);
    intakeRef.stop(); 
    indexerRef.stop();
}

string IntakeCubes::repr()
{
    stringstream ss;
    ss << "Intake " << timeDuration;
    return ss.str();
}

//---------------------------------------
// SCORES BLOCKS ON A GOAL

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
    return Brain.Timer.time() - startingTime >= timeDuration; //|| (indexerRef.get<bool>("detects_block") && indexerRef.get<bool>("detects_wrong_color")); 
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

string DeployMatchloader::repr()
{
    stringstream ss;
    ss << "ML" << "," << isOut;
    return ss.str();
};

// EXTENDS OR RETRACTS THE DESCORE MECHANISM

void DeployDescore::start()
{
    RobotState::manuallyModifyState("descore_f_in", isOut); 
    RobotState::manuallyModifyState("descore_b_in", isOut);
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

string WaitFor::repr()
{
    stringstream ss;
    ss << "Wait " << timeDuration;
    return ss.str();
}

//---------------------------------------
// MODIFIES THE STATE OF THE ROBOT

void ModifyRobotState::start()
{
    RobotState::manuallyModifyState(entryKey, value);
    // Drivebase::globalRef->anchorAngle();
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

string ModifyRobotState::repr()
{
    stringstream ss;
    ss << "Transform " << entryKey << "," << value;
    return ss.str();
}

CommandInterface *DriveToLocation(int zoneIndex, double dist, PathType pathType, bool intaking)
{
    std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(dist);
    return DriveToSetpoint::getCommand(setpoint[0], setpoint[1], -1, pathType, intaking);
}

CommandInterface *TurnToLocation(int zoneIndex)
{
    std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(0);
    return TurnToSetpoint::getCommand(setpoint[0], setpoint[1]);
}

CommandInterface *TurnToLocation(int zoneIndex, double dist)
{
    std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(dist);
    return TurnToSetpoint::getCommand(setpoint[0], setpoint[1]);
};

CommandInterface *CloseDistanceBetween(int zoneIndex, double dist, double offset, bool intaking)
{
    std::array<double, 2> setpoint = Drivebase::globalRef->getLocation(zoneIndex)->getProjectedSetpoint(dist);
    return CloseDistance::getCommand(setpoint[0], setpoint[1], intaking, offset);
};
