#include "commands.h"

////////////////////////////////////////////////////////////

double DrivePath::ARBITRARY_SPEED = 350; 

void DrivePath::start()
{
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
    hoodRef.periodic();
    intakeRef.periodic();
    indexerRef.periodic();
}

bool DrivePath::isOver()
{
    return operationsIndex > numOfOperations;
}

void DrivePath::end()
{
    RobotState::manuallyModifyState("intaking_to_hopper", false);
}

void DrivePath::drivePeriodic()
{
    if (!initialized)
    {
        RobotState::manuallyModifyState("intaking_to_hopper", intaking);
        initializeDrive();
        initialized = true;
    }
    else
    {
        if (isDriveOver())
        {
            operationsIndex += 1;
            initialized = false;
            drivebaseRef.stop();
            delete drivePID;
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
        RobotState::manuallyModifyState("intaking_to_hopper", false);
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
    return Brain.Timer.time(vex::sec) - lastDriveTimestamp >= timeDuration;//drivePID->atSetpoint(getDrivingError());
}

void DrivePath::initializeDrive()
{
    
    isGoingForward = setpoints.at(operationsIndex) > 0;

    startingPoint[0] = drivebaseRef.get<double>("Pos_X");
    startingPoint[1] = drivebaseRef.get<double>("Pos_Y");
   
    drivePID = new pidcontroller(drivebaseRef.getPowerPID(), fabs(setpoints.at(operationsIndex)));
    drivePID->setLastTimestamp(Brain.Timer.time()); 

    lastDriveTimestamp = Brain.Timer.time(vex::sec);  
    timeDuration = fabs(setpoints.at(operationsIndex)) / ARBITRARY_SPEED;  

}

void DrivePath::drive()
{  
    double output; 
    //output = fabs(drivePID->calculate(getDrivingError(), Brain.Timer.time())); 
    output = ARBITRARY_SPEED; 
    if (!isGoingForward)
        output *= -1;
    drivebaseRef.manualDriveForward(output); 
}

double DrivePath::getDrivingError()
{
    return hypot(drivebaseRef.get<double>("Pos_X") - startingPoint[0], drivebaseRef.get<double>("Pos_Y") - startingPoint[1]);
}

bool DrivePath::isTurnOver()
{
    return turnPID->atSetpoint(getAngularError());
}

void DrivePath::initializeTurn()
{

    double startAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");

    double angleSetpoint = setpoints.at(operationsIndex);

    double normalDist = startAngle > angleSetpoint ? (360 - startAngle) + angleSetpoint : angleSetpoint - startAngle;
    isCounterClockwise = true;

    if (normalDist > 180)
    {
        isCounterClockwise = false;
        normalDist = 360 - normalDist;
    }

    turnPID = new pidcontroller(drivebaseRef.getTurningPID(), 0);
    turnPID->setLastTimestamp(Brain.Timer.time());
}

void DrivePath::turn()
{
    double output = turnPID->calculate(getAngularError(), Brain.Timer.time());
    if (isCounterClockwise)
    {
        output *= -1;
    }
    drivebaseRef.manualTurnClockwise(-output);
}

double DrivePath::getAngularError()
{
    double currentAngle = drivebaseRef.get<double>("Angle_Degrees_CCW");
    double angleSetpoint = setpoints.at(operationsIndex);

    double dist = currentAngle > angleSetpoint ? (360 - currentAngle) + angleSetpoint : angleSetpoint - currentAngle;

    if (!isCounterClockwise)
    {
        dist = (360 - dist);
    }

    return dist;
}

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

//////////////////////////////////////////////////////////// 

void TurnToSetpoint::start(){ 
   DriveToSetpoint::start(); 
   setpoints.pop_back(); 
   numOfOperations--;
};  

void CloseDistanceBetweenSetpoint::start(){ 
   DriveToSetpoint::start(); 
   operationsIndex++; 
   setpoints[1] = setpoints[1] - distFrom;
}; 

////////////////////////////////////////////////////////////

void DriveForwardForTime::start()
{
    drivebaseRef.setSpeedFactor(1); 
    startingTime = Brain.Timer.time();  
    if (intaking)
      RobotState::manuallyModifyState("intaking_to_hopper", true); 
    else { 
      RobotState::manuallyModifyState("intaking_to_hopper", false);
    }
};

void DriveForwardForTime::periodic()
{
    drivebaseRef.arcadeDrive(percentage * 100, 0); 
    intakeRef.periodic(); 
    indexerRef.periodic(); 
    hoodRef.periodic(); 
};

bool DriveForwardForTime::isOver()
{
    return Brain.Timer.time() - startingTime >= timeDuration;
}

void DriveForwardForTime::end()
{ 
    RobotState::manuallyModifyState("intaking_to_hopper", false);
    drivebaseRef.stop();
    drivebaseRef.setSpeedFactor(0.85);
} 
//////////////////////////////////////////////////////////////////////////////// 

void TrapezoidalDriveForward::start(){ 
    profile = TrapezoidalMotionProfile(drivebaseRef.getMotionConstants(), distance, Brain.Timer.time(vex::sec)); 
};  

void TrapezoidalDriveForward::periodic(){ 
    TrapezoidalSetpoint currentSetpoint = profile.generateSetpoint(Brain.Timer.time(vex::sec)); 
    drivebaseRef.manualDriveForward(currentSetpoint.velocity);  
}; 

bool TrapezoidalDriveForward::isOver(){ 
    return (Brain.Timer.time(vex::sec) - profile.getStartTime()) >= profile.getTotalDuration(); 
}; 

void TrapezoidalDriveForward::end(){ 
    drivebaseRef.stop(); 
}; 


////////////////////////////////////////////////////////////////////////////////
void IntakeToHopper::start()
{
    RobotState::manuallyModifyState("intaking_to_hopper", true);
    startingTime = Brain.Timer.time(vex::msec);
};

void IntakeToHopper::periodic()
{
    hoodRef.periodic();
    indexerRef.periodic();
    intakeRef.periodic();
};

bool IntakeToHopper::isOver()
{
    return Brain.Timer.time(vex::msec) - startingTime >= timeDuration;
}

void IntakeToHopper::end()
{
    RobotState::manuallyModifyState("intaking_to_hopper", false);
}

////////////////////////////////////////////////////////////

void ScoreOnGoal::start()
{
    // Top: 1
    // Mid: 2
    // Low: 3
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
    hoodRef.periodic();
    hopperRef.periodic();
    indexerRef.periodic();
    intakeRef.periodic();
}

bool ScoreOnGoal::isOver()
{
    return Brain.Timer.time() - startingTime >= timeDuration;
}

void ScoreOnGoal::end()
{
    hopperRef.stop();
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
}

////////////////////////////////////////////////////////////

void DeployMatchloader::start()
{
    if (isOut)
        RobotState::manuallyModifyState("matchloader_out", true);
    else
    {
        RobotState::manuallyModifyState("matchloader_out", false);
    }
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

////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////

CommandInterface *DriveLinear(double distance, bool intaking)
{
    return DrivePath::getCommand({distance}, false, intaking);
};

CommandInterface *TurnToHeading(double angle)
{
    return DrivePath::getCommand({angle}, true, false);
};

CommandInterface *DriveToPoint(double setpointX, double setpointY, PathType pathType, bool intaking)
{
    return DriveToSetpoint::getCommand(setpointX, setpointY, -1, pathType, intaking);
};

CommandInterface *AlignWithLocation(int locationIndex, double distance, PathType pathType, bool intaking)
{
    array<double, 2> setpoint = Drivebase::getLocation(locationIndex)->getProjectedSetpoint(distance);
    return DriveToSetpoint::getCommand(setpoint[0], setpoint[1], Drivebase::getLocation(locationIndex)->getPerfectEntranceAngle(), pathType, intaking);
};

CommandInterface *FaceLocation(int locationIndex)
{
    array<double, 2> setpoint = Drivebase::getLocation(locationIndex)->getProjectedSetpoint(0);
    return TurnToSetpoint::getCommand(setpoint[0], setpoint[1]);
};

CommandInterface *Score(Goal_Pos pos, double duration)
{
    return ScoreOnGoal::getCommand(pos, duration);
};

CommandInterface *IntakeCubes(double duration)
{
    return IntakeToHopper::getCommand(duration);
};

CommandInterface *EnableMatchloader(bool out)
{
    return DeployMatchloader::getCommand(out);
};

CommandInterface *Wait(double duration)
{
    return WaitFor::getCommand(duration);
};

CommandInterface* RamForward(double percentage, double duration, bool intaking){ 
    return DriveForwardForTime::getCommand(percentage, duration, intaking);
};   

CommandInterface* GetWithinDistOfSetpoint(int locationIndex, double distFrom){  
    array<double, 2> setpoint = Drivebase::getLocation(locationIndex)->getProjectedSetpoint(0);
    return CloseDistanceBetweenSetpoint::getCommand(setpoint[0], setpoint[1], distFrom);
}

/////////////////////////////////////////////////////////////////////
