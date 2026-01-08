#include "commands.h"
#include <sstream>

//--------------------------------------- 
// A COMBINATION OF DRIVING FORWARD AND TURNING COMMANDS 

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
    intakeRef.periodic(); 
}

bool DrivePath::isOver()
{
    return operationsIndex > numOfOperations;
}

void DrivePath::end()
{
   RobotState::manuallyModifyState("intaking", false);
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
    
}

void DrivePath::drive()
{  
    double output; 
    TrapezoidalSetpoint setpoint = drivingProfile->generateSetpoint(Brain.Timer.time(vex::sec));  
    output = setpoint.velocity;
    if (!isGoingForward)
        output *= -1;
    drivebaseRef.manualDriveForward(output); 
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
    drivebaseRef.manualTurnClockwise(output);
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

//--------------------------------------- 
// DRIVES THE ROBOT FORWARD OR BACKWARD AT A CERTAIN SPEED PERCENTAGE

void DriveForwardForTime::start()
{ 
    RobotState::manuallyModifyState("intaking", intaking);
    startingTime = Brain.Timer.time(vex::msec);   
};

void DriveForwardForTime::periodic()
{
    drivebaseRef.arcadeDrive(percentage * 100, 0);  
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
}   

string DriveForwardForTime::repr(){ 
    stringstream ss; 
    ss << "DFFT " << percentage <<  "," << timeDuration; 
    return ss.str();
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

string DeployMatchloader::repr(){ 
   stringstream ss;  
   ss << "ML" << "," << isOut; 
   return ss.str(); 
};


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
// MODIFIES THE STATE OF THE ROBOT 

void ModifyRobotState::start()
{
   RobotState::manuallyModifyState(entryKey, value);
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



