#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "subsystems/wedge.h"
#include "subsystems/hooks.h" 
#include "commands.h"
#include "gui/autonInit.h"

using namespace vex;

competition Competition;
Robot robot; 


void runTelemetry()
{
  robot.runTelemetryThread(false); 
} 

void pauseUntilReady(){ 
  while (!RobotState::getStateOf("ready")){ 
    this_thread::yield();
  }
} 

void awaitStartingSignal(){ 
  return;
}

int scheduleCallbacks(){   
   
  //Add a pause where you press a button to start    

  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });   

  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  return 0;
}



void configurateAutonomous(vector<CommandInterface*> leftSide, vector<CommandInterface*> rightSide){ 
  RobotState::manuallyModifyState("configurable", true);  

  ColorPicker colorChooser = ColorPicker(220-125,200-50);  
  SidePicker sidePicker = SidePicker(360-125,200-50);  
  ExitBlock(160, 65);
  Sprite::frameLoop();   
 
  RobotState::manuallyModifyState("starting_left", sidePicker.getIsLeft()); 

  if (sidePicker.getIsLeft()){ 
    robot.setAutonomousCommand(leftSide);   
  } else { 
    robot.setAutonomousCommand(rightSide);   
  }    

  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}  

void testDrive()
{  
  RobotState::manuallyModifyState("ready", true); 
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
} 

void testAuto(vector<CommandInterface*> auton){  
  robot.setAutonomousCommand(auton); 
  RobotState::manuallyModifyState("ready", true);
  thread telemThread = thread(runTelemetry);   
  robot.autonControl();                      
  robot.driverControl();
} 

void startCommandSkillsMatch(vector<CommandInterface*> commandGroup){  
  robot.setAutonomousCommand(commandGroup);  
  scheduleCallbacks(); 
  robot.runTelemetryThread(false);
}

void startCommandCompetitiveMatch(vector<CommandInterface *> leftCommandGroup, vector<CommandInterface *> rightCommandGroup)
{
  configurateAutonomous(leftCommandGroup, rightCommandGroup);
  scheduleCallbacks(); 
  robot.runTelemetryThread(false); 
}

void driveCommandMatch(vector<CommandInterface *> leftCommandGroup, vector<CommandInterface*> rightCommandGroup)
{  
  configurateAutonomous(leftCommandGroup, rightCommandGroup);    
  thread telemThread = thread(runTelemetry);   
  //Add a pause where you press a button to start
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  robot.autonControl();                      
  robot.driverControl();                     
}


vector<CommandInterface*> closed_side_left(){ 
   return { 
       TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.875), 
       CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.875, 250, true), //DrivePath::getCommand({TILE_SIZE_MM * 0.4}, false, false),  
       DeployMatchloader::getCommand(true), 
       DriveForwardForTime::getCommand(0.25, 500, true),  
       DeployMatchloader::getCommand(false),
       IntakeCubes::getCommand(500), 
       ModifyRobotState::getCommand("is_drive_inverted", true), 
       TurnToLocation(Zones::NAT_MID),    
       DrivePath::getCommand({TILE_SIZE_MM * 0.65}, false, false),  
       ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),    
       ModifyRobotState::getCommand("is_drive_inverted", false),    
       DrivePath::getCommand({1130 + ROBOT_LENGTH_MM / 2}, false, false),
       TurnToLocation(Zones::NAT_ML_LEFT),  
       DeployMatchloader::getCommand(true), 
       WaitFor::getCommand(500), 
       DriveForwardForTime::getCommand(0.225, 850, true),
       IntakeCubes::getCommand(500),   
       ModifyRobotState::getCommand("is_drive_inverted", true),   
       TurnToLocation(Zones::NAT_HIGH_LEFT),  
       CloseDistanceBetween(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM / 2 - 100, 0, true),
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000) 
   };
} 

vector<CommandInterface*> closed_side_right(){ 
   return {  
       TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.875),  
       CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.875, 250, true),
       DeployMatchloader::getCommand(true), 
       DriveForwardForTime::getCommand(0.4, 500, true),  
       IntakeCubes::getCommand(500),   
       DriveToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM * 0.925, PathType::EUCLIDEAN, false),  
       TurnToLocation(Zones::NAT_ML_RIGHT),    
       DriveForwardForTime::getCommand(0.25, 500, true),
       IntakeCubes::getCommand(750),    
       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({100}, false, false),   
       DriveToLocation(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM / 2 - 100, PathType::EUCLIDEAN, false), 
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 3000), 
       DrivePath::getCommand({-300, 0, ROBOT_LENGTH_MM / 2 + 100, 90}, false, false), 
       DriveForwardForTime::getCommand(0.25, 1500, false)
   };
} 

vector<CommandInterface*> auto_skills(){ 
   return {   
       TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 1.1),  //Get cubes near low
       CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 1.1, 250, true),
       DeployMatchloader::getCommand(true), 
       DriveForwardForTime::getCommand(0.4, 500, true),  
       IntakeCubes::getCommand(500),       
       
       ModifyRobotState::getCommand("is_drive_inverted", true),
       DriveToLocation(Zones::NAT_HIGH_RIGHT, (ROBOT_LENGTH_MM / 2) + 250, PathType::EUCLIDEAN, false),  //Score low cubes on 4-high
       TurnToLocation(Zones::NAT_HIGH_RIGHT),    
       CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, 100, 0, false),   
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),  
       
       ModifyRobotState::getCommand("is_drive_inverted", false),  
       DriveToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM * 0.925, PathType::EUCLIDEAN, false), // Matchload cubes from 4
       TurnToLocation(Zones::NAT_ML_RIGHT), 
       DriveForwardForTime::getCommand(0.25, 500, true),
       IntakeCubes::getCommand(1500),    

       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({300}, false, false),  
       DeployMatchloader::getCommand(false),       
       DriveToSetpoint::getCommand((TILE_SIZE_MM * 5) + (ROBOT_WIDTH_MM / 2) + 250, (TILE_SIZE_MM * 4) + ((ROBOT_LENGTH_MM / 4) * 3), -1, PathType::MANHATTAN_XY, false),  
       DriveToLocation(Zones::FAR_HIGH_RIGHT, TILE_SIZE_MM * 0.925, PathType::MANHATTAN_XY, false), 
       TurnToLocation(Zones::FAR_HIGH_RIGHT), 
       CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, 100, 0, false),  
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), //Go to the other side (1) and score 
       DeployMatchloader::getCommand(true), 

       ModifyRobotState::getCommand("is_drive_inverted", false),  
       TurnToLocation(Zones::FAR_ML_RIGHT), 
       CloseDistanceBetween(Zones::FAR_ML_RIGHT, TILE_SIZE_MM * 0.925, 0, false),  //Matchload from other side (1)
       DriveForwardForTime::getCommand(0.25, 500, true),
       IntakeCubes::getCommand(1500), 

       ModifyRobotState::getCommand("is_drive_inverted", true),  
       DrivePath::getCommand({250}, false, false), 
       TurnToLocation(Zones::FAR_HIGH_RIGHT), 
       CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, 100, 0, false), // Score on high goal side (1)
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
       
       ModifyRobotState::getCommand("is_drive_inverted", false), 
       DrivePath::getCommand({250}, false, false),  
       DriveToLocation(Zones::FAR_ML_LEFT, TILE_SIZE_MM * 0.925, PathType::MANHATTAN_XY, false),  
       TurnToLocation(Zones::FAR_ML_LEFT),   
       DriveForwardForTime::getCommand(0.25, 500, true),
       IntakeCubes::getCommand(1500),  
       
       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({250}, false, false), 
       DeployMatchloader::getCommand(false), 
       DriveToSetpoint::getCommand(TILE_SIZE_MM / 2, TILE_SIZE_MM + (ROBOT_LENGTH_MM/2), -1, PathType::MANHATTAN_XY, false), 
       DriveToLocation(Zones::NAT_HIGH_LEFT, TILE_SIZE_MM * 0.925, PathType::MANHATTAN_XY, false), 
       TurnToLocation(Zones::NAT_HIGH_LEFT), 
       CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 100, 0, false), 
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
       DeployMatchloader::getCommand(true), 

       ModifyRobotState::getCommand("is_drive_inverted", false), 
       DrivePath::getCommand({250}, false, false), 
       TurnToLocation(Zones::NAT_ML_LEFT),  
       CloseDistanceBetween(Zones::NAT_ML_LEFT, TILE_SIZE_MM * 0.925, 0, false),  
       DriveForwardForTime::getCommand(0.25, 500, true),
       IntakeCubes::getCommand(1500), 

       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({250}, false, false), 
       TurnToLocation(Zones::NAT_HIGH_LEFT), 
       CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 100, 0, false), 
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
       DeployMatchloader::getCommand(false),  
       
       ModifyRobotState::getCommand("is_drive_inverted", false), 
       DrivePath::getCommand({250}, false, false),
       DriveToLocation(Zones::NAT_PARK, TILE_SIZE_MM * 1.1, PathType::MANHATTAN_XY, false), 
       DriveForwardForTime::getCommand(1, 3000, true), 
       IntakeCubes::getCommand(5000)
   }; 

} 


int main()
{ 

  vexcodeInit();
  
  Drivebase drive = Drivebase(0,0); 

  Intake intake; 
  Indexer indexer;
  Matchloader matchloader;  
  Hooks hooks;     
  Wedge wedge; 
  
  robot.initialize();   

  RobotState::manuallyModifyState("color_sensitive", false); 

  //---------------------------------------------------------------  
  
  driveCommandMatch( 
    closed_side_left(), 
    closed_side_right()
  );
  
  /*
  testAuto( 
    { 
      DrivePath::getCommand({90}, true, false),   
      WaitFor::getCommand(500),
      DrivePath::getCommand({15}, true, false),   
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({30}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({45}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({60}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({75}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500), 
      DrivePath::getCommand({270}, true, false),
      //DrivePath::getCommand({15+180+90}, true, false),   
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({30+180+90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({45+180+90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({60+180+90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({75+180+90}, true, false),  
      WaitFor::getCommand(500),
      DrivePath::getCommand({90}, true, false),
    }
  );  
  */
  


}
