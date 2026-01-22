#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "subsystems/hooks.h" 
#include "commands.h"
#include "gui/autonInit.h"

using namespace vex;

competition Competition;
Robot robot; 


void runTelemetry()
{
  robot.runTelemetryThread(); 
} 

void awaitStartingSignal(){ 
  LoadingScreen(); 
  Sprite::frameLoop();
}

int scheduleCallbacks(){   
   
  //Add a pause where you press a button to start    
  awaitStartingSignal();  

  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });   

  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  return 0;
}

void configurateAutonomous(vector<CommandInterface*> leftSide, vector<CommandInterface*> rightSide){ 
  ColorPicker colorChooser = ColorPicker(220-125,200-50);  
  SidePicker sidePicker = SidePicker(360-125,200-50);  
  ExitBlock(160, 65);
  Sprite::frameLoop();   
  
  if (sidePicker.getIsLeft()){ 
    robot.setAutonomousCommand(leftSide);  
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
  } else { 
    robot.setAutonomousCommand(rightSide);    
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
  }    

  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}  

void testDrive()
{   
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
} 

void testAuto(vector<CommandInterface*> auton){  
  robot.setAutonomousCommand(auton); 
  thread telemThread = thread(runTelemetry);    
  robot.autonControl();                      
  robot.driverControl();
} 

void startCommandSkillsMatch(vector<CommandInterface*> commandGroup){  
  robot.setAutonomousCommand(commandGroup);  
  thread callBackTrigger = thread(scheduleCallbacks);  
  robot.runTelemetryThread();
}

void startCommandCompetitiveMatch(vector<CommandInterface *> leftCommandGroup, vector<CommandInterface *> rightCommandGroup)
{
  configurateAutonomous(leftCommandGroup, rightCommandGroup);
  thread callBackTrigger = thread(scheduleCallbacks); 
  robot.runTelemetryThread(); 
}

void driveCommandMatch(vector<CommandInterface *> leftCommandGroup, vector<CommandInterface*> rightCommandGroup)
{  
  configurateAutonomous(leftCommandGroup, rightCommandGroup);    
  thread telemThread = thread(runTelemetry);   
  awaitStartingSignal();
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  robot.autonControl();                      
  robot.driverControl();                     
}



/*------------------------------------------------------------------------------------------------------------------------------------------------- 
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|  

 --------------------------------------------------------------------------------------------------------------------------------------------------
*/

vector<CommandInterface*> closed_side_left(){ 
   return { 
       TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.875), 
       CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.875, 250, true),   
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
       TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 1.25),  //Get cubes near low
       CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 1.25, 500, true),
       DeployMatchloader::getCommand(true), 
       DriveForwardForTime::getCommand(0.4, 350, true),  
       IntakeCubes::getCommand(500),       
       
       
       ModifyRobotState::getCommand("is_drive_inverted", true),
       DriveToLocation(Zones::NAT_HIGH_RIGHT, (ROBOT_LENGTH_MM * 2), PathType::MANHATTAN_YX, false),  //Score low cubes on 4-high
       TurnToLocation(Zones::NAT_HIGH_RIGHT),    
       CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, 100, 0, false),   
       Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, -0.2, 300),
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),   

      
       ModifyRobotState::getCommand("is_drive_inverted", false),  
       DriveToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM * 0.4, PathType::EUCLIDEAN, false), // Matchload cubes from 4
       TurnToLocation(Zones::NAT_ML_RIGHT), 
       DriveForwardForTime::getCommand(0.3, 50, true),
       IntakeCubes::getCommand(1500),     
      
       
       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({200}, false, false),  
       DeployMatchloader::getCommand(false),       
       DriveToSetpoint::getCommand((TILE_SIZE_MM * 4.25), (TILE_SIZE_MM * 4) + (ROBOT_LENGTH_MM), 0, PathType::MANHATTAN_XY, false),   
       DriveToLocation(Zones::FAR_HIGH_RIGHT, ROBOT_LENGTH_MM * 2 / 4, PathType::MANHATTAN_XY, false), 
       TurnToLocation(Zones::FAR_HIGH_RIGHT),   
       CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, 100, 0, false), 
       Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, -0.25, 1500),
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), //Go to the other side (1) and score 
       DeployMatchloader::getCommand(true), 


       ModifyRobotState::getCommand("is_drive_inverted", false),  
       DrivePath::getCommand({200}, false, false), 
       CloseDistanceBetween(Zones::FAR_ML_RIGHT, TILE_SIZE_MM * 0.4, 0, false),  //Matchload from other side (1)  
       TurnToLocation(Zones::FAR_ML_RIGHT),
       DriveForwardForTime::getCommand(0.25, 100, true),
       IntakeCubes::getCommand(1500), 
       
       
       ModifyRobotState::getCommand("is_drive_inverted", true),  
       DrivePath::getCommand({250}, false, false), 
       TurnToLocation(Zones::FAR_HIGH_RIGHT), 
       CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, 100, 0, false), // Score on high goal side (1) 
       Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, -0.25, 1000), 
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
       
       ModifyRobotState::getCommand("is_drive_inverted", false), 
       DrivePath::getCommand({250}, false, false),  
       DriveToLocation(Zones::FAR_ML_LEFT, TILE_SIZE_MM * 1, PathType::MANHATTAN_YX, false),  
       TurnToLocation(Zones::FAR_ML_LEFT),   
       DriveForwardForTime::getCommand(0.25, 750, true),
       IntakeCubes::getCommand(1500),  
       
       
       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({250}, false, false), 
       DeployMatchloader::getCommand(false),   
       DriveToSetpoint::getCommand(TILE_SIZE_MM + (ROBOT_WIDTH_MM * 1.1), TILE_SIZE_MM + (ROBOT_LENGTH_MM/2), -1, PathType::MANHATTAN_XY, false), 
       DriveToLocation(Zones::NAT_HIGH_LEFT, ROBOT_LENGTH_MM * 3 / 4, PathType::MANHATTAN_XY, false), 
       TurnToLocation(Zones::NAT_HIGH_LEFT), 
       CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 100, 0, false),  
       Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, -0.25, 1000), 
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
       DeployMatchloader::getCommand(true),   

       
       ModifyRobotState::getCommand("is_drive_inverted", false), 
       DrivePath::getCommand({250}, false, false), 
       TurnToLocation(Zones::NAT_ML_LEFT),  
       CloseDistanceBetween(Zones::NAT_ML_LEFT, TILE_SIZE_MM * 0.925, 0, false),  
       DriveForwardForTime::getCommand(0.25, 750, true),
       IntakeCubes::getCommand(1500),  

       ModifyRobotState::getCommand("is_drive_inverted", true), 
       DrivePath::getCommand({250}, false, false), 
       TurnToLocation(Zones::NAT_HIGH_LEFT), 
       CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 100, 0, false),  
       Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, -0.25, 1000),
       ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500), 
       DeployMatchloader::getCommand(false),   
       
       ModifyRobotState::getCommand("is_drive_inverted", false), 
       DrivePath::getCommand({250}, false, false),
       DriveToLocation(Zones::NAT_PARK, TILE_SIZE_MM * 1, PathType::MANHATTAN_XY, false),  
       TurnToLocation(Zones::NAT_PARK), 
       DriveForwardForTime::getCommand(1, 3000, true), 
       IntakeCubes::getCommand(2000)  
       
   }; 

} 

//------------------------------------------------------------------------------------------------------------------------------------------------- 

int main()
{ 

  vexcodeInit();
  
  //--------------------DONT MODIFY-----------------
  Drivebase drive = Drivebase((TILE_SIZE_MM * 3 + 200),  425 / TILE_SIZE_MM);  
  Intake intake; 
  Indexer indexer;
  Matchloader matchloader;  
  Hooks hooks;     
  robot.initialize();   
  RobotState::manuallyModifyState("color_sensitive", false); 

  //-------------------PROTOCOL-------------------  
  
  testAuto( 
    auto_skills()
  ); 
  
  
  
  


}
