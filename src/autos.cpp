#include "autos.h"

/*
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
 --------------------------------------------------------------------------------------------------------------------------------------------------
*/

AutonOption convertRoutineToOption(Routine routine, int index)
{
  AutonOption option;
  option.description = routine.desc;
  option.name = routine.name;
  option.hasLeftSide = !routine.autos.at(0).empty();
  option.hasRightSide = !routine.autos.at(1).empty();
  option.index = index;
  return option;
}

vector<AutonOption> getOptionVector(vector<Routine> routines)
{
  vector<AutonOption> res;
  for (int index = 0; index < routines.size(); index++)
  {
    res.push_back(convertRoutineToOption(routines.at(index), index));
  }
  return res;
}

vector<Routine> generateRoutinePool(){ 
  vector<Routine> routines; 

  routines.push_back(
      (Routine){
          "7 Block Long Wing",
          "Symmetrical: Score 4 on mid then 3 on high",
          { 
           renegade_left(),
           renegade_right(),
          }});

  
  routines.push_back(
      (Routine){
          "7 Block Split",
          "Symmetrical: Grab near cubes, score on the center goal: (mid : left, low right), then score matchloaded cubes on high",
          {closed_side_left(),
           closed_side_right()}});  


  routines.push_back(
      (Routine){
          "4 Block Control Rush",
          "Symmetrical: Score 4 blocks on the high goal, push to control then camp. [USE STRATEGICALLY]",
          {control_rush_left(),
           control_rush_right(),
          }});  


  routines.push_back(
      (Routine){
          "Getting Carried",
          "Our alliance has a FOR SURE AWP: (left : nothing), (right : forward a smidge)",
          {do_nothing(),
           drive_forward()}}); 

  return routines;
}

//-------------------------------------------------------------------------------------- 


vector<CommandInterface *> closed_side_left() // Needs to be tuned but is mostly good
{
  return {  
      DeployDescore::getCommand(false), 
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2.1125, TILE_SIZE_MM * 1.75}, true}},
          true),  
      DriveForwardForTime::getCommand(0.2, 600, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      //DriveForwardForTime::getCommand(0.2, 200, true), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      TurnToLocation(Zones::NAT_MID), 
      CloseDistanceBetween(Zones::NAT_MID, 50, 0, false),  
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1300), 
     
      ModifyRobotState::getCommand("is_drive_inverted", false),     
      WaitFor::getCommand(30),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM - (ROBOT_LENGTH_MM / 2) + 125, TILE_SIZE_MM * 1}, true}},
          false),  
    
      DrivePath::getCommand({270}, true, false),  
      DriveForwardForTime::getCommand(0.25, 750, true), 
      IntakeCubes::getCommand(500), 

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, -ROBOT_LENGTH_MM*0.55, 0, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1400),  

      DriveForwardForTime::getCommand(-0.25, 600, false),
      DrivePath::getCommand({30}, true, false),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.4225, TILE_SIZE_MM * 2}, true}}, 
          false),   

      DrivePath::getCommand({95}, true, false), 
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.35, 750, false), 
      WaitFor::getCommand(5000)


  };
}

vector<CommandInterface *> closed_side_right() //Must be tuned but is mostly good 
{
  return { 
      DeployDescore::getCommand(false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * (4 - 0.1125), TILE_SIZE_MM * 1.75}, true}},
          true),  
      DriveForwardForTime::getCommand(0.2, 350, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      DriveForwardForTime::getCommand(0.2, 450, true),  
      TurnToLocation(Zones::NAT_LOW),
     
      DeployMatchloader::getCommand(false),
      
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.2, 0, false), 
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500),  
      
      WaitFor::getCommand(20),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.2, TILE_SIZE_MM * 1}, true}},
          false),    
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({270}, true, false),   
      DriveForwardForTime::getCommand(0.2, 825, true),
      IntakeCubes::getCommand(375),
 
      ModifyRobotState::getCommand("is_drive_inverted", true), 

      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, -ROBOT_LENGTH_MM/2, 0, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1350),

      DriveForwardForTime::getCommand(-0.25, 650, false),
      DrivePath::getCommand({30}, true, false),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.46, TILE_SIZE_MM * 2}, true}}, 
          false),   

      DrivePath::getCommand({95}, true, false), 
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.35, 750, false), 
      WaitFor::getCommand(5000)
      
  };
} 

vector<CommandInterface*> switcheroo_right(){ //Needs to be tested [low priority] 
  return {  
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * (4 - 0.1125), TILE_SIZE_MM * 1.75}, true}},
          true),  
      DriveForwardForTime::getCommand(0.2, 350, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      DriveForwardForTime::getCommand(0.2, 200, true),
      DrivePath::getCommand({135}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.2, TILE_SIZE_MM * 1.75}, true}},
          true),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({90, TILE_SIZE_MM * 1}, true, false), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),   
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0, 40),
      WaitFor::getCommand(40), 

      ModifyRobotState::getCommand("is_drive_inverted", false),   
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, ROBOT_LENGTH_MM/2+150, 0, true),
      IntakeCubes::getCommand(650), 
      SlantedAlignWithY::getCommand(TILE_SIZE_MM * 1), 
      TurnToSetpoint::getCommand(TILE_SIZE_MM * 3, TILE_SIZE_MM * 3),    
      DeployMatchloader::getCommand(false),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.4, 0, true), 
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 2000)
      

    };
}

vector<CommandInterface *> switcheroo_left() //Needs to be tested [low priority]
{ 
  return { 
    DeployDescore::getCommand(true),   

    FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.9, TILE_SIZE_MM * 2}, true}},
          true),  
    DeployMatchloader::getCommand(true), 
    IntakeCubes::getCommand(300),   

    DrivePath::getCommand({45}, true, false),
    FollowCirclePath::getCommand(
        {(BiarcEnum){{TILE_SIZE_MM, TILE_SIZE_MM * 1.75}, true}}, 
        true),   
    
    ModifyRobotState::getCommand("is_drive_inverted", true),
    DrivePath::getCommand({90, TILE_SIZE_MM * 0.5}, true, false),  
    ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),
    
    ModifyRobotState::getCommand("is_drive_inverted", false), 
    DriveForwardForTime::getCommand(0.3, 1000, true),  
    IntakeCubes::getCommand(1500),  

    ModifyRobotState::getCommand("is_drive_inverted", true), 
    FlatAlignWithY::getCommand(TILE_SIZE_MM), 
    TurnToLocation(Zones::NAT_MID), 

    CloseDistanceBetween(Zones::NAT_MID, 225, 0, false), 
    ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000), 
    
    SlantedAlignWithX::getCommand(TILE_SIZE_MM * 1.3), 
    DrivePath::getCommand({90}, true, false), 
    DeployDescore::getCommand(false), 
    DriveForwardForTime::getCommand(0.25, 500, false)
    
  };
}

vector<CommandInterface *> renegade_right() //Must be tuned but is mostly good
{
  return { 
      DeployDescore::getCommand(false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * (4 - 0.1125), TILE_SIZE_MM * 1.75}, true}},
          true),  
      DriveForwardForTime::getCommand(0.2, 350, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      DriveForwardForTime::getCommand(0.2, 200, true), 

      DrivePath::getCommand({135}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.23, TILE_SIZE_MM * 1.15}, true}},
          false), 
          
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.2, 850, true),
      IntakeCubes::getCommand(350), 

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({TILE_SIZE_MM * 1.45}, false, false),   
      //DriveForwardForTime::getCommand(0.2, 850, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),  

      DriveForwardForTime::getCommand(-0.25, 675, false),
      DrivePath::getCommand({30}, true, false),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.48, TILE_SIZE_MM * 2}, true}}, 
          false),  
      DrivePath::getCommand({93}, true, false), 
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.15, 1000, false), 
      WaitFor::getCommand(5000)
};
}

vector<CommandInterface *> renegade_left() // Must be tuned but is mostly good
{
  return { 
      DeployDescore::getCommand(false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2.1125, TILE_SIZE_MM * 1.75}, true}},
          true),   
      DriveForwardForTime::getCommand(0.2, 350, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      DriveForwardForTime::getCommand(0.2, 200, true), 

      DrivePath::getCommand({42.5}, true, true),  

      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 0.91),

      DrivePath::getCommand({270}, true, false), 
      DriveForwardForTime::getCommand(0.2, 850, true),
      IntakeCubes::getCommand(1000),  
      
      WaitFor::getCommand(100),
      ModifyRobotState::getCommand("is_drive_inverted", true), 

      DrivePath::getCommand({TILE_SIZE_MM * 1.45}, false, false), 
      //DriveForwardForTime::getCommand(0.4, 500, false), 
    
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1650),   

      DriveForwardForTime::getCommand(-0.25, 650, false),
      DrivePath::getCommand({30}, true, false),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.425, TILE_SIZE_MM * 2}, true}}, 
          false),  
      DrivePath::getCommand({95}, true, false), 
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.35, 750, false), 
      WaitFor::getCommand(5000)
      
    };
}

vector<CommandInterface *> control_rush_left() //Its tuned :)
{
  return { 
    
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2.1125, TILE_SIZE_MM * 1.75}, true}},
          true),   
      DriveForwardForTime::getCommand(0.2, 350, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      DriveForwardForTime::getCommand(0.2, 200, true), 
      
      DrivePath::getCommand({45}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM - (ROBOT_WIDTH_MM/2) + 130, TILE_SIZE_MM * 1.75 + ROBOT_WIDTH_MM/2}, true}}, 
          false),    

      ModifyRobotState::getCommand("is_drive_inverted", true),  
      DrivePath::getCommand({90, TILE_SIZE_MM}, true, false), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250),  

      DriveForwardForTime::getCommand(-0.25, 500, false),
      DrivePath::getCommand({30}, true, false),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.46, TILE_SIZE_MM * 2}, true}}, 
          false),  
      DrivePath::getCommand({93}, true, false),
      DriveForwardForTime::getCommand(0.35, 750, false), 
      WaitFor::getCommand(5000) 
      
     }; 

}

vector<CommandInterface *> control_rush_right() //Its pretty much tuned
{
  return {  
      DeployDescore::getCommand(false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * (4 - 0.1125), TILE_SIZE_MM * 1.75}, true}},
          true),  
      DriveForwardForTime::getCommand(0.2, 350, true), 
      DeployMatchloader::getCommand(true),   
      IntakeCubes::getCommand(250),
      DriveForwardForTime::getCommand(0.2, 200, true),

      DrivePath::getCommand({135}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.25, TILE_SIZE_MM * 1.75}, true}},
          true), 
      ModifyRobotState::getCommand("is_drive_inverted", true),
      DrivePath::getCommand({90, TILE_SIZE_MM * 0.6}, true, false), 
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),  
      
      DriveForwardForTime::getCommand(-0.25, 650, false),
      DrivePath::getCommand({30}, true, false),  
      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5.46, TILE_SIZE_MM * 2}, true}}, 
          false),  
      DrivePath::getCommand({95}, true, false), 
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.35, 750, false)
     
    };
}

vector<CommandInterface *> center_cleanup_left()
{
  return {
      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true), 
      ModifyRobotState::getCommand("is_drive_inverted", true),
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000), 
      
      ModifyRobotState::getCommand("is_drive_inverted", false),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9, 315}, false, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4, TILE_SIZE_MM * 2}, true}},
          false), 

      DriveForwardForTime::getCommand(0.1, 350, true), 
      IntakeCubes::getCommand(300),  

      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1000)

  };
}

vector<CommandInterface *> center_cleanup_right()
{
  return {
      DriveToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.9, PathType::EUCLIDEAN, true),
      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1000), 
      DrivePath::getCommand({-TILE_SIZE_MM * 0.9}, false, false),
      DrivePath::getCommand({225}, true, false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2, TILE_SIZE_MM * 2}, true}},
          false), 

      DriveForwardForTime::getCommand(0.1, 350, true), 
      IntakeCubes::getCommand(300),  

      ModifyRobotState::getCommand("is_drive_inverted", true), 
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.9}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000)
  };
}

vector<CommandInterface *> auto_skills_f__k_it_we_ball()
{ // 119 pts
  return {

      //-------------------------------------------------------------------------------
      // Mid goal control fill
      // PRETTY PLEASE WITH CHERRIES ON TOP WORK ON FIRST TRY
      
      //Has cube at the start 

      ParkClear::getCommand(
          { 
           (PCPhase){0.25, 500, true},
           (PCPhase){-0.25, 500, false},
           (PCPhase){0.05, 250, false} 
          },
          true),

      ModifyRobotState::getCommand("is_drive_inverted", true),
  
      DriveForwardForTime::getCommand(0.1, 250, false),
      DriveToLocation(Zones::NAT_MID, TILE_SIZE_MM, PathType::EUCLIDEAN, false),
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DrivePath::getCommand({TILE_SIZE_MM * 0.6}, false, false),
     
      //-------------------------------------------------------------------------------

      DriveToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM * 1.5, PathType::EUCLIDEAN, false),
      DrivePath::getCommand({270}, true, false),
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.125, 500, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4.5, TILE_SIZE_MM * 2}, true}},
          false),

      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5, TILE_SIZE_MM * 4.75}, true}},
          false),

      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4, TILE_SIZE_MM * 5.75}, true}},
          false), 
      
      DeployMatchloader::getCommand(false),
    
      //------------------------------------------------------------------
      // Low Goal control fill
      // IM LITERALLY ON MY KNEES BEGGING THAT THIS WORKS
   
      ParkClear::getCommand(
          {(PCPhase){0.25, 750, true},
           (PCPhase){-0.05, 250, false}},
          false),
    
      DriveForwardForTime::getCommand(0.1, 500, false),
      DriveToLocation(Zones::FAR_LOW, TILE_SIZE_MM * 2, PathType::EUCLIDEAN, false),
      TurnToLocation(Zones::FAR_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.75}, false, true),
      IntakeCubes::getCommand(100),
      DrivePath::getCommand({TILE_SIZE_MM}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1000),

      //-------------------------------------------------------------------
      DeployMatchloader::getCommand(true), 
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 5}, true}},
          false),
    
      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.5, TILE_SIZE_MM * 4}, true}},
          false), 

      FlatAlignWithY::getCommand(TILE_SIZE_MM * 2),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1.5}, true}},
          false),
      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0, 100),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2, TILE_SIZE_MM * 0.75}, true}},
          false), 
      DeployMatchloader::getCommand(false), 
      DriveForwardForTime::getCommand(0.2, 1000, true) 
    };
}

vector<CommandInterface *> auto_skills_safety_net()
{
  return {
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),
      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.1, 500, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 4.5, TILE_SIZE_MM * 2}, true}},
          false),

      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 5, TILE_SIZE_MM * 4.75}, true}},
          false),

      DrivePath::getCommand({270}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1200),

      DriveForwardForTime::getCommand(-0.1, 300, false),
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 1),
      DrivePath::getCommand({90}, true, false),

      DriveForwardForTime::getCommand(0.25, 1000, true),
      IntakeCubes::getCommand(1000),
      
      ModifyRobotState::getCommand("is_drive_inverted", true), 
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1.5, TILE_SIZE_MM * 4}, true}},
          false),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 2),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 1, TILE_SIZE_MM * 1.5}, true}},
          false),
      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.125, 500, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      IntakeCubes::getCommand(1000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 1000, false),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0, 100),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      FollowCirclePath::getCommand(
          {(BiarcEnum){{TILE_SIZE_MM * 2, TILE_SIZE_MM * 0.75}, true}},
          false),
      DriveForwardForTime::getCommand(0.2, 1000, true)
  };
}

vector<CommandInterface *> auto_skills()
{
  return auto_skills_f__k_it_we_ball();
}

vector<CommandInterface *> empty()
{
  return {};
}

vector<CommandInterface *> do_nothing()
{
  return {
      IntakeCubes::getCommand(500)};
}

vector<CommandInterface *> drive_forward()
{
  return {
      DriveForwardForTime::getCommand(0.2, 500, false)};
}
