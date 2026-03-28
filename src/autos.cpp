#include "autos.h"

/*
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
 --------------------------------------------------------------------------------------------------------------------------------------------------
*/

vector<CommandInterface *> closed_side_left()
{
  return {
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.925), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.925, ROBOT_LENGTH_MM / 2 + 50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),

      ModifyRobotState::getCommand("is_drive_inverted", true),

      // TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.3),

      DrivePath::getCommand({50, TILE_SIZE_MM * 0.95}, true, true),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      DriveForwardForTime::getCommand(0.2, 300, false),
      TurnToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM / 2 + 250),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + 80),
      DrivePath::getCommand({270}, true, false),

      DriveForwardForTime::getCommand(0.25, 600, true),
      IntakeCubes::getCommand(500),
      ModifyRobotState::getCommand("is_drive_inverted", true),

      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, -125, true),

      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250),
      // DisengageHighGoal::getCommand(0.2, 750),

  };
}

vector<CommandInterface *> closed_side_right()
{
  return {
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM / 2 + 25, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM / 2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),

      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 2000),
      DeployMatchloader::getCommand(true),

      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5),

      TurnToLocation(Zones::NAT_ML_RIGHT),
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, ROBOT_LENGTH_MM / 2 + 250, 0, true),
      DriveForwardForTime::getCommand(0.2, 400, true),
      IntakeCubes::getCommand(625),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      TurnToLocation(Zones::NAT_HIGH_RIGHT),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, ROBOT_LENGTH_MM / 2, 0, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 750),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1250),
      // DisengageHighGoal::getCommand(0.2, 500),

      DrivePath::getCommand({30}, true, false),
      DeployDescore::getCommand(true),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM / 2) + 100)),
      DrivePath::getCommand({90}, true, false),
      DriveForwardForTime::getCommand(0.3, 750, false),
      WaitFor::getCommand(5000)};
}

vector<CommandInterface *> switcheroo_left()
{
  return {
      DeployMatchloader::getCommand(true),
      DriveForwardForTime::getCommand(0.2, 300, false),

      TurnToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + 30),

      DrivePath::getCommand({270}, true, false),
      WaitFor::getCommand(300),
      DriveForwardForTime::getCommand(0.3, 750, true),
      IntakeCubes::getCommand(500),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, -75, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.35, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1000),
      DeployMatchloader::getCommand(false),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      WaitFor::getCommand(50),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1),

      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 1.3),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 1.3, ROBOT_LENGTH_MM / 2 + 25, false),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(600),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 1.175}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 1500)

  };
}

vector<CommandInterface *> renegade_right()
{
  return {
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.885),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.885, ROBOT_LENGTH_MM / 2 + 50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      TurnToLocation(Zones::NAT_ML_RIGHT, TILE_SIZE_MM * 1.125),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + 35),
      DrivePath::getCommand({270}, true, false),

      DriveForwardForTime::getCommand(0.25, 800, true),
      IntakeCubes::getCommand(500),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, -75, 0, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.3, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),
      // DisengageHighGoal::getCommand(0.225, 700),

      DrivePath::getCommand({30}, true, false),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM / 2) + 120)),
      DrivePath::getCommand({90}, true, false),
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 825, false),
      WaitFor::getCommand(5000)

  };
}

vector<CommandInterface *> renegade_left()
{
  return {
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.925), // Get cubes near low
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.925, ROBOT_LENGTH_MM / 2 + 50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      TurnToLocation(Zones::NAT_ML_LEFT, TILE_SIZE_MM),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + 80),
      DrivePath::getCommand({270}, true, false),

      DriveForwardForTime::getCommand(0.25, 800, true),
      IntakeCubes::getCommand(500),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, -75, 0, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.3, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),
      // DisengageHighGoal::getCommand(0.225, 700),

      DrivePath::getCommand({30}, true, false),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + ((ROBOT_WIDTH_MM / 2) + 100)),
      DrivePath::getCommand({90}, true, false),
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 825, false),
      WaitFor::getCommand(5000)

  };
}

vector<CommandInterface *> control_rush_left()
{
  return {
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.925),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.925, ROBOT_LENGTH_MM / 2 + 50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),

      DeployMatchloader::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.4),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM),
      TurnToLocation(Zones::NAT_HIGH_LEFT),

      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 0, -75, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.2, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),
      // DisengageHighGoal::getCommand(0.225, 700),

      DrivePath::getCommand({30}, true, false),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM + ((ROBOT_WIDTH_MM / 2) + 100)),
      DrivePath::getCommand({90}, true, false),
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 850, false),
      WaitFor::getCommand(5000)

  };
}

vector<CommandInterface *> control_rush_right()
{
  return {
      DeployDescore::getCommand(false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.875),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.875, ROBOT_LENGTH_MM / 2 + 50, true),
      DriveForwardForTime::getCommand(0.125, 1000, true),
      DeployMatchloader::getCommand(true),
      IntakeCubes::getCommand(400),

      DeployMatchloader::getCommand(false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.125),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5),
      TurnToLocation(Zones::NAT_HIGH_RIGHT),

      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, 0, -75, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.2, 100),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 1500),
      // DisengageHighGoal::getCommand(0.225, 700),

      DrivePath::getCommand({30}, true, false),
      SlantedAlignWithX::getCommand(TILE_SIZE_MM * 5 + ((ROBOT_WIDTH_MM / 2) + 100)),
      DrivePath::getCommand({90}, true, false),
      DeployDescore::getCommand(true),
      DriveForwardForTime::getCommand(0.4, 825, false),
      WaitFor::getCommand(5000)};
}

vector<CommandInterface *> center_cleanup_left()
{
  return {
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM / 2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM / 2}, false, true),
      DeployMatchloader::getCommand(false),
      IntakeCubes::getCommand(400),
      ModifyRobotState::getCommand("is_drive_inverted", true),

      TurnToLocation(Zones::NAT_MID),
      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000),
      ModifyRobotState::getCommand("is_drive_inverted", false),

      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false),
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM, ROBOT_LENGTH_MM / 2, false),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM / 2 + 50}, false, true),
      IntakeCubes::getCommand(400),
      DeployMatchloader::getCommand(false),

      TurnToLocation(Zones::NAT_LOW),
      CloseDistanceBetween(Zones::NAT_LOW, ROBOT_LENGTH_MM / 2, 0, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500),
  };
}

vector<CommandInterface *> center_cleanup_right()
{
  return {
      TurnToLocation(Zones::NAT_LOW, TILE_SIZE_MM * 0.95),
      CloseDistanceBetween(Zones::NAT_LOW, TILE_SIZE_MM * 0.95, ROBOT_LENGTH_MM / 2, true),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM / 2}, false, true),
      IntakeCubes::getCommand(400),
      DeployMatchloader::getCommand(false),

      TurnToLocation(Zones::NAT_LOW),
      DrivePath::getCommand({TILE_SIZE_MM * 0.8}, false, false),
      ScoreOnGoal::getCommand(Goal_Pos::LOW_GOAL, 1500),

      DrivePath::getCommand({-TILE_SIZE_MM * 0.8}, false, false),
      TurnToLocation(Zones::NAT_MID, TILE_SIZE_MM),
      CloseDistanceBetween(Zones::NAT_MID, TILE_SIZE_MM, ROBOT_LENGTH_MM / 2, false),
      DeployMatchloader::getCommand(true),
      DrivePath::getCommand({ROBOT_LENGTH_MM / 2 + 50}, false, true),
      IntakeCubes::getCommand(400),
      DeployMatchloader::getCommand(false),
      ModifyRobotState::getCommand("is_drive_inverted", true),

      TurnToLocation(Zones::NAT_MID),
      CloseDistanceBetween(Zones::NAT_MID, ROBOT_LENGTH_MM / 2, 0, false),
      ScoreOnGoal::getCommand(Goal_Pos::MID_GOAL, 2000)
  };
}

vector<CommandInterface *> auto_skills()
{
  return {

      DeployDescore::getCommand(false),
      DriveForwardForTime::getCommand(0.2, 450, false),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      DriveToLocation(Zones::NAT_HIGH_RIGHT, TILE_SIZE_MM * 0.5, PathType::EUCLIDEAN, false),
      TurnToLocation(Zones::NAT_HIGH_RIGHT),
      CloseDistanceBetween(Zones::NAT_HIGH_RIGHT, -12.5, 0, true),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_RIGHT, 0.45, 400),

      DeployMatchloader::getCommand(true),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 500),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_ML_RIGHT, 300, 0, false),
      DriveForwardForTime::getCommand(0.25, 250, true),
      IntakeCubes::getCommand(2000),

      //////////////////////////////////////////

      ModifyRobotState::getCommand("is_drive_inverted", true),
      DriveForwardForTime::getCommand(0.25, 500, true),

      DriveToSetpoint::getCommand(TILE_SIZE_MM * 4.25, TILE_SIZE_MM * 1.6, -1, PathType::EUCLIDEAN, true),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 4.55),
      FlatAlignWithX::getCommand(TILE_SIZE_MM * 5),

      // DrivePath::getCommand({270}, true, false),
      TurnToLocation(Zones::FAR_HIGH_RIGHT),
      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.45, 1550),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),
      // DisengageHighGoal::getCommand(0.1, 250),
      DeployMatchloader::getCommand(true),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::FAR_ML_RIGHT, 300, 0, false),
      DriveForwardForTime::getCommand(0.25, 250, true),

      IntakeCubes::getCommand(2000),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::FAR_HIGH_RIGHT, ROBOT_LENGTH_MM / 2, 0, true),
      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_RIGHT, 0.45, 800),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),

      // DisengageHighGoal::getCommand(0.25, 750),

      ModifyRobotState::getCommand("is_drive_inverted", true),

      DrivePath::getCommand({180, TILE_SIZE_MM * 4 + 15}, true, false),
      TurnToLocation(Zones::FAR_HIGH_LEFT),
      CloseDistanceBetween(Zones::FAR_HIGH_LEFT, ROBOT_LENGTH_MM / 2, 0, false),

      Calibrate::getCommand(Alignment_Structure::FOREIGN_HIGH_LEFT, 0.45, 950),
      WaitFor::getCommand(50),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::FAR_ML_LEFT, 300, 0, false),
      DriveForwardForTime::getCommand(0.25, 250, true),
      IntakeCubes::getCommand(2250),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),

      DriveForwardForTime::getCommand(0.3, 500, false),
      DriveToSetpoint::getCommand(TILE_SIZE_MM * 1.75, TILE_SIZE_MM * 4, 270, PathType::EUCLIDEAN, true),
      FlatAlignWithY::getCommand(TILE_SIZE_MM * 1.5),
      FlatAlignWithX::getCommand(TILE_SIZE_MM),

      TurnToLocation(Zones::NAT_HIGH_LEFT),

      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.4, 1300),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2000),
      // DisengageHighGoal::getCommand(0.2, 350),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      WaitFor::getCommand(50),
      CloseDistanceBetween(Zones::NAT_ML_LEFT, 300, 0, false),
      DriveForwardForTime::getCommand(0.25, 250, true),
      IntakeCubes::getCommand(2200),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      // TurnToLocation(Zones::NAT_HIGH_LEFT),
      CloseDistanceBetween(Zones::NAT_HIGH_LEFT, 150, 0, true),

      ModifyRobotState::getCommand("is_drive_inverted", true),
      WaitFor::getCommand(50),
      Calibrate::getCommand(Alignment_Structure::NEARBY_HIGH_LEFT, 0.45, 500),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 2250),

      DeployMatchloader::getCommand(false),
      // DisengageHighGoal::getCommand(0.1, 250),

      ModifyRobotState::getCommand("is_drive_inverted", false),
      WaitFor::getCommand(50),
      SlantedAlignWithY::getCommand(TILE_SIZE_MM),
      TurnToSetpoint::getCommand(TILE_SIZE_MM * 2.65, TILE_SIZE_MM * 0.1),
      DrivePath::getCommand({TILE_SIZE_MM * 2.35}, false, true),
      ScoreOnGoal::getCommand(Goal_Pos::HIGH_GOAL, 750)
  };
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
