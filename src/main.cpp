#include "vex.h"
#include "architecture/robot.h"
#include <iostream>

#include "autos.h"
#include "gui/autonInit.h"

using namespace vex;

competition Competition;
Robot robot;

typedef struct
{
  string name;
  string desc;
  array<vector<CommandInterface *>, 2> autos;
} Routine;

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

void runTelemetry()
{
  robot.runTelemetryThread();
}

void awaitStartingSignal()
{
  LoadingScreen();
  Sprite::frameLoop();
}

int scheduleCallbacks()
{

  // Add a pause where you press a button to start
  // awaitStartingSignal();

  Competition.autonomous([]()
                         { robot.autonControl(); });
  Competition.drivercontrol([]()
                            { robot.driverControl(); });

  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  return 0;
}

void testDrive()
{
  thread telemetryThread = thread(runTelemetry);
  robot.driverControl();
}

void testAuto(vector<CommandInterface *> auton, bool startingLeft)
{
  if (startingLeft)
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
  }
  else
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
  }
  robot.setAutonomousCommand(auton);
  thread telemThread = thread(runTelemetry);
  robot.autonControl();
  robot.driverControl();
}

void configurateAutonomous(vector<Routine> routines)
{
  ColorPicker colorChooser = ColorPicker(135 + 115, 200);
  SidePicker sidePicker = SidePicker(235 + 115, 200);
  RoutineCatalog catalog = RoutineCatalog(10, 10, getOptionVector(routines));
  AutonDisplay(250, 80, &catalog, &sidePicker);
  ExitBlock(250, 10);
  Sprite::frameLoop();

  int sideIndex;
  int autonIndex = catalog.getCurrentAuto().index;

  if (sidePicker.getIsLeft())
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
    sideIndex = 0;
  }
  else
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
    sideIndex = 1;
  }

  robot.setAutonomousCommand(routines.at(autonIndex).autos.at(sideIndex));
  RobotState::manuallyModifyState("is_team_color_blue", colorChooser.getIsBlue());
}

void startCommandSkillsMatch(vector<CommandInterface *> commandGroup, bool startingLeft)
{
  RobotState::manuallyModifyState("in_skills", true);
  if (startingLeft)
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 2 + 430 - ROBOT_WIDTH_MM), 425);
  }
  else
  {
    Drivebase::globalRef->setStartingPos((TILE_SIZE_MM * 3 + 200), 425);
  }
  robot.setAutonomousCommand(commandGroup);
  thread callBackTrigger = thread(scheduleCallbacks);
  robot.runTelemetryThread();
}

void startCommandCompetitiveMatch(vector<Routine> routines)
{
  configurateAutonomous(routines);
  thread callBackTrigger = thread(scheduleCallbacks);
  robot.runTelemetryThread();
}

void driveCommandMatch(vector<Routine> routines)
{
  configurateAutonomous(routines);
  thread telemThread = thread(runTelemetry);
  awaitStartingSignal();
  drawLogo(RobotState::getStateOf("is_team_color_blue"));
  robot.autonControl();
  robot.driverControl();
}

//------------------------------>-------------------------------------------------------------------------------------------------------------------

int main()
{

  vexcodeInit();

  //--------------------DONT MODIFY (MOSTLY)-----------------

  Drivebase drive = Drivebase(0,0);

  Intake intake;
  Matchloader matchloader;
  Hooks hooks;

  robot.initialize();

  RobotState::manuallyModifyState("color_sensitive", false);     // <- We don't have color-sort currently
  RobotState::manuallyModifyState("is_counterclockwise", false); // Adjust to match inertial sensor orientation

  //-------------------ROUTINE CREATION-------------------

  vector<Routine> routines;

  routines.push_back(
      (Routine){
          "Default [WHEN IN DOUBT]",
          "If on the left side: >>[4 blocks in mid - matchload 3 cubes - 3 blocks in long]. If right: [matchload 3 cubes - 7 blocks in long]",
          {closed_side_left(),
           renegade_right()}});

  routines.push_back(
      (Routine){
          "7 Block High",
          "Symmetrical: Gets middle cubes and matchloaded cubes and scores on the high goal",
          {
              renegade_left(),
              renegade_right(),
          }});

  routines.push_back(
      (Routine){
          "2/3 AWP",
          "Symmetrical: Grab near cubes, score on the center goal: (mid : left, low right), then score matchloaded cubes on high",
          {closed_side_left(),
           closed_side_right()}});

  routines.push_back(
      (Routine){
          "Control Rush",
          "Symmetrical: Score 3 blocks on the high goal, push to control then camp. [USE STRATEGICALLY]",
          {
              control_rush_left(),
              control_rush_right(),
          }});

  routines.push_back(
      (Routine){
          "Switcheroo",
          "Left ONLY: Switch the order of scoring for 7 block auto [high goal -> mid goal]",
          {switcheroo_left(),
           empty()}});

  routines.push_back(
      (Routine){
          "Center Goal Clean",
          "Symmetrical: Score a combined 7 blocks on center goals - order depends on side [left mid, right low]",
          {center_cleanup_left(),
           center_cleanup_right()}});

  routines.push_back(
      (Routine){
          "Getting Carried",
          "Our alliance has a FOR SURE AWP: (left : nothing), (right : forward a smidge)",
          {do_nothing(),
           drive_forward()}});

  //-----------------------PROTOCOL------------------------

  // startCommandCompetitiveMatch(routines);  //Uncomment when loading up for a comp
  // startCommandSkillsMatch(auto_skills(), false);  //Uncomment when loading up for skills
  //testDrive(); // Uncomment when getting driver practice

 //startCommandCompetitiveMatch(routines);  //Uncomment when loading up for a comp   
 //startCommandSkillsMatch(auto_skills(), false);  //Uncomment when loading up for skills 
 testDrive();//Uncomment when getting driver practice 

 /*
 testAuto( 
   renegade_right(),
   false
 );    
 */
 
 
 
 
 
 
    
 


}
