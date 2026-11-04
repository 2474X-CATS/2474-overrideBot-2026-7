#include "autos.h"


/* 
-------------------------------------------------------------------------------------------------------------------------------------------------
     _              _______   _____    _____
    / \    |     |     |     |     |   |
   /___\   |     |     |     |     |   -----
  /     \  |_____|     |     |_____|   ____|
----------------------------------------------------------------------------------------------------------------------------------------------------
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
  /* 
  Add routines here
  */ 
  Routine toggPin;
  toggPin.name = "4 Pin"; 
  toggPin.desc = "2 Yellow on Alliance goal (alliance line) left > right";
  toggPin.autos = {
    two_pin_alliance_right(),
    two_pin_alliance_left()
  };
  
  Routine togg;
  togg.name = "Toggle + Preload";
  togg.desc = "Basic and reliable";
  togg.autos = { 
    toggle_right(), 
    toggle_left()
  };  


  routines.push_back(toggPin); 
  routines.push_back(togg);
  return routines;
}

//-------------------------------------------------------------------------------------- 

vector<CommandInterface*> two_pin_alliance_right(){ 
  return RoutineWrap({  
            WaitForToggle(),
            //ResetPose::getCommand({ResetFace::LEFT, ResetFace::RIGHT}),
            FrontRunElevatorSetpoint::getCommand(3.5, 25.4),  
            WaitFor::getCommand(350),
            DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.8, TILE_SIZE_MM, false, 0)), 
            Score(),
            DriveForward::getCommand(-300),   
            
            DriveIntoElement(DriveToSetpoint::getCommand(TILE_SIZE_MM * 1.95, TILE_SIZE_MM * 1.89, RouteType::EUCLIDEAN)), 
            
            WaitFor::getCommand(300),
            ParallelCommandGroup::makeGroup({ 
              FrontRunElevatorSetpoint::getCommand(5, 35.4),
              SequentialCommandGroup::makeGroup({ 
                    WaitFor::getCommand(350), 
                    DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.88, TILE_SIZE_MM*0.6, true, 0))
                  })  
             }),
            Score(),  
            
            DriveForward::getCommand(-TILE_SIZE_MM*0.75),
            FaceTarget::getCommand(TILE_SIZE_MM*0.85, TILE_SIZE_MM * 0.5), 
            DriveIntoElement(DriveForward::getCommand(TILE_SIZE_MM * 1.3)),   

            ParallelCommandGroup::makeGroup({ 
              FrontRunElevatorSetpoint::getCommand(7, 35.4),
              SequentialCommandGroup::makeGroup({
                    WaitFor::getCommand(350), 
                    DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.915, TILE_SIZE_MM*0.85, true, 0))
                  })  
            }), 
            Score()   
         });
} 

/*
vector<CommandInterface*> two_pin_alliance_right(){ 
  return RoutineWrap({    
            ResetPose::getCommand({ResetFace::LEFT, ResetFace::RIGHT}),
            WaitForToggle(),
            FrontRunElevatorSetpoint::getCommand(3, 25.4),  
            WaitFor::getCommand(350),
            DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.5, TILE_SIZE_MM, false, 0)), 
            Score(),
            DriveForward::getCommand(-300),     
            DriveIntoElement(DriveToSetpoint::getCommand(TILE_SIZE_MM * 1.95, TILE_SIZE_MM * 1.89, RouteType::EUCLIDEAN)), 
            
            /*
            WaitFor::getCommand(300),
            ParallelCommandGroup::makeGroup({ 
              FrontRunElevatorSetpoint::getCommand(5, 35.4),
              SequentialCommandGroup::makeGroup({ 
                    WaitFor::getCommand(350), 
                    DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.88, TILE_SIZE_MM*0.6, true, 0))
                  })  
             }),
            Score(),  
            
            
            DriveForward::getCommand(-TILE_SIZE_MM*0.75),
            FaceTarget::getCommand(TILE_SIZE_MM*0.875, TILE_SIZE_MM * 0.5), 
            DriveIntoElement(DriveForward::getCommand(TILE_SIZE_MM * 1.25)),  
            
            ParallelCommandGroup::makeGroup({ 
              FrontRunElevatorSetpoint::getCommand(7, 35.4),
              SequentialCommandGroup::makeGroup({
                    WaitFor::getCommand(350), 
                    DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.915, TILE_SIZE_MM*0.85, true, 0))
                  })  
            }), 
            Score()    
            
         });
}  
*/

vector<CommandInterface*> two_pin_alliance_left(){ 
  return RoutineWrap({   
            ResetPose::getCommand({ResetFace::LEFT, ResetFace::RIGHT}),
            WaitForToggle(),
            FrontRunElevatorSetpoint::getCommand(3.5, 25.4),  
            WaitFor::getCommand(300),
            DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * (6 - 2), TILE_SIZE_MM, false, 0)), 
            Score(), 
            DriveForward::getCommand(-300),
            DriveIntoElement(DriveToSetpoint::getCommand(TILE_SIZE_MM * (6 - 2.175), TILE_SIZE_MM * 2, RouteType::EUCLIDEAN)), 
            WaitFor::getCommand(300), 
            FrontRunElevatorSetpoint::getCommand(5, 35.4),
            FaceTarget::getCommand(TILE_SIZE_MM * (6 - 2.25), TILE_SIZE_MM*0.9),
            DriveIntoGoal(DriveForward::getCommand(TILE_SIZE_MM*0.9)), 
            Score(),
            DriveForward::getCommand(-TILE_SIZE_MM*0.75),
            //FaceTarget::getCommand(TILE_SIZE_MM * 4.85, TILE_SIZE_MM * 0.65),  
            DriveIntoElement(DriveToSetpoint::getCommand(TILE_SIZE_MM * 4.95, TILE_SIZE_MM * 0.7, RouteType::EUCLIDEAN)/*DriveForward::getCommand(TILE_SIZE_MM * 1.35)*/),  
            ParallelCommandGroup::makeGroup({ 
              FrontRunElevatorSetpoint::getCommand(8, 35.4),
              SequentialCommandGroup::makeGroup({
                    WaitFor::getCommand(600), 
                    DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * (6 - 2.15), TILE_SIZE_MM*0.815, true, 0))
                  })  
             }),  
            //WaitFor::getCommand(250),
            Score(), 
         });
}  

vector<CommandInterface*> toggle_left(){ 
  return {
    RoutineWrap( 
       { 
        WaitForToggle(), 
        //DriveForward::getCommand(250), 
        FrontRunElevatorSetpoint::getCommand(2.5, 25.4),  
        WaitFor::getCommand(300),
        DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * (6 - 2/*1.75*/), TILE_SIZE_MM, false, 0)), 
        Score()
       }
    )
  };
}   

vector<CommandInterface*> toggle_right(){ 
  return {
    RoutineWrap( 
       { 
        WaitForToggle(), 
        //DriveForward::getCommand(250), 
        FrontRunElevatorSetpoint::getCommand(3, 25.4),  
        WaitFor::getCommand(350),
        DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * 1.8, TILE_SIZE_MM, false, 0)), 
        Score(),
        DriveForward::getCommand(-300),
       }
    )
  };
}  

vector<CommandInterface*> auto_skills(){ 
  return RoutineWrap({   
            ResetPose::getCommand({ResetFace::LEFT, ResetFace::RIGHT}),
            WaitForToggle(),
            FrontRunElevatorSetpoint::getCommand(2, 25.4),  
            WaitFor::getCommand(300),
            DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * (6 - 2), TILE_SIZE_MM, false, 0)), 
            Score(), //=====================

            DriveForward::getCommand(-300),
            DriveIntoElement(DriveToSetpoint::getCommand(TILE_SIZE_MM * (6 - 2.25), TILE_SIZE_MM * 2, RouteType::EUCLIDEAN)), 
            WaitFor::getCommand(300), 
            FrontRunElevatorSetpoint::getCommand(5, 35.4),
            FaceTarget::getCommand(TILE_SIZE_MM * (6 - 2.25), TILE_SIZE_MM*0.9),
            DriveIntoGoal(DriveForward::getCommand(TILE_SIZE_MM*0.9)), 
            Score(), //=====================

            DriveForward::getCommand(-TILE_SIZE_MM*0.75), 
            DriveIntoElement(DriveToSetpoint::getCommand(TILE_SIZE_MM * 4.95, TILE_SIZE_MM * 0.8, RouteType::EUCLIDEAN)/*DriveForward::getCommand(TILE_SIZE_MM * 1.35)*/),  
            ParallelCommandGroup::makeGroup({ 
              FrontRunElevatorSetpoint::getCommand(8, 35.4),
              SequentialCommandGroup::makeGroup({
                    WaitFor::getCommand(600), 
                    DriveIntoGoal(MoveToPoint::getCommand(TILE_SIZE_MM * (6 - 2.15), TILE_SIZE_MM * 0.85, true, 0))
                  })  
             }),  
            Score(),//=====================     

            ResetPose::getCommand({ResetFace::LEFT, ResetFace::BACK}),
            AlignWithAxis::getCommand(TILE_SIZE_MM * 5.5, AlignmentAxis::HORI), 
            TurnToHeading::getCommand(270), 
            DriveIntoElement(DriveForward::getCommand(TILE_SIZE_MM)),  
            ParallelCommandGroup::makeGroup({
                SequentialCommandGroup::makeGroup({ 
                    WaitFor::getCommand(350),
                    FrontRunElevatorSetpoint::getCommand(11, 25.4)
                }),
                AlignWithAxis::getCommand(TILE_SIZE_MM * 0.75, AlignmentAxis::VERT), 
            }),
            DriveIntoGoal(DriveToSetpoint::getCommand(TILE_SIZE_MM * 4.05, TILE_SIZE_MM * 0.4, RouteType::EUCLIDEAN)), 
            Score(), //===================== 

            AlignWithAxis::getCommand(TILE_SIZE_MM * 5.5, AlignmentAxis::HORI), 
            TurnToHeading::getCommand(270),
            DriveIntoElement(DriveForward::getCommand(TILE_SIZE_MM * 1.25)),
            DriveForward::getCommand(-TILE_SIZE_MM*0.5),
            FrontRunElevatorSetpoint::getCommand(14, 25.4), 
            WaitFor::getCommand(400),
            DriveIntoGoal(DriveToSetpoint::getCommand(TILE_SIZE_MM * 4, TILE_SIZE_MM * 0.4, RouteType::EUCLIDEAN)), 
            Score() //===================== 

         });
}


