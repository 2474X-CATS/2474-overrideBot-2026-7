#include "elevator.h" 
#include "../utilities/functools.h" 

Elevator* Elevator::globalPtr = nullptr;

double Elevator::GROUND_INTAKE_HEIGHT = 100;  
double Elevator::LEVELED_HEIGHT = 0; 
       
double Elevator::MAX_HEIGHT = 1000;

double Elevator::ELEVATOR_ERROR_TOLERANCE = 3; 
double Elevator::STACK_HEIGHT = 164.5;  

double Elevator::PRIMING_SPEED = 500; //Every half second

double Elevator::MINIMUM_ALIGNER_DISTANCE = 0.0;  
double Elevator::ALIGNER_ERROR_TOLERANCE = 0.0; 


Elevator& Elevator::getObject(){ 
  return *globalPtr;
}

void Elevator::init(){ 
    
    set<double>("current_height", 0);
     
    motionConsts.maxVelocity = 3000; 
    motionConsts.maxAcceleration = 3000; 

} 

void Elevator::periodic(){  
   double elevatorVelocity; 

   if (currentState == ElevatorState::E_PRIMING){ //Rise or fall at a constant rate
     elevatorVelocity = PRIMING_SPEED;
   } else if (currentState == ElevatorState::E_PURSUING){  //Pursuing a setpoint
     TrapezoidalSetpoint motionGoal = motionProfile->generateSetpoint(Brain.Timer.time());
     elevatorVelocity = motionGoal.velocity * 1.0 * setpointDirection;
   }  else {
     elevatorVelocity = PRIMING_SPEED * raisingDirection;
   }
   updatePosition(elevatorVelocity);
}  

void Elevator::updateTelemetry(){     
    // Update status of stack sight  
    set<bool>("sensing_stack", get<double>("current_height") < 300); 
    stateControl();
    if (!RobotState::getStateOf("in_autonomous")){ 
       respondToRequests();
    } else { 
       raisingDirection = 0;
    } 
} 

void Elevator::stop(){ 
  return;
}

void Elevator::updatePosition(double velocity){    
  double frameTranslation = velocity * (20.0 / 1000.0); 
  set<double>("current_height", min<double>(max<double>(get<double>("current_height") + frameTranslation, 0), MAX_HEIGHT)); 
  previousTimestamp = Brain.Timer.time();
}

void Elevator::setSetpoint(double setpoint){  
   double error = setpoint - get<double>("current_height");  
   setpointDirection = copysign(1, error); 
   error = fabs(error); 
   if (error < ELEVATOR_ERROR_TOLERANCE){ 
    return;
   } 
   motionProfile = new TrapezoidalMotionProfile(motionConsts, error, 0, 0);   
   motionProfile->setLastTimestamp(Brain.Timer.time());
   currentState = ElevatorState::E_PURSUING;
}

bool Elevator::reachedSetpoint(){ 
   return Brain.Timer.time() - motionProfile->getStartTime() >= motionProfile->getTotalDuration(); 
}

void Elevator::stateControl(){   

    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));   
    
    if (get<bool>("requesting_setpoint")){   
       if (get<bool>("sniper_score_enabled")){ 
         set<double>("priming_setpoint", get<double>("requested_height"));
       } else {
         setSetpoint(get<double>("requested_height"));
       }
       set<bool>("requesting_setpoint", false);
    } 
    
    if (get<bool>("hold")){ 
      if (sin(toRadians(Telemetry::inst.getValueAt<double>("forearm", "current_angle"))) > -0.7){ 
         set<bool>("hold", false);
      }
    }

    if (currentState == ElevatorState::E_PURSUING){ //
      if (reachedSetpoint()){   
        currentState = ElevatorState::E_HOLDING;   
      }
    } else if (currentState == ElevatorState::E_PRIMING){   
        if (!get<bool>("sensing_stack")){
          currentState = ElevatorState::E_HOLDING; 
          set<bool>("active", false);   
          set<int>("task_id", get<int>("task_id") + 1); 

          if (get<int>("task_id") == 1){ 
             Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
          } else { 
             Telemetry::inst.placeValueAt<bool>(true, "ss_manager", "task_completed");    
             set<int>("task_id", 0);
             set<bool>("requesting_setpoint", true);
             set<double>("requested_height", GROUND_INTAKE_HEIGHT);
          }
          
        }
    } else if (!get<bool>("hold")) {
        switch (pos){
            case AUTO:
               if (get<bool>("active")){   
                 if (get<bool>("sniper_score_enabled")){  
                   set<bool>("sniper_score_enabled", false);
                   setSetpoint(get<double>("priming_setpoint"));
                 } else { 
                   currentState = ElevatorState::E_PRIMING; 
                 }
               } 
               break; 
            case GROUND:  
               set<bool>("requesting_setpoint", true);
               set<double>("requested_height", GROUND_INTAKE_HEIGHT);   
               break; 
            case STANDING: 
               set<bool>("requesting_setpoint", true); 
               set<double>("requested_height", LEVELED_HEIGHT); 
               break;
            case PRIMED:
               if (get<bool>("sniper_score_enabled")){ 
                   setSetpoint(get<double>("priming_setpoint")); 
                   set<bool>("sniper_score_enabled", false);
               } else if (!Telemetry::inst.getValueAt<bool>("claw", "senses_object")){ 
                   set<bool>("requesting_setpoint", true); 
                   set<double>("requested_height", GROUND_INTAKE_HEIGHT);  
               }
               break;
            default: 
               break;
        } 
    } 


    set<bool>("at_setpoint", currentState == ElevatorState::E_HOLDING && !get<bool>("hold"));  
   
}

void Elevator::respondToRequests(){ 
    if (currentState == E_PRIMING || currentState == E_PURSUING){ 
      return;
    } 

    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
    
    if (pos == SuperStructurePosition::PRIMED && Telemetry::inst.getValueAt<bool>("ss_manager", "setpoints_reached") && Telemetry::inst.getValueAt<bool>("claw", "senses_object")){   
      if (RobotState::getStateOf("awaiting_land")){ 
        set<bool>("requesting_setpoint", true); 
        set<double>("requested_height", GROUND_INTAKE_HEIGHT); 
        return;
      }  
      if (RobotState::getStateOf("rise")){ 
        currentState = E_HOLDING; 
        raisingDirection = 1;
      } else if (RobotState::getStateOf("fall")){ 
        raisingDirection = -1; 
        currentState = E_HOLDING; 
      } else {   
        currentState = E_HOLDING;
        raisingDirection = 0;
      }
    } else { 
      raisingDirection = 0;
    }
}