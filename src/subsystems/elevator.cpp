#include "elevator.h" 

Elevator* Elevator::globalPtr = nullptr;

double Elevator::GROUND_INTAKE_HEIGHT = 100;  
double Elevator::LEVELED_HEIGHT = 0; 
       
double Elevator::MAX_HEIGHT = 1000;

double Elevator::ELEVATOR_ERROR_TOLERANCE = 3; 
double Elevator::STACK_HEIGHT = 0.0;  

double Elevator::PRIMING_SPEED = 0.0; 

double Elevator::MINIMUM_ALIGNER_DISTANCE = 0.0;  
double Elevator::ALIGNER_ERROR_TOLERANCE = 0.0; 


Elevator& Elevator::getObject(){ 
  return *globalPtr;
}

void Elevator::init(){ 
    //Set up all the constants   

    //Motion: Max speed and acceleration 
    set<double>("current_height", 200);
     
    motionConsts.maxVelocity = 1000; 
    motionConsts.maxAcceleration = 250; 

} 

void Elevator::periodic(){  
   double elevatorVelocity;
   if (currentState == ElevatorState::E_HOLDING){//Stay Still
     elevatorVelocity = PRIMING_SPEED * raisingDirection; 
   } else if (currentState == ElevatorState::E_PRIMING){ //Rise or fall at a constant rate
     elevatorVelocity = PRIMING_SPEED;
   } else {  //Pursuing a setpoint
     TrapezoidalSetpoint motionGoal = motionProfile->generateSetpoint(Brain.Timer.time());
     elevatorVelocity = motionGoal.velocity * 1.0 * setpointDirection;
   } 
   updatePosition(elevatorVelocity);
}  

void Elevator::updateTelemetry(){     
    // Update status of stack sight  
    Brain.Screen.printAt(20, 60, "Elevator Height: %.2f", get<double>("current_height"));   
    
    string setpointMessage; 
    if (currentState == ElevatorState::E_PURSUING){ 
     setpointMessage = "PURSUING";
    } else if (currentState == ElevatorState::E_PRIMING){ 
     setpointMessage = "PRIMING";
    } else { 
     setpointMessage = "HOLDING"; 
    }  
    Brain.Screen.printAt(20, 80, setpointMessage.c_str()); 
    Brain.Screen.printAt(70, 80, "- %d", get<bool>("responded"));
    
    set<bool>("sensing_stack", false); 
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
  set<double>("current_height", get<double>("current_height") + frameTranslation); 
  previousTimestamp = Brain.Timer.time();
}


void Elevator::setSetpoint(double setpoint){ 
   double error = setpoint - get<double>("current_height");  
   setpointDirection = copysign(1, error); 
   error = fabs(error); 
   if (error < ELEVATOR_ERROR_TOLERANCE || currentState != ElevatorState::E_HOLDING){ 
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
      set<bool>("requesting_setpoint", false); 
      setSetpoint(get<double>("requested_height"));
    }

    if (currentState == ElevatorState::E_PURSUING){ //
      if (reachedSetpoint()){ 
        currentState = ElevatorState::E_HOLDING;
      }
    } else if (currentState == ElevatorState::E_PRIMING){ 
        if (!get<bool>("sensing_stack")){ 
          currentState = ElevatorState::E_HOLDING; 
          set<bool>("active", false); 
          Telemetry::inst.placeValueAt<bool>(true, "forearm", "active");
        }
    } else { 
        switch (pos){ 
            case AUTO: 
               if (get<bool>("active")){  
                 currentState = ElevatorState::E_PRIMING;
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
            default: 
               break;
        }
        
    }  
    

    /*  
    - Needs to check if the elevator *has* a setpoint yet 
      [This is essentially checking if there is an important height the elevator needs 
      to reach. If there is a named position or a known height we want to reach then we will  
      have a setpoint. In this case the motion profile will not be a nullptr] 
    - If we don't have a known setpoint though then 
    | 
    v
    If in the GROUND position or STANDING position we will have a setpoint (specifically the ground position) 
    | 
    v 
    Because we want to go to specific heights in the autonomous period then we will have setpoints (but this
    can be controlled by an entry "requested_height") In fact ground and standing will also run the same  
    protocol. Modifying requested_height and has_setpoint to show that we want to go to a specifc height.  
    | 
    v  
    Finally (actually this should be at the start) when we are running the macro the first stage is to prime 
    at least when we aren't in auto. (Going to make a prime function)
    */   

   set<bool>("at_setpoint", currentState == ElevatorState::E_HOLDING); 
   set<bool>("responded", true);
   
}

void Elevator::respondToRequests(){ 
    SuperStructurePosition pos = static_cast<SuperStructurePosition>(Telemetry::inst.getValueAt<int>("ss_manager", "position"));
    //respond to driver/operator input 
    if (pos == SuperStructurePosition::PRIMED && currentState == ElevatorState::E_HOLDING){ 
      raisingDirection = ((int)RobotState::getAxisState(AxisType::S_LEFT_VERTICAL)) / 100;  
      if (get<double>("current_height") >= MAX_HEIGHT){ 
        raisingDirection = min<int>(raisingDirection, 0);
      } else if (get<double>("current_height") <= GROUND_INTAKE_HEIGHT){ 
        raisingDirection = max<int>(raisingDirection, 0);
      }
    }
}