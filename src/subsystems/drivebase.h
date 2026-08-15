#ifndef __DRIVEBASE_H__ 
#define __DRIVEBASE_H__ 


#include "../architecture/subsystem.h"  
#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h" 
#include "../architecture/command.h"

class Drivebase : public Subsystem { 
    public:  
      using Subsystem::get;  
      
      static double MAX_RPM; 
      static double WHEEL_RADIUS_MM;
      
      static Drivebase& getObject(); 

      Drivebase(): 
      Subsystem( 
        "drivebase", 
        { 
          (EntrySet){"is_on", EntryType::BOOL}
        }
      ), 
      leftFront(vex::motor(vex::PORT7)), 
      leftBack(vex::motor(vex::PORT6)), 
      rightFront(vex::motor(vex::PORT5)), 
      rightBack(vex::motor(vex::PORT4)), 
      leftMotors(leftFront, leftBack), 
      rightMotors(rightFront, rightBack)
      { 
        globalPtr = this;
      };
      
      void manualDrive(double voltage); 
      
      void manualSpin(double voltage); //Negative for the opposite direction


    private:     
      static Drivebase* globalPtr;

      vex::motor leftFront; 
      vex::motor leftBack; 

      vex::motor rightFront; 
      vex::motor rightBack;  

      vex::motor_group leftMotors;
      vex::motor_group rightMotors;
      
      void arcadeDrive(double speed, double rotation); 

    protected: 
      using Subsystem::set; 

      void init() override;
      void periodic() override; 
      void updateTelemetry() override; 
      void stop() override;
}; 

//--------------------------------------------------------------------------------- 


class DriveForward : public Command<Drivebase> {  
   
   private: 
     double distance;
     int direction;
     
     TrapezoidalMotionProfile* motionProfile = nullptr;  
     errorcontroller* controller = nullptr; 
     FeedForward* ffController = nullptr;

     static double MOTION_CONSTANTS_MAX_VELO; 
     static double MOTION_CONSTANTS_MAX_ACCEL; 

     static double PID_CONSTANTS_KP;
     static double PID_CONSTANTS_KI;
     static double PID_CONSTANTS_KD;

     static double FF_CONSTANTS_S;
     static double FF_CONSTANTS_V;
     static double FF_CONSTANTS_A;
  
   public:

     static CommandInterface* getCommand(double distance){ 
         return new DriveForward(Drivebase::getObject(), distance);
     }

     DriveForward(Drivebase& drive, double dist):  
     Command<Drivebase>(drive),
     drivebaseRef(drive), 
     direction(copysign(1, dist)),
     distance(fabs(dist))
     {};

   protected: 
     Drivebase& drivebaseRef;  

     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override; 
}; 

//-----------------------------------------------------------------

class TurnToHeading : public Command<Drivebase> {  
   
   private: 
     
     double setpoint; 

     pidcontroller* controller = nullptr; 

     static double PID_CONSTANTS_KP;
     static double PID_CONSTANTS_KI;
     static double PID_CONSTANTS_KD;  

     double getError(); 

   public:

     static CommandInterface* getCommand(double angle){ 
         return new TurnToHeading(Drivebase::getObject(), angle);
     }

     TurnToHeading(Drivebase& drive, double angle):  
     Command<Drivebase>(drive),
     drivebaseRef(drive), 
     setpoint(angle)
     {};

   protected: 
     Drivebase& drivebaseRef;  

     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override; 

};


#endif