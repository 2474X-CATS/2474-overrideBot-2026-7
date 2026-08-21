#ifndef __DRIVEBASE_H__ 
#define __DRIVEBASE_H__ 


#include "../architecture/subsystem.h"  
#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h" 
#include "../architecture/command.h" 
#include "../streams/odometry.h"

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
      leftFront(vex::motor(vex::PORT20)), 
      leftBack(vex::motor(vex::PORT19)), 
      rightFront(vex::motor(vex::PORT17)), 
      rightBack(vex::motor(vex::PORT18)), 
      leftMotors(leftFront, leftBack), 
      rightMotors(rightFront, rightBack)
      { 
        globalPtr = this;
      };
      
      
      void manualDrive(double voltageDrive, double voltageTurn); 
      

    private:     
      static Drivebase* globalPtr; 
      static double TURN_SENSITIVITY; 
      static double DRIVE_SENSITIVITY;

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
     int direction = 0; 

     double startX;
     double startY;  

     double initialAngle;

     double getDistTraveled(); 
     
     TrapezoidalMotionProfile* motionProfile = nullptr;  
     pidcontroller* controller = nullptr; 
     FeedForward* ffController = nullptr; 

     pidcontroller* straightener = nullptr;

     static double MOTION_CONSTANTS_MAX_VELO; 
     static double MOTION_CONSTANTS_MAX_ACCEL; 

     static double PID_CONSTANTS_KP;
     static double PID_CONSTANTS_KI;
     static double PID_CONSTANTS_KD;

     static double FF_CONSTANTS_S;
     static double FF_CONSTANTS_V;
     static double FF_CONSTANTS_A; 

     static double STRAIGHTEN_PID_KP; 
     static double STRAIGHTEN_PID_KI; 
     static double STRAIGHTEN_PID_KD;
   
   public:

     static CommandInterface* getCommand(double distance){ 
         return new DriveForward(Drivebase::getObject(), distance);
     }

     DriveForward(Drivebase& drive, double dist):  
     Command<Drivebase>(drive),
     drivebaseRef(drive),
     distance(dist)
     {}; 

     void setDistance(double distance);

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

     void setAngle(double angle); 

   protected: 
     Drivebase& drivebaseRef;  

     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override; 

};

//--------------------------------------------------------------------------- 

class FaceTarget : public TurnToHeading {  
  private:  

    double targetX; 
    double targetY;
 
  public:  

    static CommandInterface* getCommand(double tX, double tY){ 
      return new FaceTarget(Drivebase::getObject(), tX, tY);
    }  

    static CommandInterface* getCommand(Setpoint setp){  
      Location* location = Odometry::getLocation(setp);
      return getCommand(location->getX(), location->getY());
    } 


    FaceTarget(Drivebase& drive, double tX, double tY): 
    TurnToHeading(drive, 0), 
    targetX(tX),
    targetY(tY){}; 

  protected: 
     void start() override;
};   

//----------------------------------------------------------------- 

class ApproachTarget : public DriveForward {

  private:
    double targetX; 
    double targetY; 

    double offset;

  public:
    
    static CommandInterface* getCommand(double tX, double tY, double tOffset){ 
      return new ApproachTarget(Drivebase::getObject(), tX, tY, tOffset);
    }  

    static CommandInterface* getCommand(double tX, double tY){ 
      return new ApproachTarget(Drivebase::getObject(), tX, tY, 0);
    } 

    static CommandInterface* getCommand(Setpoint setp){  
      Location* location = Odometry::getLocation(setp);
      return getCommand(location->getX(), location->getY(), location->getRadius());
    } 

    ApproachTarget(Drivebase& drive, double tX, double tY, double off): 
    DriveForward(drive, 0),
    targetX(tX),
    targetY(tY),
    offset(off)
    {};

  protected: 
     void start() override;
};

//----------------------------------------------------------------- 

typedef enum { 
   EUCLIDEAN,
   MANHATTAN_XY,
   MANHATTAN_YX
} RouteType;

class DriveToSetpoint : public SequentialCommandGroup {   

  private:

    RouteType path;

    double setpointX;
    double setpointY;

    void calibrateSetpoints_man_xy(double currentX, double currentY, double currentAngle);
    void calibrateSetpoints_man_yx(double currentX, double currentY, double currentAngle); 
    void calibrateSetpoints_euc(double currentX, double currentY);

  public: 
    
    static CommandInterface* getCommand(Setpoint setp, RouteType type){ 
      Location* location = Odometry::getLocation(setp); 
      return new DriveToSetpoint(location->getX(), location->getY(), type, location->getRadius());
    }

    static CommandInterface* getCommand(double x, double y, RouteType route){ 
       return new DriveToSetpoint(x, y, route, 0);
    }  

    DriveToSetpoint(double x, double y, RouteType route, double offset) :  
    SequentialCommandGroup(TurnToHeading::getCommand(0)), 
    setpointX(x),
    setpointY(y),
    path(route)
    {  
      if (path != RouteType::EUCLIDEAN){ 
          chainThen(DriveForward::getCommand(0))-> 
          chainThen(TurnToHeading::getCommand(0))-> 
          chainThen(ApproachTarget::getCommand(x,y,offset));
      } else { 
          chainThen(ApproachTarget::getCommand(x,y,offset));
      }
    } 

  protected:
      
      void start() override;

}; 



#endif