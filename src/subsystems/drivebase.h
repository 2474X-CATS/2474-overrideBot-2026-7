#ifndef __DRIVEBASE_H__ 
#define __DRIVEBASE_H__ 

#include "../architecture/subsystem.h"   
#include "../architecture/command.h"  

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"  

#include "../streams/odometry.h" 

#include "../control/path.h" 
#include "../control/pursuit.h"


class Drivebase : public Subsystem { 
    public:
      using Subsystem::get;  
      
      static const double MAX_RPM; 
      static const double WHEEL_RADIUS_MM; 

      static const double MAX_LIN_SPEED;
      static const double MAX_ANG_SPEED;
      
      static Drivebase& getObject();

      Drivebase();
      
      void manualDrive(double voltageDrive, double voltageTurn);  

      //---------------------------------------------------- 

      void setSpeeds(double linearVelocity, double angularVelocity);

    private:
      static Drivebase* globalPtr;  
       
      //Trajectory* path = nullptr;
      //std::vector<Point> points; 
      //PurePursuit* donkey = nullptr;

      static const double TURN_SENSITIVITY; 
      static const double DRIVE_SENSITIVITY; 
      
      static const double ACCELERATION_LIMIT_LIN; 
      static const double ACCELERATION_LIMIT_ANG;

      double lastLinearVoltage = 0;
      double lastAngularVoltage = 0;

      vex::motor leftFront; 
      vex::motor leftBack; 

      vex::motor rightFront; 
      vex::motor rightBack;  
      
      //vex::motor leftExtra; 
      //vex::motor rightExtra;

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

     double startX;
     double startY;  

     double initialAngle; 

     double percentAccel; 
     double percentVelo; 

     double getDistTraveled(); 
     
     TrapezoidalMotionProfile* motionProfile = nullptr;  
     pidcontroller* controller = nullptr; 
     FFConstants ffController; 

     pidcontroller* straightener = nullptr;

     static const double MOTION_CONSTANTS_MAX_VELO; 
     static const double MOTION_CONSTANTS_MAX_ACCEL; 

     static const double PID_CONSTANTS_KP;
     static const double PID_CONSTANTS_KI;
     static const double PID_CONSTANTS_KD;

     static const double FF_CONSTANTS_S;
     static const double FF_CONSTANTS_V;
     static const double FF_CONSTANTS_A; 

     static const double STRAIGHTEN_PID_KP; 
     static const double STRAIGHTEN_PID_KI; 
     static const double STRAIGHTEN_PID_KD;
   
   public:

     static CommandInterface* getCommand(double distance){ 
         return new DriveForward(Drivebase::getObject(), distance);
     } 

     static CommandInterface* getCommand(double distance, double percentVelocity, double percentAcceleration){ 
         return new DriveForward(Drivebase::getObject(), distance, percentVelocity, percentAcceleration);
     }

     DriveForward(Drivebase& drive, double dist, double percentVelocity, double percentAcceleration):  
     Command<Drivebase>(drive),
     drivebaseRef(drive),
     distance(dist), 
     percentVelo(percentVelocity), 
     percentAccel(percentAcceleration)
     {};

     DriveForward(Drivebase& drive, double dist): 
     DriveForward(drive, dist, 100, 100){}; 

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

     double getError();

   public: 

     static const double PID_CONSTANTS_KP;
     static const double PID_CONSTANTS_KI;
     static const double PID_CONSTANTS_KD; 

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

//-------------------------------------------------------- 

class DriveTrajectory : public Command<Drivebase> {   

  private:  

    Drivebase& drivebaseRef;  

    PurePursuit* donkey = nullptr; 
    Trajectory* trajectory = nullptr;    
    
    pidcontroller* turnController = nullptr; 

    double maxAccel; 
    double maxVelo; 

    void findNextSpeeds(double linearError, double angularError, double& linearSpeed, double& angularSpeed); 

  public: 
    DriveTrajectory(Drivebase& drivebase, double lookAheadDist, double maxVelocity, double maxAcceleration);
  
  protected:   
    double lDist; //Same throughout extended classes
    virtual void initializePath(){}; //Actually make the trajectory   

    void start() override; //Call initializePath and partitions trajectory
    void periodic() override; //Follow path
    bool isOver() override; //Reached end of path
    void end() override; //Stop
    
}; 

class Toggle : public Command<Drivebase> { 
   
   private:  

     static CommandInterface* getCommand(){ 
       return new Toggle(Drivebase::getObject()); 
     }  

     double startTime; 

   public: 
   
     static CommandInterface* getCommand(int bounces){ 
         SequentialCommandGroup* group = SequentialCommandGroup::makeGroup(Toggle::getCommand()); 
         for (int i = 0; i < bounces - 1; i++){ 
          group->chainThen(Toggle::getCommand());
         } 
         return group; 
     }

     Toggle(Drivebase& drive):  
     Command<Drivebase>(drive),
     drivebaseRef(drive)
     {}; 

   protected: 
     Drivebase& drivebaseRef;  

     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override; 
};



#endif