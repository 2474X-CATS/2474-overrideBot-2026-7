#ifndef __DRIVEBASE_H__ 
#define __DRIVEBASE_H__ 

#include "../architecture/subsystem.h"   
#include "../architecture/command.h"  

#include "../control/feedForward.h" 
#include "../control/pidcontroller.h" 
#include "../control/trapezoidalMotion.h"  

#include "../streams/odometry.h" 

//#include "../control/path.h"
//#include "../control/pursuit.h"

class Drivebase : public Subsystem { 
    public:
      using Subsystem::get;  
      
      static const double MAX_RPM; 
      static const double WHEEL_RADIUS_MM; 

      static const double MAX_LIN_SPEED;
      static const double MAX_ANG_SPEED;
      
      static Drivebase& getObject();

      Drivebase(); 

      Drivebase(bool usingTendy); 
      
      void manualDrive(double voltageDrive, double voltageTurn);   

      void tankDrive(double leftSpeed, double rightSpeed);

      //---------------------------------------------------- 

      void setSpeeds(double linearVelocity, double angularVelocity);

    private:
      static Drivebase* globalPtr;  
 
      static const double TURN_SENSITIVITY; 
      static const double DRIVE_SENSITIVITY; 
      
      static const double ACCELERATION_LIMIT_LIN; 
      static const double ACCELERATION_LIMIT_ANG;

      double lastLinearVoltage = 0;
      double lastAngularVoltage = 0;

      vex::motor leftFront;  
      vex::motor leftMid;
      vex::motor leftBack; 

      vex::motor rightFront;  
      vex::motor rightMid; 
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

class PivotToAngle : public Command<Drivebase> { 
   
   private:     

      static const double PIV_PID_CONSTANTS_KP;
      static const double PIV_PID_CONSTANTS_KI;
      static const double PIV_PID_CONSTANTS_KD; 

      double setpointAngle;   
      pidcontroller* controller = nullptr; 

      Drivebase& drivebaseRef;

   public:

     static CommandInterface* getCommand(double angle){ 
         return new PivotToAngle(Drivebase::getObject(), angle);
     } 
    

     static CommandInterface* getCommand(Setpoint pivotPoint, double x, double y, bool faceAway){
         Location* pivotLoc = Odometry::getLocation(pivotPoint);
         double angle = angleSum(toDegrees(atan2(y - pivotLoc->getY(), x - pivotLoc->getX())), 0); 
         if (faceAway){ 
          angle = angleSum(angle, 180);
         }
         return getCommand(angle); 
     }

     PivotToAngle(Drivebase& drive, double angle):
     Command<Drivebase>(drive),
     drivebaseRef(drive),
     setpointAngle(angle)
     {};

   protected:
     
     void start() override; 
     void periodic() override; 
     bool isOver() override; 
     void end() override;
};

//--------------------------------------------------------------------------------- 

class DriveForward : public Command<Drivebase> {  
   
   private: 
     double distance;

     double initialAngle;

     double getDistTraveled();
     
     pidcontroller* controller = nullptr; 
  
     pidcontroller* straightener = nullptr;

     static const double STRAIGHTEN_PID_KP; 
     static const double STRAIGHTEN_PID_KI; 
     static const double STRAIGHTEN_PID_KD;
   
   public:
      
     static const double PID_CONSTANTS_KP;
     static const double PID_CONSTANTS_KI;
     static const double PID_CONSTANTS_KD; 

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

     virtual double getAnchorAngle(){return initialAngle;};  

     double startX;
     double startY;
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

class MoveToPoint : public Command<Drivebase> {   

  private:  
    
    double setpX;
    double setpY;  

    bool cent;

    pidcontroller* linController = nullptr; 
    pidcontroller* angController = nullptr;

    Drivebase& drivebaseRef;  

    double getLinError(); 
    double getAngError();  

    double offsetDist;
  
  public:
    
    static CommandInterface* getCommand(double setpointX, double setpointY, bool centered, double offset){ 
      return new MoveToPoint(Drivebase::getObject(), setpointX, setpointY, centered, offset); 
    }

    static CommandInterface* getCommand(Setpoint setpoint, bool centered){ 
      Location* loc = Odometry::getLocation(setpoint); 
      return getCommand(loc->getX(), loc->getY(), centered, loc->getRadius()); 
    }

    MoveToPoint(Drivebase& drive, double setpointX, double setpointY, bool centered, double offset): 
    Command<Drivebase>(drive),
    drivebaseRef(drive),
    setpX(setpointX),
    setpY(setpointY),
    cent(centered), 
    offsetDist(offset) 
    {};
  
  protected:  

    void start() override; //Call initializePath and partitions trajectory
    void periodic() override; //Follow path
    bool isOver() override; //Reached end of path
    void end() override; //Stop
  
}; 

//-------------------------------------------------------- 

typedef enum { 
   HORI, 
   VERT
} AlignmentAxis;

class AlignWithAxis : DriveForward {
  private: 

    double setpointPos; 
    AlignmentAxis ax;

  public:
    
    static CommandInterface* getCommand(double coordinate, AlignmentAxis axis){ 
       return new AlignWithAxis(Drivebase::getObject(), coordinate, axis);
    }  

    static CommandInterface* getCommand(Setpoint setpoint, AlignmentAxis axis){ 
       Location* loc = Odometry::getLocation(setpoint); 
       if (axis == AlignmentAxis::HORI){ 
         return getCommand(loc->getX(), axis);
       } else { 
         return getCommand(loc->getY(), axis);
       }
    }

    AlignWithAxis(Drivebase& drive, double coordinate, AlignmentAxis axis): 
    DriveForward(drive, 0),
    setpointPos(coordinate), 
    ax(axis)
    {};

  protected: 
     void start() override;
};

//-------------------------------------------------------- 

/*
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
*/


#endif