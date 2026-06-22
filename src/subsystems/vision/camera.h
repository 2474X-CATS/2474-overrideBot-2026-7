#ifndef __CAMERA_H__ 
#define __CAMERA_H__ 


#include "../../architecture/dataStream.h" 

typedef struct { 
   double relativeX; 
   double relativeY; 
   double facingAngle; 
} CameraOrientation; 


class Camera : public DataStream {   

    private: 
       
       static int RESOLUTION_X; 
       static int RESOLUTION_Y; 
       static double YAW_FOV; 
       static double PITCH_FOV;   
       
       void getTranslationToTarget(int objectIndex, double* xTransLoc, double* yTransLoc);
       
       CameraOrientation orientation;

    protected:   
       
       using DataStream::set; 

       vex::aivision camera;    
       
       virtual void initializeCamera(int32_t smartPort) = 0;//Abstract  

       virtual bool shouldRecord() = 0; //Abstract [Is in recording mode?]
       virtual void takeSnapshot() = 0; //Abstract [Update camera objects]
       virtual void aggregateData() = 0; //Abstract [Plaster data onto subtables]

       double getDistFromCamera(int objectIndex);
       double getObjectPitch(int objectIndex); 
       double getObjectYaw(int objectIndex);   

       array<double, 2> robotToTarget(int objectIndex);  //X_trans, Y_trans
       void robotToTarget(int objectIndex, double* xTransLoc, double* yTransLoc);  

  
    public:  
       Camera(string cameraDesc, vector<EntrySet> entries, CameraOrientation orient, int32_t port) :  
       DataStream(cameraDesc, entries), 
       orientation(orient)
       {
           initializeCamera(port); 
       }
 
       void init() override; 
       void refreshData() override;
       
};



#endif