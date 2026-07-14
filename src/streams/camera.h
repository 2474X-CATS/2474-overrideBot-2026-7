#ifndef __CAMERA_H__ 
#define __CAMERA_H__ 

#include "../architecture/dataStream.h" 
#include "../utilities/functools.h"  

typedef struct { 
   double relativeX; 
   double relativeY; 
   double facingAngle; 
} CameraOrientation; 


class Camera : public DataStream {   

    private: 
        
       static double CAMERA_PIXEL_WIDTH; 
       static double CAMERA_PIXEL_HEIGHT;

       static double getFocalWidth(); 
       static double getFocalHeight();

       static double YAW_FOV; 
       static double PITCH_FOV;   
       
       CameraOrientation orientation; 

    protected:   
       
       using DataStream::set; 

       vex::aivision* camera;    
       
       double getDistFromCamera(vex::aivision::object obj);
       double getObjectPitch(vex::aivision::object obj); 
       double getObjectYaw(vex::aivision::object obj);   

       void robotToTarget(vex::aivision::object obj, double* angleTransLoc, double* distTransLoc);   
       
       virtual double getObjectHeight() = 0; 
       virtual void initializeCamera(int32_t port) = 0;

    public:   

       Camera(string cameraDesc, vector<EntrySet> entries, CameraOrientation orient, int32_t port) :  
       DataStream(cameraDesc, entries), 
       orientation(orient)
       {};


};



#endif