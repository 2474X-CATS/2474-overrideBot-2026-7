#ifndef __APRIL_VISION_H__ 
#define __APRIL_VISION_H__ 

#include "camera.h" 



class TagCamera : public Camera {  
    
    private:    
       void localizedPosition(int objectIndex, double* xLoc, double* yLoc, double* headingLoc); 

       array<double, 3> getAveragedPosition(); 
       
    public: 
       TagCamera(CameraOrientation orient, int32_t smartPort) :
       Camera( 
        "tag_camera",
        { 
          (EntrySet){"a_target_dist", EntryType::DOUBLE}, //Distance to target
          (EntrySet){"a_target_theta", EntryType::DOUBLE}, //Yaw to target
          (EntrySet){"a_target_id", EntryType::INT}, //Alignment target [Alliance / Neutral / Mid Neutral]
          
          (EntrySet){"aligning", EntryType::BOOL}, //Are we aligning with a tag/goal 

          (EntrySet){"l_estimated_x", EntryType::DOUBLE}, //Estimated robot x_position
          (EntrySet){"l_estimated_y", EntryType::DOUBLE}, //Estimated robot y position
          (EntrySet){"l_estimated_angle", EntryType::DOUBLE}, //Estimated robot heading

          (EntrySet){"localizing", EntryType::DOUBLE} //Are we localizing odometry
        },  
        orient, 
        smartPort
       )
       {};  
    
    protected:    
       
       void takeSnapshot() override; 
       bool shouldRecord() override;  
       void aggregateData() override;

};



#endif