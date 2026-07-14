#ifndef __TAG_CAMERA_H__ 
#define __TAG_CAMERA_H__  

#include "camera.h"

class AprilTagCamera : public Camera {  
    private: 
       static double TAG_SIZE_MM; 

       void setToAveragedRelativePose(); 
       void setToLargestRelativePose(); 

    public: 
       AprilTagCamera(int32_t port, CameraOrientation orient) :  
       Camera(  
          "tag_camera", 
          { 
            (EntrySet){"target_found", EntryType::BOOL}, 
            (EntrySet){"target_id", EntryType::INT}, 
            (EntrySet){"dist_from_robot", EntryType::DOUBLE}, 
            (EntrySet){"angle_from_robot", EntryType::DOUBLE}, 
            (EntrySet){"locked_in", EntryType::BOOL}, 
            (EntrySet){"blind", EntryType::BOOL}, 
            (EntrySet){"requesting_lock", EntryType::BOOL}
          }, 
          orient, 
          port
        )
        { 
         initializeCamera(port);
        }; 

       void init() override;  
       void refreshData() override; 

       void initializeCamera(int32_t port) override; 
       double getObjectHeight() override;   

      

};



#endif 
