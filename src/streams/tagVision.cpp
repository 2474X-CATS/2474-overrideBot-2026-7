#include "tagVision.h" 


double AprilTagCamera::TAG_SIZE_MM = 25.4 * 1.5;

double AprilTagCamera::getObjectHeight(){ 
    return TAG_SIZE_MM;
} 

void AprilTagCamera::initializeCamera(int32_t port){ 
    camera = new vex::aivision(port, vex::aivision::ALL_TAGS);  
} 

void AprilTagCamera::init(){ 
    set<bool>("blind", true);
} 

void AprilTagCamera::setToAveragedRelativePose(){    
    double polarAngle = -1;
    double polarDist = -1;
    
    if (!camera->objects[0].exists){ 
        return; 
    } 

    switch (camera->objectCount){ 
        case 1: 
           robotToTarget(camera->objects[0], polarAngle, polarDist); 
           break; 
        case 2:   
           double yaw1;
           double yaw2;  

           double dist1;
           double dist2; 

           robotToTarget(camera->objects[0], yaw1, dist1); 
           robotToTarget(camera->objects[1], yaw2, dist2);  

           yaw1 = toRadians(yaw1); 
           yaw2 = toRadians(yaw2);
           
           polarAngle = toDegrees(atan2((sin(yaw1) + sin(yaw2)) / 2, (cos(yaw1) + cos(yaw2)) / 2));
    
           double avgDist = (dist1 + dist2) / 2; 
           double hypotDist = hypot(dist1, dist2); 

           polarDist = hypotDist - ((hypotDist - avgDist) * 2);  
           break;  
    } 

    set<double>("dist_from_robot", polarDist); 
    set<double>("angle_from_robot", polarAngle); 
} 

void AprilTagCamera::setToLargestRelativePose(){ 
    vex::aivision::object largestObject = camera->largestObject; 
    if (largestObject.exists){ 
        double dist; 
        double angle;  
        robotToTarget(largestObject, angle, dist); 
        set<double>("dist_from_target", dist); 
        set<double>("angle_from_target", angle);  
        set<double>("target_id", largestObject.id);
    }
}

void AprilTagCamera::refreshData(){  

    if (get<bool>("blind")){ 
        return;
    }  

    if (get<bool>("locked_in")){ 
       camera->takeSnapshot( 
           vex::aivision::tagdesc(static_cast<int32_t>(get<int>("target_id"))), 
           4
       );       
       setToAveragedRelativePose(); 

    } else { 
       camera->takeSnapshot( 
           vex::aivision::ALL_TAGS, 
           4
       );  
       setToLargestRelativePose(); 

    }  

    set<bool>("target_found", camera->objects[0].exists);  

    if (get<bool>("requesting_lock")){ 
        if (!get<bool>("locked_in")){ 
            if (get<bool>("target_found")){ 
                set<bool>("locked_in", true); 
                set<bool>("requested_lock", false);
            }
        }
    }
}