#include "aprilVision.h" 
#include "../../utilities/functools.h" 

void TagCamera::initializeCamera(int32_t smartPort){ 
   camera = vex::aivision( 
    smartPort,
    vex::aivision::ALL_TAGS
   );
} 

void TagCamera::takeSnapshot(){   
   if (get<bool>("aligning")){ 
      camera.takeSnapshot(  
        static_cast<uint32_t>(get<int>("target_id")),
        vex::aivision::objectType::tagObject,
        count = 1;
      );
   } else if (get<bool>("localizing")){ 
      camera.takeSnapshot(  
        vex::aivision::ALL_TAGS, 
        count = 4;
      );
   }
} 

array<double, 3> TagCamera::getAveragedPosition(){   

   double avg_x_pos = 0; 
   double avg_y_pos = 0;  
   double avg_heading; 

   double sum_sin = 0; 
   double sum_cos = 0;   

   int entries = 0; 
   
   double current_x = 0;
   double current_y = 0; 
   double current_theta = 0;

   for (int i = 0; i < camera.objectCount; i ++){ 
      if (camera.objects[i].exists){  
         localizedPosition(i, &current_x, &current_y, &current_theta); 
         avg_x_pos += current_x; 
         avg_y_pos += current_y; 
         sum_cos += cos(toRadians(current_theta)); 
         sum_sin += sin(toRadians(current_theta));
         entries ++;
      }
   } 

   avg_heading = toDegrees(atan2(sum_sin, sum_cos) + M_PI);
   avg_x_pos = avg_x_pos / entries; 
   avg_y_pos = avg_y_pos / entries; 

   return {avg_x_pos, avg_y_pos, avg_heading};
}

bool TagCamera::shouldRecord(){ 
   return get<bool>("aligning") || get<bool>("localizing"); 
} 

void TagCamera::aggregateData(){ 
  if (get<bool>("localizing")){ 
    array<double, 3> robotPos = getAveragedPosition();  
    set<double>("l_estimated_x",robotPos[0]); 
    set<double>("l_estimated_y", robotPos[1]); 
    set<double>("l_estimated_angle", robotPos[2]);
  }  
  if (get<bool>("aligning")){  
    set<double>("a_target_dist", getDistFromCamera(0)); 
    set<double>("a_target_theta", getObjectYaw(0));
  }
} 

void TagCamera::localizedPosition(int objectIndex, double* xLoc, double* yLoc, double* headingLoc){ 
   return;
};

