#include "camera.h" 

//Need to implement each method except "init"

void Camera::init(){ 
    return;
}; 

void Camera::refreshData(){ 
    if (shouldRecord()){ 
        takeSnapshot(); 
        if (camera.objects[0].exists){ 
          aggregateData();
        }
    }
    
} 

double Camera::getDistFromCamera(int objectIndex){ 
    return 0.0; 
}; 

double Camera::getObjectPitch(int objectIndex){ 
    return 0.0;
};

double Camera::getObjectYaw(int objectIndex){ 
    return 0.0;
}; 

void Camera::getTranslationToTarget(int objectIndex, double* xTransLoc, double* yTransLoc){ 
    return;
}

array<double, 2> Camera::robotToTarget(int objectIndex){ 
    array<double,2> res; 
    return res;
} 

void Camera::robotToTarget(int objectIndex, double* xTransLoc, double* yTransLoc){ 
    return;
}
