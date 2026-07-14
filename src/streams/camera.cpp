#include "camera.h" 

//Need to implement each method except "init"

double Camera::CAMERA_PIXEL_HEIGHT = 240; 
double Camera::CAMERA_PIXEL_WIDTH = 320; 

double Camera::YAW_FOV = 74; 
double Camera::PITCH_FOV = 63;

double Camera::getFocalHeight(){ 
    return CAMERA_PIXEL_HEIGHT / (2 * tan(toRadians(PITCH_FOV) / 2));
}

double Camera::getFocalWidth(){ 
    return CAMERA_PIXEL_WIDTH / (2 * tan(toRadians(YAW_FOV) / 2));
}

double Camera::getDistFromCamera(vex::aivision::object obj){ 
    return (getFocalHeight() * getObjectHeight()) / obj.height;
}; 

double Camera::getObjectPitch(vex::aivision::object obj){  
    double centerPixelY = obj.centerY; 
    double pitchPercent = (centerPixelY - (CAMERA_PIXEL_HEIGHT/2)) / (CAMERA_PIXEL_HEIGHT/2);  
    return pitchPercent * PITCH_FOV;
};

double Camera::getObjectYaw(vex::aivision::object obj){  
    double centerPixelX = obj.centerX; 
    double yawPercent = (centerPixelX - (CAMERA_PIXEL_WIDTH/2)) / (CAMERA_PIXEL_WIDTH/2);  
    return yawPercent * YAW_FOV;
}; 


void Camera::robotToTarget(vex::aivision::object obj, double& angleTransLoc, double& distTransLoc){  
    double objectDist = getDistFromCamera(obj);
    double yawToTarget = getObjectYaw(obj);  

    yawToTarget += orientation.facingAngle;   

    double cameraTrans_x = objectDist * cos(toRadians(yawToTarget)); 
    double cameraTrans_y = objectDist * sin(toRadians(yawToTarget));

    double xTrans = cameraTrans_x + orientation.relativeX; 
    double yTrans = cameraTrans_y + orientation.relativeY;  

    distTransLoc = hypot(xTrans, yTrans); 
    angleTransLoc = toDegrees(atan2(yTrans, xTrans));
}
