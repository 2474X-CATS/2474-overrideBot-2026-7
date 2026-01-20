#include "autonInit.h" 
#include "math.h"

void ColorPicker::draw(){  
    uint32_t currentColor = isBlue ? globalColor.blue : globalColor.red;
    drawRectangle(x,y,width, height, globalColor.rgb(200,200,200)); 
    drawRectangle(x + (width / 5) * 3, y + (height / 10), width / 3, ((height / 10) * 9), currentColor); 
    renderText("Color", x + (width / 20), y + (height/3) * 2, globalColor.black, globalColor.rgb(200,200,200), vex::fontType::mono20); 
} 

void ColorPicker::update(){ 
    return; 
} 

void ColorPicker::mousePressed(int mx, int my){ 
    return;
}; 

void ColorPicker::mouseReleased(){  
    isBlue = !isBlue;
}  

bool ColorPicker::getIsBlue(){ 
    return isBlue;
}  

//---------------------------------------------------------------------- 

void SidePicker::draw(){ 
    drawRectangle(x,y,width,height, globalColor.rgb(200,200,200));   
    int centerX = x + (width / 3) * 2;
    drawLine(centerX, y, centerX, y + height, globalColor.black);  
    int tipX;
    if (isLeft){ 
      tipX = centerX - (width / 6);
    } else { 
      tipX = centerX + (width / 6);
    } 
    drawLine(centerX, y, tipX, y + (height / 2), globalColor.black); 
    drawLine(centerX, y + height, tipX, y + (height / 2), globalColor.black);  
    renderText("Side", x + (width / 20), y + (height/3) * 2, globalColor.black, globalColor.rgb(200,200,200),  vex::fontType::mono20); 
} 

void SidePicker::update(){
    return;
} 

void SidePicker::mousePressed(int mx, int my){ 
    return;
} 

void SidePicker::mouseReleased(){ 
    isLeft = !isLeft; 
}  

bool SidePicker::getIsLeft(){ 
    return isLeft; 
} 


//------------------------------------------------------------------------------------  

void ExitBlock::draw(){ 
  drawRectangle(x, y, width, height, Sprite::globalColor.green);  
  renderText("Continue", x+(width/6), y+((height/3)*2), Sprite::globalColor.black, Sprite::globalColor.green, vex::fontType::mono20);
} 

void ExitBlock::update(){ 
  return;
} 

void ExitBlock::mousePressed(int mx, int my){ 
  return;
}

void ExitBlock::mouseReleased(){ 
  Sprite::running = false;
}


//------------------------------------------------------------------------------------  

void LoadingScreen::draw(){   
    if (pressed){ 
        drawEllipse(x+width/2,y+width/2,width, globalColor.rgb(64, 64, 64));
    } else { 
        drawEllipse(x+width/2,y+width/2,width, globalColor.rgb(128, 128, 128));
    }  

    double hypotenuse = width/4*3;  
    double currentAngle = angle; 
    
  
    for (int i = 0; i < 9; i ++){ 
       drawEllipse( 
         static_cast<int>((x+width/2.0) + hypotenuse * cos(currentAngle / 360.0 * M_PI)), 
         static_cast<int>((y+width/2.0) + hypotenuse * sin(currentAngle / 360.0 * M_PI)), 
         20,  
         globalColor.rgb( 
            static_cast<int>(255 / 6.0 * 1.0), 
            static_cast<int>(255 / 6.0 * 1.0), 
            static_cast<int>(255 / 6.0 * 1.0)
         )
       );  
       currentAngle -= 90;
    }

} 

void LoadingScreen::update(){ 
    angle += 10;
} 

void LoadingScreen::mousePressed(int mx, int my){  

    return;
} 

void LoadingScreen::mouseReleased(){ 
    Sprite::running = false;
}


//------------------------------------------------------------------------------------ 

