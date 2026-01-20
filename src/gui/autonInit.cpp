#include "autonInit.h" 

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
    drawEllipse(x,y,width, globalColor.red);
} 

void LoadingScreen::update(){ 
    return; 
} 

void LoadingScreen::mousePressed(int mx, int my){ 
    return;
} 

void LoadingScreen::mouseReleased(){ 
    return;
}


//------------------------------------------------------------------------------------ 

