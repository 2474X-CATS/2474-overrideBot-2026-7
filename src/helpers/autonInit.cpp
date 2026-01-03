#include "autonInit.h" 

void ColorPicker::draw(){  
    uint32_t currentColor = isBlue ? globalColor.blue : globalColor.red;
    drawRectangle(x,y,width, height, globalColor.rgb(200,200,200)); 
    drawRectangle(x + (width / 5) * 3, y + (height / 10), width / 3, ((height / 10) * 9), currentColor); 
    renderText("Color", x + (width / 20), y + (height/3) * 2, globalColor.white); 
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
    renderText("Side", x + (width / 20), y + (height/3) * 2, globalColor.white); 
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

void Field::draw(){  
    uint32_t tileColor; 
    for (int i = 0; i < 6; i ++){ 
        for (int j = 0; j < 6; j ++){ 
            if ((i + j) % 2 == 0){ 
                tileColor = globalColor.rgb(175, 175, 175); 
            } else { 
                tileColor = globalColor.rgb(180, 180, 180); 
            } 
            drawRectangle(x + ((width / 6) * j), y + ((height / 6) * i), width / 6, height / 6, tileColor);
        }
    }
}

void Field::update(){ 
    return; 
} 

void Field::mousePressed(int mx, int my){ 
    return;
} 

void Field::mouseReleased(){ 
    return; 
} 

std::array<int, 2> Field::normalizeCoordinates(int fx, int fy){  
    std::array<int, 2> res;
    res[0] = static_cast<int>((fx * 1.0  / (TILE_SIZE_MM * 6)) * width);  
    res[1] = height - static_cast<int>((fy * 1.0  / (TILE_SIZE_MM * 6)) * height);
    return res;
} 

int Field::getTileSize(){ 
    return width / 6;
}