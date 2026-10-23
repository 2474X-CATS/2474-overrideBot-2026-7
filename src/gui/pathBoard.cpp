#include "pathBoard.h"  
#include <sstream>

int PathBoard::normalizeX(double nx){
   return x + (nx / (TILE_SIZE_MM * 6) * width);
} 

int PathBoard::normalizeY(double ny){ 
   return y + height - (ny * 1.0 / (TILE_SIZE_MM * 6) * height);
}

void PathBoard::draw(){   
   
    drawRectangle(x,y,width,height, Sprite::globalColor.rgb(225,225,225));  
    
    std::stringstream output;
    output << trajectory.calculateLength(); 
    renderText(output.str(), 300, 120, Sprite::globalColor.white, Sprite::globalColor.black, vex::fontType::mono12); 
    
    for (int i = 1; i < 6; i ++){ 
        drawLine(x, y+i/6.0*height, x+width, y+i/6.0*height, Sprite::globalColor.black); 
        drawLine(x+i/6.0*width, y, x+i/6.0*width, y+height, Sprite::globalColor.black);
    }
    for (int i = 0; i < points.size(); i ++){  
        drawEllipse(normalizeX(points[i].pt.x), normalizeY(points[i].pt.y), 2, Sprite::globalColor.rgb(255 - points[i].velocity * 255, 0, 0));   
    }      

}