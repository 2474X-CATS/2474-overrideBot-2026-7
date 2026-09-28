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
    
    //drawEllipse(normalizeX(TILE_SIZE_MM), normalizeY(1200), 3, Sprite::globalColor.black); 
    
    std::stringstream output; 
    output << trajectory.calculateLength(); 
    renderText(output.str(), 300, 120, Sprite::globalColor.white, Sprite::globalColor.black, vex::fontType::mono12); 

    for (int i = 0; i < points.size(); i ++){  
        drawEllipse(normalizeX(points[i].x), normalizeY(points[i].y), 2, Sprite::globalColor.black);   
        
        /*
        std::stringstream num; 
        num << "(" << points[i].x << ", " << points[i].y << ")"; 
        renderText(num.str(), 25, 20+i*10, Sprite::globalColor.white, Sprite::globalColor.black, vex::fontType::mono12); 
        */ 
       
    }    
      

    /*
    drawEllipse(normalizeX(trajectory.generateX(1)), normalizeY(trajectory.generateY(1)), 3, Sprite::globalColor.black);
    */
    /*
    double len = points.size();  
    std::stringstream num; 
    num << "This trajectory is " << trajectory.calculateLength() << " mm long and has generated " << len << " points";
    renderText(num.str(), 75, 80, Sprite::globalColor.white, Sprite::globalColor.black, vex::fontType::mono12); 
    */
}