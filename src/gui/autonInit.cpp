#include "autonInit.h" 
#include "math.h"

void ColorPicker::draw(){  
    uint32_t currentColor = isBlue ? globalColor.blue : globalColor.red; 
    int32_t backdrop = pressed ? globalColor.rgb(150,150,150) : globalColor.rgb(200,200,200);  


    drawRectangle(x,y,width, height, backdrop); 
    drawRectangle(x + (width / 5) * 3, y + (height / 10), width / 3, ((height / 10) * 9), currentColor); 
    renderText("Color", x + (width / 20), y + (height/3) * 2, globalColor.black, backdrop, vex::fontType::mono20); 
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
    int32_t backdrop = pressed ? globalColor.rgb(150,150,150) : globalColor.rgb(200,200,200); 

    drawRectangle(x,y,width,height, backdrop);   
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
    renderText("Side", x + (width / 20), y + (height/3) * 2, globalColor.black, backdrop, vex::fontType::mono20); 
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
  int32_t backdrop = pressed ? globalColor.rgb(150,150,150) : globalColor.rgb(200,200,200);
  drawRectangle(x, y, width, height, backdrop);  
  renderText("Continue", x+(width/4)+10, y+((height/3)*2), Sprite::globalColor.black, backdrop, vex::fontType::mono20);
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


RoutineCatalog::RoutineCatalog(int x, int y, std::vector<AutonOption> autosShell) :  
Sprite(x, y, 200, (autosShell.size()) * 30 + 35)
{ 
  autos.push_back( 
    (AutonOption){"", "", -1, false, false}
  );
  for (AutonOption option : autosShell){ 
    autos.push_back(option); 
  } 

  chosenIndex = 0;
}; 
 
void RoutineCatalog::draw(){   

  drawRectangle(x, y, width, 34, globalColor.white); 
  renderText("Choose your routine", x+5, y + 22, globalColor.black, globalColor.white, vex::fontType::mono20); 

  int referenceY = y + 35;   
  for (int i = 1; i < autos.size() ; i++){  
    renderCell(autos.at(i), referenceY);
    referenceY += 30;
  }   
}  

void RoutineCatalog::renderCell(AutonOption option, int yPos){  
  int32_t leftColor = option.hasLeftSide ? globalColor.green : globalColor.red; 
  int32_t rightColor = option.hasRightSide ? globalColor.green : globalColor.red;  
   
  int32_t textColor = option.index == chosenIndex ? globalColor.white : globalColor.black; 
  int32_t backdropColor = option.index == chosenIndex ? globalColor.rgb(48, 48, 48) : globalColor.rgb(151, 151, 151);

  drawRectangle(x, yPos, width, 30, backdropColor);
  renderText(option.name, x+10, yPos + 20, textColor, backdropColor, vex::fontType::mono15); 

  drawRectangle(x + (width / 10 * 7), yPos + 5, 20, 20, leftColor); 
  drawRectangle(x + (width / 10 * 7) + 20, yPos + 5, 20, 20, rightColor);
}

void RoutineCatalog::update(){  
  return;
} 

void RoutineCatalog::mousePressed(int mx, int my){   
  if (my > y + 35){ 
    chosenIndex = ((my - (y + 35)) / 30);
  }
} 

void RoutineCatalog::mouseReleased(){ 
  return;
}  

int RoutineCatalog::getAutonIndex(){ 
  return chosenIndex; 
} 

AutonOption RoutineCatalog::getCurrentAuto(){ 
  return autos.at(chosenIndex+1);
} 

//---------------------------------------------------------

AutonDisplay::AutonDisplay(int x, int y, RoutineCatalog* catalog, SidePicker* sidePicker) : 
Sprite(x, y, 200, 100),
catalog(catalog), 
sidePicker(sidePicker)
{ 
  return;
} 

void AutonDisplay::draw(){    

    int32_t backdrop_color;
    AutonOption currentAuto = catalog->getCurrentAuto(); 

    if ( (currentAuto.hasLeftSide && sidePicker->getIsLeft()) || (currentAuto.hasRightSide && (!sidePicker->getIsLeft())) ){  
       backdrop_color = globalColor.rgb(72, 227, 0);
    } else { 
       backdrop_color = globalColor.rgb(229, 4, 0);
    } 

    drawRectangle(x, y, width, height, backdrop_color);

    renderText(currentAuto.name, x+5, y+20, globalColor.black, backdrop_color, vex::fontType::mono20);  

    std::string referenceString = currentAuto.description;  
    int referenceY = y + 40;   
    string currentMessage = ""; 
    
   while (true){  
      std::string::size_type sz = referenceString.find(" ");  
      if (sz == std::string::npos){ 
           renderText(currentMessage + referenceString, x+5, referenceY, globalColor.black, backdrop_color, vex::fontType::mono12);  
           break;
      }    
      if (currentMessage.size() + sz > 31){ 
          renderText(currentMessage, x+5, referenceY, globalColor.black, backdrop_color, vex::fontType::mono12);   
          
          currentMessage = referenceString.substr(0, sz) + " ";   
          referenceString = referenceString.substr(sz+1); 

          referenceY += 12;
      } else { 
          currentMessage += referenceString.substr(0, sz) + " "; 
          referenceString = referenceString.substr(sz+1);
      }
   }  

} 

void AutonDisplay::update(){ 
    return;
} 

void AutonDisplay::mousePressed(int mx, int my){ 
    return;
} 

void AutonDisplay::mouseReleased(){ 
    return;
}



