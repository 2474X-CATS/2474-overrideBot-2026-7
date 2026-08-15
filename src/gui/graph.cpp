#include "graph.h" 
#include <sstream>

double DataSupplier::getValue(){ 
    return Telemetry::inst.getValueAt<double>(directory, name);
} 

std::string DataSupplier::getLabel(){ 
    return label;
}


double Graph::colors[7][3] = { 
    {224, 51, 38}, 
    {224, 147, 38}, 
    {218, 242, 2}, 
    {30, 204, 14}, 
    {28, 21, 214}, 
    {167, 0, 176}, 
    {107, 42, 12}
};


void Graph::draw(){  
    drawFrame();
    drawData(); 
    drawLegend(); 
    drawTicks(); 
    drawTitle();
    
}

void Graph::update(){ 
    Frame currentFrame;  
    double currentTimestamp = Brain.Timer.time(); 

    currentFrame.timestamp = currentTimestamp;  
    double currentOutput;
    for (DataSupplier supplier : outputSupplier){  
        currentOutput = supplier.getValue();
        currentFrame.yValues.push_back(currentOutput); 
        if (currentOutput > maximumValue){
            maximumValue = currentOutput;
        } 
        if (currentOutput < minimumValue){ 
            minimumValue = currentOutput;
        }
    }
    frames.push_back(currentFrame);  

    if (currentTimestamp - frames.at(0).timestamp >= displayTimeRange - 20){ 
      frames.erase(frames.begin()); 
      minimumTimestamp  = frames.at(0).timestamp;
    }
    
} 

void Graph::mousePressed(int mx, int my){ 
    return;
} 

void Graph::mouseReleased(){ 
    return;
} 

void Graph::drawFrame(){  
    drawRectangle(x,y,width,height, globalColor.rgb(220,220,220));  
    Brain.Screen.setPenWidth(2);
    drawLine(x, y, x, y + height, globalColor.rgb(25,25,25)); 
    drawLine(x, y + height, x + width, y + height, globalColor.rgb(25,25,25));  
    drawLine(x, y, x + width, y, globalColor.white);
    drawLine(x + width, y + height, x + width, y, globalColor.white); 
    Brain.Screen.setPenWidth(1); 
}   

void Graph::drawData(){ 
    int currentSize = frames.size(); 
    Brain.Screen.setPenWidth(2);
    if (frames.size() >= 2){  
     int vectorSize = frames.at(0).yValues.size();  
     for (int i = 1; i < currentSize; i++){ 
        double pixX = getLocalizedX(frames.at(i).timestamp); 
        double prevPixX = getLocalizedX(frames.at(i-1).timestamp); 
        for (int j = 0; j < vectorSize; j++){ 
            drawLine(prevPixX, getLocalizedY(frames.at(i-1).yValues.at(j)), pixX, getLocalizedY(frames.at(i).yValues.at(j)), globalColor.rgb(colors[j][0], colors[j][1], colors[j][2]));
        }
     }  
    }  
    Brain.Screen.setPenWidth(1);
} 

void Graph::drawLegend(){  
    Brain.Screen.setPenWidth(2);
    drawLine(x + width, 0, x + width, 240, globalColor.black);   
    renderText("Legend", x + width + 15, y-5, globalColor.black, globalColor.white, vex::fontType::mono15);
    drawLine(x + width, 30, x + width + 80, 30, globalColor.black); 
    
    for (int i = 0; i < outputSupplier.size(); i ++){ 
       drawRectangle(x + width + 10, y+15 + (i * 30), 10, 10, globalColor.rgb(colors[i][0], colors[i][1], colors[i][2]));
       renderText("=", x + width + 30, y + 25 + (i * 30), globalColor.black, globalColor.white, vex::fontType::mono15);
       renderText(outputSupplier.at(i).getLabel(), x + width + 42, y + 25 + (i * 30), globalColor.black, globalColor.white, vex::fontType::mono15);
    } 

    Brain.Screen.setPenWidth(1);
} 

void Graph::drawTicks(){ 
    renderText("Time(sec)", x + (width / 2) - 40, y + height + 30, globalColor.black, globalColor.white, vex::fontType::mono15);  
    
    int closestSecond = (minimumTimestamp + displayTimeRange) / 1000; 
    stringstream lastTick; 
    lastTick << closestSecond;  

    renderText(lastTick.str(), x + width - (lastTick.str().size() * 4), y + height + 17, globalColor.black, globalColor.white, vex::fontType::mono15);

    double interval = displayTimeRange / 5.0;  
    Brain.Screen.setPenWidth(2); 

    for (int i = 0; i <= 5; i++){  
        stringstream currentTick; 
        currentTick << closestSecond - (5 - i); 
        renderText(currentTick.str(), x + ((width/5) * i) - (currentTick.str().size() * 4), y + height + 17, globalColor.black, globalColor.white, vex::fontType::mono15);
        drawLine(x + ((width/5) * i), y + height - 5, x + ((width/5) * i), y + height + 5, globalColor.black); 
    }    

    double range = maximumValue - minimumValue;

    for (int i = 0; i < 5; i ++){   
        int val = (minimumValue + (range / 4.0 * i)) / 100;
        stringstream currentVal; 
        currentVal << val * 100; 
        drawLine(x - 20, y + height - ((i / 4.0) * height), x, y + height - ((i / 4.0) * height), globalColor.black);
        renderText(currentVal.str(), x - 10 - (currentVal.str().size() * 5), y + height - ((i / 4.0) * height) - 3, globalColor.black, globalColor.white, vex::fontType::mono12);
    }

    Brain.Screen.setPenWidth(1);
} 

void Graph::drawTitle(){   
    renderText(header, x + (width/2.0) - (charsInTitle * 5), 17, globalColor.black, globalColor.white, vex::fontType::mono20);
}

double Graph::getLocalizedY(double rawY){ 
    return y + height - (((rawY - minimumValue) / (maximumValue - minimumValue)) * (height * 1.0));
} 

double Graph::getLocalizedX(double timestamp){ 
    return x + ((timestamp - minimumTimestamp) / displayTimeRange) * width;
}