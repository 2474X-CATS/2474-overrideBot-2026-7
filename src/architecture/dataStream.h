#ifndef __DATA_STREAM_H__
#define __DATA_STREAM_H__ 

#include "vex.h" //Devices
#include "telemetry.h"  //Making subtable + accessing data
#include "robotState.h" //Looking at current robot processes
#include <string> 

class DataStream {  
    
    private:  

      static vector<DataStream*> streams;  

    public:   

       DataStream(string dataDesc, vector<EntrySet> entryDecls);   

       virtual void refreshData() = 0; //Calls every telemetry frame  
       
       virtual void init() = 0; //Sets up sensors for data-collection

       static void refreshAll(); 

       static void initializeAll(); 

};


#endif
