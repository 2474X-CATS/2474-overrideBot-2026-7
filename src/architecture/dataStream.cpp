#include "dataStream.h" 


vector<DataStream*> DataStream::streams;

DataStream::DataStream(string dataDesc, vector<EntrySet> entryDecls){ 
   Telemetry::inst.registerSubtable(dataDesc, entryDecls); 
   streams.push_back(this);
}  

void DataStream::initializeAll(){ 
    for (DataStream* s : streams){ 
        s->init(); 
    }
}

void DataStream::refreshAll(){ 
    for (DataStream* s : streams){ 
        s->refreshData(); 
    }
}

