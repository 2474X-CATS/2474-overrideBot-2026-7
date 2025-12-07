#ifndef __INDEXER_H__
#define __INDEXER_H__

#include "../architecture/subsystem.h"
#include "vex.h"

typedef enum { 
   HIGH, 
   MID,  
   STORAGE,
   NONE
} Feed; 

class Indexer : public Subsystem
{
public:
   using Subsystem::get;
   using Subsystem::getFromInputs;
   
   static Indexer* globalRef; 

   Indexer() : Subsystem(
                   "indexer",
                   { 
                     (EntrySet){"isOn", EntryType::BOOL} 
                   } 
                  ) { 
                     globalRef = this;
                   } 
               
   void init() override;
   void periodic() override;
   void updateTelemetry() override; 
   void stop() override; 

   
protected:
   using Subsystem::set;   

private: 
   static double ABSOLUTE_INDEXER_SPEED; 

   void spinOver(); 
   void spinUnder();
   
};

#endif