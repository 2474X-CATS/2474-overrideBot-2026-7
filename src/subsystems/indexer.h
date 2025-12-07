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
   
   static Indexer* globalRef; 

   Indexer() : Subsystem(
                   "indexer",
                   { 
                     (EntrySet){"detects_block_high", EntryType::BOOL},  
                     (EntrySet){"detects_block_mid", EntryType::BOOL},
                     (EntrySet){"is_block_blue", EntryType::BOOL}, 
                     (EntrySet){"detects_jam", EntryType::BOOL}
                   } 
                  ), 
                  indexerMotor(vex::motor(vex::PORT19)), 
                  colorSensor(vex::optical(vex::PORT10)), 
                  distanceSensor(vex::distance(vex::PORT7))
                  { 
                     globalRef = this;
                   } 
               
   void init() override;
   void periodic() override;
   void updateTelemetry() override; 
   void stop() override; 

   
protected:
   using Subsystem::set;   

private:   

   vex::motor indexerMotor;  
   vex::optical colorSensor;  
   vex::distance distanceSensor;

   static double ABSOLUTE_INDEXER_SPEED;  

   static double BLUE_HUE; 
   static double RED_HUE;   

   static double DISTANCE_THRESHOLD;

   bool detectsBlock();

   void spinOver(); 
   void spinUnder();  

   bool shouldSpinOver(); 
   bool shouldSpinUnder(); 

   bool sensesValidBlock(); 
   bool sensesInvalidBlock(); 

   double getExpectedVelocity(); 

   
};

#endif