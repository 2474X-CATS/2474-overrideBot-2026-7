#ifndef __INDEXER_H__
#define __INDEXER_H__

#include "../architecture/subsystem.h"


class Indexer : public Subsystem
{
public:
   using Subsystem::get;
   
   static Indexer* globalRef; 

   Indexer() : Subsystem(
                   "indexer",
                   {(EntrySet){"isOn", EntryType::BOOL}}),  
                   indexerMotor(vex::motor(vex::PORT19))
                   { 
                     globalRef = this;
                   }
   void init() override;
   void periodic() override;
   void updateTelemetry() override; 
   void stop() override; 

   void spinOver(); 
   void spinUnder();

protected:
   using Subsystem::set; 
private:   
   static double ABSOLUTE_INDEXER_SPEED; 

   vex::motor indexerMotor; 

   bool shouldSpinOver(); 
   bool shouldSpinUnder(); 

};

#endif