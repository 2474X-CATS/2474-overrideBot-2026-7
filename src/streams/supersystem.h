#ifndef __SUPER_SYSTEM_H__ 
#define __SUPER_SYSTEM_H__ 

#include "../architecture/dataStream.h"

class SuperSystem : public DataStream { 
    
    public:  
       SuperSystem() :  
       DataStream(  
        "ss_manager", 
        {
          (EntrySet){"macro_requested", EntryType::BOOL},//Do we want to run auto-score?  
          (EntrySet){"position", EntryType::INT}, //The position state the robot is in [AUTO when a macro]
          (EntrySet){"setpoints_reached", EntryType::BOOL}, //Is the elevator and forearm finished pursuing?
          (EntrySet){"task_completed", EntryType::BOOL}, //Is the macro done running?    
          (EntrySet){"transition_delay", EntryType::DOUBLE}, 
          (EntrySet){"transition_stamp", EntryType::DOUBLE},  
          (EntrySet){"override", EntryType::BOOL}
        }
       ){}; 
       
       void refreshData() override; 

       void init() override;   

    private:   

       static const double MINIMUM_SCORING_CLEAREANCE; 
       double clearance = 0; 
       void setPosition(int pos);
       
};

#endif 