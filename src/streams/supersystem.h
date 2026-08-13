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
          (EntrySet){"in_autopilot", EntryType::BOOL}, //Is a macro running?
          (EntrySet){"position", EntryType::INT}, //The position state the robot is in [AUTO when a macro]
          (EntrySet){"setpoints_reached", EntryType::BOOL}, //Is the elevator and forearm finished pursuing?
          (EntrySet){"task_completed", EntryType::BOOL}, //Is the macro done running?  
          (EntrySet){"pickup_position", EntryType::INT}, 
          (EntrySet){"can_transition", EntryType::BOOL}, 
          (EntrySet){"distance_backed", EntryType::DOUBLE}
        }
       ){}; 

       void refreshData() override; 

       void init() override;   

    private:  
       static double BACKUP_DISTANCE;
       void resetTaskProgress();


};


#endif 