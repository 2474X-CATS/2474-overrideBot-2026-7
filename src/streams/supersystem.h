#ifndef __SUPER_SYSTEM_H__ 
#define __SUPER_SYSTEM_H__ 

#include "../architecture/dataStream.h"

class SuperSystem : public DataStream { 
    
    public:
       SuperSystem();

       void refreshData() override; 

       void init() override;   

    private:   

       void setPosition(int pos);
       
};

#endif 