#ifndef __SLEW_H__ 
#define __SLEW_H__  

class SRLimiter { 
    
    private: 
      double previousOutput = 0;  
      int lastTimestamp = 0;  
      double maxJ; 

    public: 
      SRLimiter(double maxJolt): 
      maxJ(maxJolt){};

      void setTimestamp(double timestamp);   
      void setPreviousOutput(double output); 

      void limit(double& output); 
}; 


#endif