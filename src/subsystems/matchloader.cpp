#include "matchloader.h"

Matchloader *Matchloader::globalRef = nullptr;

void Matchloader::init()
{
   set<bool>("is_on", true);
   stop();
}

void Matchloader::periodic()
{
   if (RobotState::getStateOf("matchloader_out"))
   {
      mlPiston.set(true);
   }
   else
   {
      mlPiston.set(false);
   }
}
//Low goal descroe port E
void Matchloader::updateTelemetry()
{
   return;
}

void Matchloader::stop()
{
   mlPiston.set(false);
}