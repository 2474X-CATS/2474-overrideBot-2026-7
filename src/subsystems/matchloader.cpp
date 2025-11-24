#include "vex.h"
#include "matchloader.h"

Matchloader *Matchloader::globalRef = nullptr;

void Matchloader::init()
{
    matchloaderPiston.close();
    set<bool>("isOn", true);
}

void Matchloader::updateTelemetry()
{
    return;
}

void Matchloader::periodic()
{
    if (RobotState::getStateOf("matchloader_out"))
    {
        deploy();
    }
    else
    {
        retract();
    }
}

void Matchloader::deploy()
{
    matchloaderPiston.open();
}

void Matchloader::retract()
{
    matchloaderPiston.close();
}

void Matchloader::stop()
{
    retract();
}
