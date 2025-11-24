#include "intake.h"
#include "vex.h"

Intake *Intake::globalRef = nullptr;

double Intake::ABSOLUTE_INTAKE_SPEED = 200;

void Intake::init()
{
    set<bool>("isOn", true);
};

void Intake::periodic()
{
    if (shouldIntake())
    {
        intake();
    }
    else if (shouldOuttake())
    {
        outtake();
    }
    else
    {
        stop();
    }
};

void Intake::updateTelemetry()
{
    Telemetry::inst.placeValueAt<double>(intakeMotor.temperature(), "Motor_Temps", "IntakeMotor");
}

bool Intake::shouldIntake()
{
    return RobotState::getStateOf("scoring_high") || RobotState::getStateOf("scoring_mid") || RobotState::getStateOf("intaking_to_hopper");
}

bool Intake::shouldOuttake()
{
    return RobotState::getStateOf("scoring_low");
}

void Intake::intake()
{
    intakeMotor.setVelocity(-ABSOLUTE_INTAKE_SPEED, vex::velocityUnits::rpm);
    intakeMotor.spin(vex::directionType::fwd);
}

void Intake::outtake()
{
    intakeMotor.setVelocity(ABSOLUTE_INTAKE_SPEED * 0.5, vex::velocityUnits::rpm);
    intakeMotor.spin(vex::directionType::fwd);
}

void Intake::stop()
{
    intakeMotor.setVelocity(0, vex::percentUnits::pct);
    intakeMotor.spin(vex::directionType::fwd);
}
