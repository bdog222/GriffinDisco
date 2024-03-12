package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ActuatorLeftSubsystem;

public class ActuatorLeftDownCommandFix extends Command 
{
    private ActuatorLeftSubsystem actuatorLeftSubsystem;

    public ActuatorLeftDownCommandFix(ActuatorLeftSubsystem actuatorLeftSubsystem)
    {
        this.actuatorLeftSubsystem = actuatorLeftSubsystem;
        addRequirements(actuatorLeftSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("actuatorLeftDownSubsystem started");
    }

    @Override
    public void execute()
    {
        actuatorLeftSubsystem.Spin(0.2);
    }

    @Override
    public void end(boolean interrupted)
    {
        actuatorLeftSubsystem.Spin(0);
        System.out.println("actuatorLeftDownSubsystem");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}