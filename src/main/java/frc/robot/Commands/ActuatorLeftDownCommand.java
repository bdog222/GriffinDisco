package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ActuatorLeftSubsystem;

public class ActuatorLeftDownCommand extends Command 
{
    private final ActuatorLeftSubsystem actuatorLeftSubsystem;

    public ActuatorLeftDownCommand(ActuatorLeftSubsystem actuatorLeftSubsystem)
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
        if(actuatorLeftSubsystem.ActuatorLefEncoder.getPosition() > -125)
        {
             actuatorLeftSubsystem.Spin(-0.6);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        actuatorLeftSubsystem.Spin(0);
         System.out.println("actuatorLeftDownSubsystem ended");

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}