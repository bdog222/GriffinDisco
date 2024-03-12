package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ActuatorLeftSubsystem;

public class ActuatorLeftUpCommand extends Command 
{
    private final ActuatorLeftSubsystem actuatorLeftSubsystem;

    public ActuatorLeftUpCommand(ActuatorLeftSubsystem actuatorLeftSubsystem)
    {
        this.actuatorLeftSubsystem = actuatorLeftSubsystem;
        addRequirements(actuatorLeftSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("actuatorLeftUpSubsystem started");
  
    }

    @Override
    public void execute()
    {
        if(actuatorLeftSubsystem.ActuatorLefEncoder.getPosition() < 0)
        {
            actuatorLeftSubsystem.Spin(0.6);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        actuatorLeftSubsystem.Spin(0);
         System.out.println("actuatorLeftUpSubsystem ended");

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}