package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ActuatorRightSubsystem;

public class ActuatorRightUpCommand extends Command 
{
    private final ActuatorRightSubsystem actuatorRightSubsystem;

    public ActuatorRightUpCommand(ActuatorRightSubsystem actuatorRightSubsystem)
    {
        this.actuatorRightSubsystem = actuatorRightSubsystem;
        addRequirements(actuatorRightSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("actuatorRightUpSubsystem started");

    }

    @Override
    public void execute()
    {
        if(actuatorRightSubsystem.ActuatorRighEncoder.getPosition() < 0) 
        {
            actuatorRightSubsystem.Spin(0.6);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        actuatorRightSubsystem.Spin(0);
         System.out.println("actuatorRightUpSubsystem ended");

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}