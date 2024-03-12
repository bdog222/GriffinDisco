package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ActuatorRightSubsystem;

public class ActuatorRightDownCommand extends Command 
{
    private final ActuatorRightSubsystem actuatorRightSubsystem;

    public ActuatorRightDownCommand(ActuatorRightSubsystem actuatorRightSubsystem)
    {
        this.actuatorRightSubsystem = actuatorRightSubsystem;
        addRequirements(actuatorRightSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("actuatorRightDownSubsystem started");

    }

    @Override
    public void execute()
    {
        if(actuatorRightSubsystem.ActuatorRighEncoder.getPosition() > -125)
        {
             actuatorRightSubsystem.Spin(-0.6);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        actuatorRightSubsystem.Spin(0);
         System.out.println("actuatorRightDownSubsystem ended");

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}