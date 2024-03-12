package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ActuatorRightSubsystem;

public class ActuatorRightDownCommandFix extends Command 
{
    private final ActuatorRightSubsystem actuatorRightSubsystem;

    public ActuatorRightDownCommandFix(ActuatorRightSubsystem actuatorRightSubsystem)
    {
        this.actuatorRightSubsystem = actuatorRightSubsystem;
        addRequirements(actuatorRightSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("actuatorRightDownSubsystem");
    }

    @Override
    public void execute()
    {
        actuatorRightSubsystem.Spin(0.2);
    }

    @Override
    public void end(boolean interrupted)
    {
        actuatorRightSubsystem.Spin(0);
        System.out.println("actuatorRightDownSubsystem");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}