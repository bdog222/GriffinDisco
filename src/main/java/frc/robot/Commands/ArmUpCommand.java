package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmUpCommand extends Command 
{
    private final ArmSubsystem armSubsystem;

    public ArmUpCommand(ArmSubsystem armSubsystem)
    {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("ArmUpSubsystem started");

    }

    @Override
    public void execute()
    {
        armSubsystem.Spin(0.3, -0.3);
    }

    @Override
    public void end(boolean interrupted)
    {
        armSubsystem.Spin(0, 0);
        System.out.println("ArmUpSubsystem ended");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
