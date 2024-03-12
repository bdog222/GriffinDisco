package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmStop extends Command
{
    private final ArmSubsystem armSubsystem;

    public ArmStop(ArmSubsystem armSubsystem)
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
        armSubsystem.Spin(0, 0);
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
