package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmDown extends Command
{
    private final ArmSubsystem armSubsystem;

    public ArmDown(ArmSubsystem armSubsystem)
    {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("ArmDownSubsystem started");

    }

    @Override
    public void execute()
    {
        armSubsystem.Spin(-0.3, 0.3);
    }

    @Override
    public void end(boolean interrupted)
    {
        armSubsystem.Spin(0, 0);
        System.out.println("ArmDownSubsystem ended");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
