package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeStop extends Command
{
     private final IntakeSubsystem intakeSubsystem;

    public IntakeStop(IntakeSubsystem intakeSubsystem)
    {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("IntakeOneSubsystem started");

    }

    @Override
    public void execute()
    {
        intakeSubsystem.Spin(0);
    }

    @Override
    public void end(boolean interrupted)
    {
        intakeSubsystem.Spin(0);
         System.out.println("IntakeOneSubsystem ended");

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
