package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class OuttakeStop extends Command
{
    private final OuttakeSubsystem outtakeSubsystem;

    public OuttakeStop(OuttakeSubsystem outtakeSubsystem)
    {
        this.outtakeSubsystem = outtakeSubsystem;
        addRequirements(outtakeSubsystem);
    }

    @Override
    public void initialize()
    {
        System.out.println("outtakeSubsystem started");

    }

    @Override
    public void execute()
    {
        outtakeSubsystem.Spin(0, 0);
    }

    @Override
    public void end(boolean interrupted)
    {
        outtakeSubsystem.Spin(0, 0);
         System.out.println("outtakeSubsystem ended");

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
