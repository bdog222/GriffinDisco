package frc.robot.Commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class OuttakeStart extends Command 
{
    private final OuttakeSubsystem outtakeSubsystem;

    public OuttakeStart(OuttakeSubsystem outtakeSubsystem)
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
        outtakeSubsystem.Spin(0.6, 0.6);

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
