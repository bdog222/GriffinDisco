package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.OuttakeSubsystem;

public class OuttakeCommand extends Command 
{
    private final OuttakeSubsystem outtakeSubsystem;
    public static double speed;

    public OuttakeCommand(OuttakeSubsystem outtakeSubsystem)
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
        outtakeSubsystem.Spin(speed, speed);

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