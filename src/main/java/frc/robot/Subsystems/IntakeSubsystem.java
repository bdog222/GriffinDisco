package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase 
{
    CANSparkMax IntakeNeo = new CANSparkMax (IntakeConstants.IntakeNeoCanId, MotorType.kBrushless);

    public IntakeSubsystem()
 {
    IntakeNeo.setInverted(false);
 }

 public void Spin(double IntakeNeoSpeed)
 {
    IntakeNeo.set(IntakeNeoSpeed);
 }
}   
