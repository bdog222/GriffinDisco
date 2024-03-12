package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase 
{
    CANSparkMax ArmLeftNeo = new CANSparkMax(ArmConstants.LeftNeoCanId, MotorType.kBrushless);
    CANSparkMax ArmRightNeo = new CANSparkMax(ArmConstants.RightNeoCanId, MotorType.kBrushless);
    RelativeEncoder armLefEncoder = ArmLeftNeo.getEncoder();
    RelativeEncoder armRightEncoder = ArmRightNeo.getEncoder();

    public ArmSubsystem()
    {

    }

    public void Spin(double LeftSpeed, double RightSpeed)
    {
        ArmLeftNeo.set(LeftSpeed);
        ArmRightNeo.set(RightSpeed);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("LeftArmPosition", armLefEncoder.getPosition());
        SmartDashboard.putNumber("LeftArmPosition", armLefEncoder.getPosition());
        System.out.println("LeftArmPosition" + armLefEncoder.getPosition());
        System.out.println("RightArmPosition" + armRightEncoder.getPosition());
    }
}