package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorsConstants;

public class ActuatorLeftSubsystem extends SubsystemBase 
{
    public  CANSparkMax ActuatorLeftNeo = new CANSparkMax(ActuatorsConstants.LeftNeoCanId, MotorType.kBrushless);
    public  RelativeEncoder ActuatorLefEncoder = ActuatorLeftNeo.getEncoder();

    public ActuatorLeftSubsystem()
    {
       ActuatorLeftNeo.setInverted(false);
    }
   
    public void Spin(double leftSpeed)
    {
      ActuatorLeftNeo.set(leftSpeed);
    }  

    @Override
    public void periodic()
    {
      SmartDashboard.putNumber("ActuatorLeftPosition", ActuatorLefEncoder.getPosition());
    }
}
