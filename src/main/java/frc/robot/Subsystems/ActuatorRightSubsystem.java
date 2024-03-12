package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ActuatorsConstants;

public class ActuatorRightSubsystem extends SubsystemBase 
{
    public  CANSparkMax ActuatorRightNeo = new CANSparkMax(ActuatorsConstants.RightNeoCanId, MotorType.kBrushless);
    public  RelativeEncoder ActuatorRighEncoder = ActuatorRightNeo.getEncoder();
   

    public ActuatorRightSubsystem()
    {
       ActuatorRightNeo.setInverted(false);
    }
   
    public void Spin(double leftSpeed)
    {
       ActuatorRightNeo.set(leftSpeed);
    }  

    @Override
    public void periodic()
    {
      SmartDashboard.putNumber("ActuatorRightPosition", ActuatorRighEncoder.getPosition());
    }
}
