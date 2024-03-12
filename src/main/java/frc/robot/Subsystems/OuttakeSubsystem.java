package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Commands.OuttakeCommand;
import frc.robot.Constants.OuttakeConstants;

public class OuttakeSubsystem extends SubsystemBase 
{
 CANSparkMax Leftsim = new CANSparkMax (OuttakeConstants.LeftsimCanId, MotorType.kBrushed);
 CANSparkMax Rightsim = new CANSparkMax (OuttakeConstants.RightsimCanId, MotorType.kBrushed);

 double SliderValue;

 public OuttakeSubsystem()
 {
    Leftsim.setInverted(false);
    Rightsim.setInverted(false);
 }

 public void Spin(double leftSpeed, double rightSpeed)
 {
    Leftsim.set(leftSpeed);
    Rightsim.set(rightSpeed);
 }  

 public void outtakeSpeed()
 {
   SliderValue = RobotContainer.joystickR.getRawAxis(3);

   if(SliderValue == 1.00)
   {
      OuttakeCommand.speed = 0.1;
   }
   else if(SliderValue >= 0.8 && SliderValue <= 0.9)
   {
      OuttakeCommand.speed = 0.2;
   }
   else if(SliderValue >= 0.7 && SliderValue <= 0.8)
   {
      OuttakeCommand.speed = 0.3;
   }
   else if(SliderValue >= 0.6 && SliderValue <= 0.7)
   {
      OuttakeCommand.speed = 0.4;
   }
   else if(SliderValue >= 0.5 && SliderValue <= 0.6)
   {
      OuttakeCommand.speed = 0.5;
   }
   else if(SliderValue >= 0.4 && SliderValue <= 0.5)
   {
      OuttakeCommand.speed = 0.6;
   }
   else if(SliderValue >= 0.3 && SliderValue <= 0.4)
   {
      OuttakeCommand.speed = 0.7;
   }
   else if(SliderValue >= 0.2 && SliderValue <= 0.3)
   {
      OuttakeCommand.speed = 0.8;
   }
   else if(SliderValue >= 0.1 && SliderValue <= 0.2)
   {
      OuttakeCommand.speed = 0.9;
   }
   else if(SliderValue == -1.00)
   {
      OuttakeCommand.speed = 1.0;
   }
 }

 @Override
 public void periodic() 
 {
   outtakeSpeed();
 }
}
