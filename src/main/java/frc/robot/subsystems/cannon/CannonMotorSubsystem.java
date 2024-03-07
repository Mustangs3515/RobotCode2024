package frc.robot.subsystems.cannon;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CannonMotorSubsystem extends SubsystemBase{
    private final CANSparkMax m_rightMotor = new CANSparkMax(Constants.cannonConstants.RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax m_leftMotor = new CANSparkMax(Constants.cannonConstants.LEFT_MOTOR_CAN_ID, MotorType.kBrushless);

    // sets the two cansparkmaxes for the shooter to amp power
    public void setCannonPower(double ampFiringPower){
        m_rightMotor.set(ampFiringPower);
        m_leftMotor.set(ampFiringPower);
    }

    // turns off the cansparkmaxes 
    public void stopCannon(){
        m_rightMotor.set(0);
        m_leftMotor.set(0);
    }

    public double getTopRPM() {
        return m_rightMotor.getEncoder().getVelocity();
    }
    public double getBottomRPM() {
        return m_leftMotor.getEncoder().getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Shooter RPM", getBottomRPM());
    }
}
