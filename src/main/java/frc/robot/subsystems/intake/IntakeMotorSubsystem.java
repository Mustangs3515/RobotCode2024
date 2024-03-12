package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {
    private final CANSparkMax m_motorTop = new CANSparkMax(Constants.intakeConstants.TOP_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax m_motorBottom = new CANSparkMax(Constants.intakeConstants.BOTTOM_INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

    public IntakeMotorSubsystem() {
        m_motorTop.setInverted(true);
        m_motorBottom.setInverted(true);
    }
    public void spinMotor() {
        this.m_motorTop.set(Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED);
        this.m_motorBottom.set(Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED);
    }

    // reverses the intake
    public void reverseSpinMotor()
    {
        this.m_motorTop.set(-Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED);
        this.m_motorBottom.set(-Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED);
    }
    
    public void shakeItUp()
    {
        this.m_motorTop.set(Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED);
        this.m_motorBottom.set(-Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED);
    }

    public void stopMotor() {
        this.m_motorTop.set(0);
        this.m_motorBottom.set(0);
   }
}