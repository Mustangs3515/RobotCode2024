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
    public void spinMotor(double topSpeed, double bottomSpeed) {
        this.m_motorTop.set(topSpeed);
        this.m_motorBottom.set(bottomSpeed);
    }
    
    public void stopMotor() {
        this.m_motorTop.set(0);
        this.m_motorBottom.set(0);
   }
}