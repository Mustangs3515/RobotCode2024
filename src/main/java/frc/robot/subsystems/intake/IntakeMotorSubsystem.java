package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeMotorSubsystem extends SubsystemBase {
    private final CANSparkMax m_motorLeft = new CANSparkMax(Constants.intakeConstants.INTAKE_MOTOR_LEFT_CAN_ID, MotorType.kBrushless);
    private final CANSparkMax m_motorRight = new CANSparkMax(Constants.intakeConstants.INTAKE_MOTOR_RIGHT_CAN_ID, MotorType.kBrushless);

    private double speed = 0;
    private double targetSpeed = 0;
    private double tickSpeed = 0;
    public void spinMotor() {
        targetSpeed = Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED;

        tickSpeed = (targetSpeed - speed)/20;
    }

    @Override
    public void periodic() {
        while (targetSpeed != speed) {
            speed += tickSpeed;
        }
        m_motorLeft.set(speed);
        m_motorRight.set(speed);
    }

    public void stopMotor() {
        targetSpeed = 0;
        tickSpeed = 0;
   }
}
