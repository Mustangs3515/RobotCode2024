package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ElevatorSubsystem extends PIDSubsystem {
    // private final CANSparkMax m_elevatorMotor = new CANSparkMax(Constants.elevatorConstants.ELEVATOR_MOTOR_CAN_ID,
    //         MotorType.kBrushless);
    public ElevatorSubsystem(PIDController controller) {
         super(controller);
     }


    @Override
    protected double getMeasurement() {
        // return m_elevatorMotor.getEncoder().getPosition();
        return 4; // this is so that the code doesn't break. Not originally here
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        
        // m_elevatorMotor.setVoltage(output);
    }

}
