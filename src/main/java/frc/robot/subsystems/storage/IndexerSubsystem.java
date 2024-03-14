package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private final static CANSparkMax m_indexer_motor = new CANSparkMax(Constants.storageConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    private final BeamBreakSubsystem m_beamBreakSubsystem;

    public IndexerSubsystem(BeamBreakSubsystem beam_break){
        this.m_beamBreakSubsystem = beam_break;
    }

    public void spinMotor(double speed){
        if (this.m_beamBreakSubsystem.isBeamBroken() && speed > 0) {
            m_indexer_motor.stopMotor();
            return;
        }
        m_indexer_motor.set(speed);
    }

    public void stopMotor(){
        m_indexer_motor.set(0);
    }
}
