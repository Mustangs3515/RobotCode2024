package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class IndexerSubsystem {
    private final CANSparkMax m_indexer_motor = new CANSparkMax(Constants.storageConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    private final BeamBreakSubsystem m_beamBreakSubsystem;

    public IndexerSubsystem(BeamBreakSubsystem beam_break){
        this.m_beamBreakSubsystem = beam_break;
    }

    public void spinMotor(){
        if (this.m_beamBreakSubsystem.isBeamBroken()) {
            this.stopMotor();
            return;
        }
        this.m_indexer_motor.set(Constants.storageConstants.INDEXER_SPIN_SPEED);
    }

    public void stopMotor(){
        this.m_indexer_motor.set(0);
    }
}
