package frc.robot.subsystems.storage;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
    private final CANSparkMax m_indexer_motor = new CANSparkMax(Constants.storageConstants.INDEXER_MOTOR_ID, MotorType.kBrushless);
    private final DigitalInput m_beamBreak = new DigitalInput(Constants.storageConstants.BEAM_BREAK_RECEIVER_DIO);

    private boolean isBroken;

    public IndexerSubsystem(){
        
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run
        this.isBroken = !m_beamBreak.get();
        SmartDashboard.putBoolean("Beam break", isBroken);
    }

    public boolean isBeamBroken(){
        return isBroken;
    }

    public void spinMotor(){
        if (isBeamBroken()) {
            this.stopMotor();
            return;
        }
        this.m_indexer_motor.set(Constants.storageConstants.INDEXER_SPIN_SPEED);
    }

    public void reverseMotor()
    {
        this.m_indexer_motor.set(-Constants.storageConstants.INDEXER_SPIN_SPEED);
    }

    public void stopMotor(){
        this.m_indexer_motor.set(0);
    }

    public void spinMotorFast(){
        this.m_indexer_motor.set(Constants.storageConstants.INDEXER_SPIN_SPEED_FAST);
    }
}
