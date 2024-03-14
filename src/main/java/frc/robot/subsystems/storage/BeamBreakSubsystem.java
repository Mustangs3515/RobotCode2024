package frc.robot.subsystems.storage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreakSubsystem extends SubsystemBase { 
    private final static DigitalInput m_beamBreak = new DigitalInput(Constants.storageConstants.BEAM_BREAK_RECEIVER_DIO);

    private boolean isBroken;
    public boolean isBeamBroken(){
        return isBroken;
    }

    @Override
    public void periodic(){
        this.isBroken = !m_beamBreak.get();
        SmartDashboard.putBoolean("Beam break", isBroken);
    }
}
