package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cannon.CannonMotorSubsystem;
import frc.robot.subsystems.storage.BeamBreakSubsystem;
import frc.robot.subsystems.storage.IndexerSubsystem;

public class IndexerCmd extends Command{
    private final BeamBreakSubsystem m_beamBreakSubsystem = new BeamBreakSubsystem();
    private IndexerSubsystem indexer = new IndexerSubsystem(m_beamBreakSubsystem);
    double speed = 0;

    public IndexerCmd(IndexerSubsystem theIndexer, double theSpeed)
    {
        this.indexer = theIndexer;
        this.speed = theSpeed;
    }

    @Override
    public void execute() {
        indexer.spinMotor(speed);
    }

    @Override
    public void initialize() {
        System.out.println("IndexerCmd started!");
    }

    // stops motor
    @Override
    public void end(boolean interrupted){
        System.out.println("IndexerCmd ended!");
        indexer.stopMotor();
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}
