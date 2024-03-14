
package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double degree;

    public TurnCmd(DriveSubsystem driveSubsystem, double degree) {
        this.driveSubsystem = driveSubsystem;
        this.degree = degree;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}