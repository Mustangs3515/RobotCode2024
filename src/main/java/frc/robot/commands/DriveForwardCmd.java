
package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForwardCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final double distance;

    public DriveForwardCmd(DriveSubsystem driveSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.distance = driveSubsystem.getEncoderFeetAverage() + distance;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        driveSubsystem.resetEncoders();
        System.out.println("DriveForwardCmd started!");
    }

    @Override
    public void execute() {
        if(distance>0) {
            driveSubsystem.setMotors(DriveConstants.kAutoDriveForwardSpeed, 0);
        }
        else{
            driveSubsystem.setMotors(-DriveConstants.kAutoDriveForwardSpeed, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setMotors(0, 0);
        System.out.println("DriveForwardCmd ended!");
    }

    @Override
    public boolean isFinished() {
        if(distance > 0)
        {
            if (driveSubsystem.getEncoderFeetAverage() > distance)
            {
                System.out.println("Positive & DriveForwardCmd Ended!");
                return true;
            }
            else{
                System.out.println("Positive & DriveForwardCmd Is Running!");
                return false;
            }
        }
        else{
            if (driveSubsystem.getEncoderFeetAverage() < distance)
            {
                System.out.println("Negative & DriveForwardCmd Ended!");
                return true;
            }
            else{
                System.out.println("Negative & DriveForwardCmd is Running!");
                return false;
            }
        }
}

}