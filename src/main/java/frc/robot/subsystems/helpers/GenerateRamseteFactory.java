package frc.robot.subsystems.helpers;

import java.io.IOException;
import java.lang.reflect.Array;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.auto.base.AutoPath;

public class GenerateRamseteFactory {

    // creates a Ramsete Command and returns it 
    private static RamseteCommand generateRamseteCommand(Trajectory path){
        return new RamseteCommand(
        path,
        Robot.m_drivetrain::getPose,
        new RamseteController(Constants.PathWeaverConstants.kRamseteB, Constants.PathWeaverConstants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.PathWeaverConstants.ksVolts,
            Constants.PathWeaverConstants.kvVoltSecondsPerMeter,
            Constants.PathWeaverConstants.kaVoltSecondsSquaredPerMeter),
        Constants.PathWeaverConstants.kDriveKinematics,
        Robot.m_drivetrain::getWheelSpeeds,
        new PIDController(Constants.PathWeaverConstants.kPDriveVel, 0, 0),
        new PIDController(Constants.PathWeaverConstants.kPDriveVel, 0, 0),
        Robot.m_drivetrain::driveByVolts,
        Robot.m_drivetrain);

    }

    // uses the ramsete command to return the auto paths
    public static RamseteCommand[] getAutoTrajectories(AutoPath path){
        
        try {
            RamseteCommand p1 = generateRamseteCommand(path.p1());
            RamseteCommand p2 = generateRamseteCommand(path.p2());
            RamseteCommand p3 = generateRamseteCommand(path.p3());
            RamseteCommand[] paths = {p1,p2,p3};

            return paths;
        } catch (IOException e) {
            System.out.println("You goofed homie");
            e.printStackTrace();

        }
        return new RamseteCommand[3];
        
    }
}
