package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.helpers.ControlReversalStore;
import frc.robot.Constants.DriveConstants;

import org.photonvision.PhotonCamera;

public class CameraIntakeCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private PhotonCamera camera;
    private ControlReversalStore m_controlReversal;
    private ShuffleboardTab tab = Shuffleboard.getTab("vision speeds");
    private GenericEntry fastentry = tab.add("fastSpeed", .7).getEntry();
    private GenericEntry slowentry = tab.add("slowSpeed", .6).getEntry();
    private PIDController pid = new PIDController(DriveConstants.kPCamSteer, DriveConstants.kICamSteer, DriveConstants.kDCamSteer);
    private double fastSpeed = 0.7;
    private double slowSpeed = 0.6;
    

    public CameraIntakeCmd(DriveSubsystem driveSubsystem, PhotonCamera camera, ControlReversalStore m_controlReversal) {
        this.driveSubsystem = driveSubsystem;
        this.camera = camera;
        this.m_controlReversal = m_controlReversal;

        addRequirements(driveSubsystem);

    }

    @Override
    public void initialize() {
        System.out.println("CameraIntakeCmd started!");
        pid.reset();
        pid.setSetpoint(0);
    }

    @Override
    public void execute() {
         //gets values from photonvision: yaw, pitch...
        var result = this.camera.getLatestResult();
        boolean hasTargets = result.hasTargets();
        //boolean gets passed into smart dashboard
        SmartDashboard.putBoolean("Approaching Note", hasTargets);

        //if robot has no targets, do not turn
        if (!hasTargets) {
            this.driveSubsystem.setMotors(0, 0);
            return;
        }

        //yaw of note
        double yaw = (result.getBestTarget().getYaw());
        // slowentry.getDouble(slowSpeed);
        // fastentry.getDouble(fastSpeed);

        //move robot forward if note detected 
        // if (Math.abs(yaw) > .05) {
        //     this.driveSubsystem.setMotors(0, slowSpeed * Math.signum(yaw));
        //     return;
        // } else if (Math.abs(yaw) > .25) {
        //     this.driveSubsystem.setMotors(0, fastSpeed * Math.signum(yaw));
        //     return;
        // }
        this.driveSubsystem.setMotors(0, pid.calculate(yaw));

        //start and end revving intake up
        double speed = 0;
        if (m_controlReversal.getForwardSide() == "intake"){
            speed = 0.7;
        } else {
            speed = -0.7;
        }
        this.driveSubsystem.setMotors(speed, 0);
        return;
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.setMotors(0, 0);
        SmartDashboard.putBoolean("Approaching Note", false);
        System.out.println("CameraIntakeCmd ended!");
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}