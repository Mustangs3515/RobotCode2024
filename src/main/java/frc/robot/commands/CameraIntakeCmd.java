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
    // private GenericEntry P = tab.add("P", .05).getEntry();
    // private GenericEntry I = tab.add("I", 0).getEntry();
    // private GenericEntry D = tab.add("D", 0).getEntry();
    private PIDController pid = new PIDController(DriveConstants.kPCameraSteer, DriveConstants.kICameraSteer, DriveConstants.kDCameraSteer);

    public CameraIntakeCmd(DriveSubsystem driveSubsystem, PhotonCamera camera, ControlReversalStore m_controlReversal) {
        this.driveSubsystem = driveSubsystem;
        this.camera = camera;
        this.m_controlReversal = m_controlReversal;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("CameraIntakeCmd started!");
        pid.setSetpoint(0);
        pid.reset();
        // pid.setP(P.getDouble(0));
        // pid.setI(I.getDouble(0));
        // pid.setD(D.getDouble(0));
        System.out.println(" P = " + pid.getP() + " I = " + pid.getI() + " D = " + pid.getD());
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
         
        if (Math.abs(yaw) > .05) 
        {
            this.driveSubsystem.setMotors(0, -(pid.calculate(yaw) / 1.5));
            System.out.println(pid.calculate(yaw) / 1.5);
            return;
        }
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