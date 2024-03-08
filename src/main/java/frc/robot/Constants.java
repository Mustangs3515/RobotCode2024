// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class storageConstants {
    public static final int BEAM_BREAK_RECEIVER_DIO = 9;
    public static final int INDEXER_MOTOR_ID = 8;
    public static final double INDEXER_SPIN_SPEED = 0.8;
    public static final double INDEXER_SPIN_SPEED_FAST = 0.7;
  }

  public static class intakeConstants {
    public static final int TOP_INTAKE_MOTOR_CAN_ID = 10;
    public static final int BOTTOM_INTAKE_MOTOR_CAN_ID = 11;
    public static final double INTAKE_MOTOR_SPIN_SPEED = 0.2;

    public static final double kP = 2;
    public static final double kI = 0.5;
    public static final double kD = 0;
  }

  public static class cannonConstants {

    public static final int RIGHT_MOTOR_CAN_ID = 7;
    public static final int LEFT_MOTOR_CAN_ID = 9;
    public static final double AMP_FIRING_POWER = 0.5;
    public static final double SPEAKER_FIRING_POWER = 1;

    public static final double kP = 2;
    public static final double kI = 0.5;
    public static final double kD = 0;
  }

  public static class elevatorConstants {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double AMP_ELEVATOR_EXTENSION_DISTANCE = 0;
    public static final double CLIMB_ELEVATOR_EXTENSION_DISTANCE = 0;
    public static final double RESET_ELEVATOR_EXTENSION_DISTANCE = 0;
    public static final int ELEVATOR_MOTOR_CAN_ID = 488;
  }

  public static class PathWeaverConstants 
  {
    //Given values
    public static final double ksVolts = 0.688f;
    public static final double kvVoltSecondsPerMeter = 2.52f;
    public static final double kaVoltSecondsSquaredPerMeter = 0.653f;
    public static final double kPDriveVel = 2.56f;

    public static final double kTrackwidthMeters = 0.612f; 
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    
    //Our values
    public static final double kMaxSpeedMetersPerSecond = 2.00f; //Was 4.00f
    public static final double kMaxAccelerationMetersPerSecondSquared = .20f; //Was 2.00f

    //Default values
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
  public static class encoderValues{
    public static final int kLeftEncoderChannelA = 0;
    public static final int kLeftEncoderChannelB = 1;
    public static final int kRightEncoderChannelA = 2;
    public static final int kRightEncoderChannelB = 3;
    public static final double kEncoderTick2Feet = (1.0 / 2048.0 * Math.PI * 9 / 17.8) * 0.3048;
  }

  public static class DriveConstants{
    public static final int kLeftFrontMotorPort = 6;
    public static final int kRightFrontMotorPort = 5;
    public static final int kLeftBackMotorPort = 3;
    public static final int kRightBackMotorPort = 2; // we are using this for kLeftEncoderChannelA as well

    public static final double returnLimit = 0.6;
    public static final double DEAD_ZONE_THRESHOLD = 0.1;
    public static final double LEFT_MOTOR_CONTROLLER_DISTORTION = 0.1;
    public static final double RIGHT_MOTOR_CONTROLLER_DISTORTION = 1.2;

    public static final double kAutoDriveForwardDistance = -7;
    public static final double kAutoDriveForwardSpeed = 0.5;
  }

  public static class cameraConstants {
    public static final String TAG_CAMERA = "Arducam_OV9281_USB_Camera";
    public static final String NOTE_CAMERA = "Microsoft_LifeCam_HD-3000";
  }

}
