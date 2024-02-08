// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int BEAM_BREAK_RECEIVER_DIO = 0;
  }

  public static class intakeConstants {
    public static final int INTAKE_MOTOR_CAN_ID = 0;
    public static final double INTAKE_MOTOR_SPIN_SPEED = 0;
  }

  public static class cannonConstants {

    public static final int RIGHT_MOTOR_CAN_ID = 0;
    public static final int LEFT_MOTOR_CAN_ID = 0;
    public static final double AMP_FIRING_POWER = 0.5;
    public static final double CANNON_FIRING_POWER = 1;
  }

  public static class elevatorConstants {
    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0;
  }
}
