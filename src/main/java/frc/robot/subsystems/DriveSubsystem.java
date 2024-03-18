// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathWeaverConstants;
import frc.robot.Constants.encoderValues;
import frc.robot.helpers.ControlReversalStore;

//values are in METERS not FEET
public class DriveSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(DriveConstants.kLeftFrontMotorPort);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(DriveConstants.kRightFrontMotorPort);
  private final WPI_VictorSPX m_leftBackMotor = new WPI_VictorSPX(DriveConstants.kLeftBackMotorPort);
  private final WPI_VictorSPX m_rightBackMotor = new WPI_VictorSPX(DriveConstants.kRightBackMotorPort);

  private static Encoder leftEncoder = new Encoder(encoderValues.kLeftEncoderChannelA,
      encoderValues.kLeftEncoderChannelB);
  private static Encoder rightEncoder = new Encoder(encoderValues.kRightEncoderChannelA,
      encoderValues.kRightEncoderChannelB);

  private PIDController m_leftPIDController = new PIDController(PathWeaverConstants.kPDriveVel, 0, 0);
  private PIDController m_rightPIDController = new PIDController(PathWeaverConstants.kPDriveVel, 0, 0);

  private DifferentialDriveOdometry odometry;
  private final AHRS navX;

  private final ControlReversalStore m_controlReversal;

  int maxEncoderTicks = 8192;
  double circumference = Math.PI * 6 * 0.0254; // pi * distance * inches to meters // about .4785
                                               // just saying you guys know wpilib has a thing to auto convert in to m
                                               // right -jon
  double victorOutput = 0;

  // m_leftFrontMotor.follow(m_leftBackMotor);
  private final DifferentialDrive diffDrive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

  // possibly instantiate encoders in an init method (?) or constructor

  /** Creates a new ExampleSubsystem. */
  // CONSTRUCTOR
  public DriveSubsystem(ControlReversalStore control) {
    m_leftFrontMotor.setInverted(true);
    m_leftBackMotor.setInverted(true);
    navX = new AHRS();
    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftEncoderFeet(), getRightEncoderFeet());

    this.m_controlReversal = control;

    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    AutoBuilder.configureRamsete(
        this::getPose,
        this::resetOdometry,
        this::getCurrentSpeeds,
        (speeds) -> {
          DifferentialDriveWheelSpeeds wheelSpeeds = PathWeaverConstants.kDriveKinematics.toWheelSpeeds(speeds);
          driveVelocity(wheelSpeeds);
        },
        new ReplanningConfig(),
        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
        this);
  }

  /*
   * public ChassisSpeeds getCurrentSpeeds() {
   * return ChassisSpeeds.fromRobotRelativeSpeeds(
   * m_leftFrontMotor.getSelectedSensorVelocity() *
   * DriveConstants.kEncoderDistancePerPulse,
   * m_rightFrontMotor.getSelectedSensorVelocity() *
   * DriveConstants.kEncoderDistancePerPulse,
   * navX.getRate()
   * );
   * }
   */

  public void driveVelocity(DifferentialDriveWheelSpeeds speeds) {
    // diffDrive.feed();

    double leftVolts = m_leftPIDController.calculate(m_leftFrontMotor.getSelectedSensorVelocity() * (circumference / maxEncoderTicks) * 10, speeds.leftMetersPerSecond);
    double rightVolts = m_rightPIDController.calculate(m_rightFrontMotor.getSelectedSensorVelocity() * (circumference / maxEncoderTicks) * 10, speeds.rightMetersPerSecond);

    driveByVolts(leftVolts, rightVolts);
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return PathWeaverConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void setMotors(double moveSpeed, double turnSpeed) {

    if (this.m_controlReversal.getForwardSide() == "shooter") {
      diffDrive.arcadeDrive(-moveSpeed, turnSpeed * DriveConstants.returnLimit);
      return;
    }

    diffDrive.arcadeDrive(moveSpeed, turnSpeed * DriveConstants.returnLimit);
  }

  // public double getLeftEncoderMeters() {
  // double leftEncoderMeters = leftEncoder.get() *
  // DriveConstants.kEncoderTick2Feet;
  // return leftEncoderFeet;
  // }

  // public double getRightEncoderMeters() {
  // double rightEncoderMeters = -rightEncoder.get() *
  // DriveConstants.kEncoderTick2Feet;
  // return rightEncoderFeet;
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_leftFrontMotor.set(ControlMode.PercentOutput, 0);
    m_rightFrontMotor.set(ControlMode.PercentOutput, 0);

    SmartDashboard.putNumber("Left Encoder Feet", getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Encoder Feet", getRightEncoderFeet());
    SmartDashboard.putNumber("AVERAGE Encoder Feet", getEncoderFeetAverage());

    diffDrive.feed();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftFrontMotor.getSelectedSensorVelocity() * (circumference / maxEncoderTicks) * 10,
        m_rightFrontMotor.getSelectedSensorVelocity() * (circumference / maxEncoderTicks) * 10);
  }

  public DifferentialDriveWheelPositions getWheelPositions() {
    return new DifferentialDriveWheelPositions(getLeftEncoderFeet(), getRightEncoderFeet());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), getWheelPositions(), pose);
  };

  public double getEncoderFeetAverage() {
    return ((getLeftEncoderFeet() + getRightEncoderFeet()) / 2);
  }

  public double getLeftEncoderFeet() {
    double leftEncoderFeet = leftEncoder.get() * encoderValues.kEncoderTick2Feet;
    return leftEncoderFeet;
  }

  public double getRightEncoderFeet() {
    double rightEncoderFeet = -rightEncoder.get() * encoderValues.kEncoderTick2Feet;
    return rightEncoderFeet;
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  // Tested: Negative, negative
  public void driveByVolts(double leftVolts, double rightVolts) {
    m_leftFrontMotor.setVoltage(-leftVolts);
    m_rightFrontMotor.setVoltage(-rightVolts);
    diffDrive.feed();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}