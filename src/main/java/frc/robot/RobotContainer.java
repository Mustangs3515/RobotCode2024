// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.text.DecimalFormat;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.cameraConstants;
import frc.robot.auto.BlueMidAutoPath;
import frc.robot.auto.BlueTopAutoPath;
import frc.robot.auto.RedMidAutoPath;
import frc.robot.auto.RedRightAutoPath;
import frc.robot.auto.RightRedLeave;
import frc.robot.auto.LeftBlueLeave;
import frc.robot.auto.base.AutoPath;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.CameraIntakeCmd;
import frc.robot.commands.CannonCmd;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.IndexerCmd;
import frc.robot.commands.SpinIntakeCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cannon.CannonMotorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.helpers.ControlReversalStore;
import frc.robot.subsystems.helpers.SprintSpeedController;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.storage.BeamBreakSubsystem;
import frc.robot.subsystems.storage.IndexerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PIDController pidController = new PIDController(Constants.elevatorConstants.kP,
      Constants.elevatorConstants.kI, Constants.elevatorConstants.kD);
  private final BeamBreakSubsystem m_beamBreakSubsystem = new BeamBreakSubsystem();
  private final IntakeMotorSubsystem m_intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final CannonMotorSubsystem m_cannonMotorSubsystem = new CannonMotorSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(pidController);
  private final ControlReversalStore m_controlReversal = new ControlReversalStore();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_controlReversal);
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem(m_beamBreakSubsystem);
  public final SprintSpeedController m_speedController = new SprintSpeedController();


  // Trajectory Generation for auto
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private String trajectoryJSON;
  private Trajectory test1Trajectory;
  private Path trajectoryPath;
  private RamseteCommand ramseteCommand;
  public DecimalFormat decimalScale = new DecimalFormat("#,###.##");

  //camera instantiation
  private PhotonCamera noteCamera = new PhotonCamera(cameraConstants.NOTE_CAMERA);
  private PhotonCamera tagCamera = new PhotonCamera(cameraConstants.TAG_CAMERA);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(m_driveSubsystem, () -> m_speedController.getSlowMultiplier()*m_driverController.getLeftY(), () -> m_speedController.getSlowMultiplier()*m_driverController.getLeftX()));
    
    TrajectoryConfig revConfig = new TrajectoryConfig(2, 3).setReversed(true);
    // Configure the trigger bindings

    // B button for slow speed
    m_driverController.b().whileTrue(
      new StartEndCommand(
        m_speedController::slowDown,
        m_speedController::stopBeingSlow
      )
    );

    // shoot the cannon with the right trigger
    m_driverController.rightTrigger().whileTrue(
      new CannonCmd(m_cannonMotorSubsystem, Constants.cannonConstants.SPEAKER_FIRING_POWER)
    );

    // run the indexer forward with the A button
    m_driverController.a().whileTrue(
      new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_SPIN_SPEED)
    );

    // right bumper is the auto intake
    m_driverController.rightBumper().whileTrue(
          new CameraIntakeCmd(m_driveSubsystem, noteCamera, m_controlReversal).alongWith(
            new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED))
            .unless(m_beamBreakSubsystem::isBeamBroken) //might need to remove this line since beambreak is already coded in SpinIntakeCmd
    );

    // Left Trigger is the Intake button --> runs intake & indexer forward
    m_driverController.leftTrigger().whileTrue(
      new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED)
      .alongWith(new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_SPIN_SPEED))
      .until(m_beamBreakSubsystem::isBeamBroken));

      // Left Bumper is for Reverse Intake --> spin intake & indexer in reverse
      m_driverController.leftBumper().whileTrue(
        new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem, Constants.intakeConstants.INTAKE_MOTOR_REVERSE_SPEED, Constants.intakeConstants.INTAKE_MOTOR_REVERSE_SPEED)
        .alongWith(new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_REVERSE_SPEED))
      );

      // X Button for SOS --> runs Intake with Spark 10 Forward & Spark 11 Reverse + the Indexer Forward
      m_driverController.x().whileTrue(
        new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED, Constants.intakeConstants.INTAKE_MOTOR_REVERSE_SPEED)
        .alongWith(new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_SPIN_SPEED))
      );

      // Right Stick Click
      m_driverController.rightStick().whileTrue(
        new CameraIntakeCmd(m_driveSubsystem, noteCamera, m_controlReversal)
      );

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    System.out.println("we runnin auton");
    return m_chooser.getSelected();
  }

  public Command noAuto()
  {
    return Commands.runOnce(() -> System.out.println("No sirree"));
  }

  public Command shootOnce()
  {
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new CannonCmd(m_cannonMotorSubsystem, Constants.cannonConstants.SPEAKER_FIRING_POWER)),
      new ParallelDeadlineGroup(
        new WaitCommand(2),
        new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_SPIN_SPEED)),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new CannonCmd(m_cannonMotorSubsystem, 0),
        new IndexerCmd(m_indexerSubsystem, 0)
      ));
  }

  public Command shootOnceThreeSecondsLater()
  {
    return new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(3),
        new CannonCmd(m_cannonMotorSubsystem, Constants.cannonConstants.SPEAKER_FIRING_POWER)),
      new ParallelDeadlineGroup(
        new WaitCommand(4),
        new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_SPIN_SPEED)),
      new CannonCmd(m_cannonMotorSubsystem, 0),
      new IndexerCmd(m_indexerSubsystem, 0));
  }

  public Command DriveForward2Feet()
  {
    return new DriveForwardCmd(m_driveSubsystem, -0.5);
  }

  // public Command midAutoTwoNotes()
  // {
  //   return new SequentialCommandGroup(
  //     shootOnce(),
  //     new WaitCommand(0.5),
  //     new ParallelRaceGroup(
  //       new DriveForwardCmd(m_driveSubsystem, 5)),
  //       //new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED)),
  //     new DriveForwardCmd(m_driveSubsystem, -m_driveSubsystem.getEncoderFeetAverage()),
  //     shootOnce() 
  //   );
  // }

  public Command midAutoTwoNotes()
  {
    return new SequentialCommandGroup(
      shootOnce(),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new DriveForwardCmd(m_driveSubsystem, -5),
          new WaitCommand(1),
          new DriveForwardCmd(m_driveSubsystem, 5)),
        new ParallelDeadlineGroup(
          new WaitCommand(2),
          new IndexerCmd(m_indexerSubsystem, Constants.storageConstants.INDEXER_SPIN_SPEED),
          new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED, Constants.intakeConstants.INTAKE_MOTOR_SPIN_SPEED)),
        shootOnce()));
  }

  public Command autoChooser() {
    // m_chooser.setDefaultOption("Blue Left Auto Path", DriveTheAutoPathCommand(new BlueTopAutoPath()));
    // m_chooser.addOption("Blue Mid Auto Path", DriveTheAutoPathCommand(new BlueMidAutoPath()));
    // m_chooser.addOption("Red Right Auto Path", DriveTheAutoPathCommand(new RedRightAutoPath()));
    // m_chooser.addOption("Red Mid Auto Path", DriveTheAutoPathCommand(new RedMidAutoPath()));

    m_chooser.addOption("No Auto", noAuto());
    m_chooser.addOption("Only Shoot", shootOnce());
    m_chooser.addOption("Shoot Once 3 Seconds Later", shootOnceThreeSecondsLater());
    m_chooser.addOption("Drive Forward 2 Feet", DriveForward2Feet());
    m_chooser.addOption("Mid 2 Note", midAutoTwoNotes());

    Shuffleboard.getTab("Auto")
     .add("chooser", m_chooser);
    return m_chooser.getSelected();
  }
  
  public Command PathCommand(String trajectoryName, String givenName) {
    trajectoryJSON = "PathWeaver/output/" + givenName + ".wpilib.json"; // "paths/"+ trajectoryName + ".wpilib.json"
    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      test1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    ramseteCommand = new RamseteCommand(
        test1Trajectory,
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

    System.out.println("Initial Pose" + decimalScale.format(test1Trajectory.getInitialPose().getX()) + ", "
        + decimalScale.format(test1Trajectory.getInitialPose().getY()) + ")");
    SmartDashboard.putString("Initial Pose", "(" + decimalScale.format(test1Trajectory.getInitialPose().getX()) + ", "
        + decimalScale.format(test1Trajectory.getInitialPose().getY()) + ")");
    SmartDashboard.putString("Pose", test1Trajectory.getInitialPose().toString());
    System.out.println(test1Trajectory.getInitialPose().toString());

    // Reset robot odometry to initial position of path
    Command resetCommand = new InstantCommand(() -> Robot.m_drivetrain.resetOdometry(test1Trajectory.getInitialPose()));
    // Create CommandGroup of the resetCommand and our ramseteCommand
    Command returnGroup = new SequentialCommandGroup(resetCommand,
        ramseteCommand.andThen(() -> Robot.m_drivetrain.driveByVolts(0, 0)));
    return returnGroup;
  }
  
}
