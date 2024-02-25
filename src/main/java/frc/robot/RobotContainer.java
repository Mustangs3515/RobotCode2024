// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SpinIntakeCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.cannon.CannonMotorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import frc.robot.subsystems.storage.BeamBreakSubsystem;
import frc.robot.Robot;

import java.io.IOException;
import java.nio.file.Path;
import java.text.DecimalFormat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final BeamBreakSubsystem m_beamBreakSubsystem = new BeamBreakSubsystem();
  private final IntakeMotorSubsystem m_intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final CannonMotorSubsystem m_cannonMotorSubsystem = new CannonMotorSubsystem();
  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem(pidController);
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  // Trajectory Generation for auto
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();
  private String trajectoryJSON;
  private Trajectory test1Trajectory;
  private Path trajectoryPath;
  private RamseteCommand ramseteCommand;
  public DecimalFormat decimalScale = new DecimalFormat("#,###.##");
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    TrajectoryConfig revConfig = new TrajectoryConfig(2, 3).setReversed(true);
    // Configure the trigger bindings
    // m_driverController.leftTrigger().whileTrue(
    //     new SpinIntakeCmd(m_beamBreakSubsystem, m_intakeMotorSubsystem));
    // m_driverController.leftBumper().whileTrue( //bumper RIGHt for speaker & left bumper amp
    //     new StartEndCommand(
    //         () -> m_cannonMotorSubsystem.setCannonPower(Constants.cannonConstants.AMP_FIRING_POWER),
    //         () -> m_cannonMotorSubsystem.setCannonPower(0),
    //         m_cannonMotorSubsystem));
    // m_driverController.rightTrigger().whileTrue(
    //     new StartEndCommand(
    //         () -> m_cannonMotorSubsystem.setCannonPower(Constants.cannonConstants.CANNON_FIRING_POWER),
    //         () -> m_cannonMotorSubsystem.setCannonPower(0),
    //         m_cannonMotorSubsystem));

    // m_driverController.a().onTrue(
    //     new RunCommand(() -> m_elevatorSubsystem.setSetpoint(Constants.elevatorConstants.RESET_ELEVATOR_EXTENSION_DISTANCE),
    //         m_elevatorSubsystem));

    // m_driverController.x().onTrue(
    //     new RunCommand(() -> m_elevatorSubsystem.setSetpoint(Constants.elevatorConstants.AMP_ELEVATOR_EXTENSION_DISTANCE),
    //         m_elevatorSubsystem));

    // m_driverController.b().onTrue(
    //     new RunCommand(() -> m_elevatorSubsystem.setSetpoint(Constants.elevatorConstants.CLIMB_ELEVATOR_EXTENSION_DISTANCE),
    //         m_elevatorSubsystem));

    configureBindings();

    // m_chooser.setDefaultOption("Left Amp", PathCommand("LeftAmp"));
    // SmartDashboard.putData(m_chooser);
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    m_chooser.setDefaultOption("Left Amp", PathCommand("LeftAmp"));
    SmartDashboard.putData(m_chooser);
    return m_chooser.getSelected();
  }

  public Command PathCommand(String trajectoryName)
  {
    trajectoryJSON = "PathWeaver/output/leftAmp.wpilib.json"; //"paths/"+ trajectoryName + ".wpilib.json"
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
            Robot.m_drivetrain
        );

        
        System.out.println("Initial Pose" + decimalScale.format(test1Trajectory.getInitialPose().getX()) + ", " + decimalScale.format(test1Trajectory.getInitialPose().getY()) + ")");
        SmartDashboard.putString("Initial Pose", "(" + decimalScale.format(test1Trajectory.getInitialPose().getX()) + ", " + decimalScale.format(test1Trajectory.getInitialPose().getY()) + ")");
        SmartDashboard.putString("Pose", test1Trajectory.getInitialPose().toString());
        System.out.println(test1Trajectory.getInitialPose().toString());

        // Reset robot odometry to initial position of path
        Command resetCommand = new InstantCommand(() -> Robot.m_drivetrain.resetOdometry(test1Trajectory.getInitialPose()));
        // Create CommandGroup of the resetCommand and our ramseteCommand
        Command returnGroup = new SequentialCommandGroup(resetCommand, ramseteCommand.andThen(() -> Robot.m_drivetrain.driveByVolts(0, 0)));
        return returnGroup;
  }
}
