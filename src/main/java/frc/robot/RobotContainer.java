// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.DecimalFormat;
import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.cameraConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.CameraIntakeCmd;
import frc.robot.helpers.ControlReversalStore;
import frc.robot.helpers.SprintSpeedController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cannon.CannonMotorSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
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
    private final ControlReversalStore m_controlReversal = new ControlReversalStore();
    public final SprintSpeedController m_speedController = new SprintSpeedController();

    // The robot's subsystems and commands are defined here...
    private final IntakeMotorSubsystem m_intakeMotorSubsystem = new IntakeMotorSubsystem();
    private final CannonMotorSubsystem m_cannonMotorSubsystem = new CannonMotorSubsystem();
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem(m_controlReversal);
    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();

    // Trajectory Generation for auto
    private SendableChooser<Command> m_chooser = new SendableChooser<>();
    public DecimalFormat decimalScale = new DecimalFormat("#,###.##");

    // camera instantiation
    private PhotonCamera noteCamera = new PhotonCamera(cameraConstants.NOTE_CAMERA);
    private PhotonCamera tagCamera = new PhotonCamera(cameraConstants.TAG_CAMERA);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        NamedCommands.registerCommand("Shoot", (new InstantCommand(
                () -> m_cannonMotorSubsystem.setCannonPower(Constants.cannonConstants.SPEAKER_FIRING_POWER),
                m_cannonMotorSubsystem)
                .alongWith(
                        new InstantCommand(m_indexerSubsystem::spinMotorFast, m_indexerSubsystem)))
                .andThen(
                        new WaitCommand(0.5))
                .andThen(
                        new InstantCommand(m_cannonMotorSubsystem::stopCannon, m_cannonMotorSubsystem)
                                .alongWith(new InstantCommand(m_indexerSubsystem::stopMotor, m_indexerSubsystem))));
        NamedCommands.registerCommand("Intake",
                (new InstantCommand(m_intakeMotorSubsystem::spinMotor, m_intakeMotorSubsystem)

                        .alongWith(
                                new InstantCommand(m_indexerSubsystem::spinMotor, m_indexerSubsystem)))
                        .until(m_indexerSubsystem::isBeamBroken));

        NamedCommands.registerCommand("Stop", new InstantCommand(m_intakeMotorSubsystem::stopMotor));

        m_chooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        m_chooser.addOption("Drive Time", DriveTimeCommand(5, 0.5));
        // Doesn't do anything
        m_chooser.addOption("No Auto", null);
        Shuffleboard.getTab("Auto")
                .add("chooser", m_chooser);

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
        m_driveSubsystem.setDefaultCommand(new ArcadeDriveCmd(m_driveSubsystem,
                () -> m_speedController.getSlowMultiplier() * m_driverController.getLeftY(),
                () -> m_speedController.getSlowMultiplier() * m_driverController.getLeftX()));

        // Configure the trigger bindings

        m_driverController.b().whileTrue(
                new StartEndCommand(
                        m_speedController::slowDown,
                        m_speedController::stopBeingSlow));

        m_driverController.rightTrigger().whileTrue(
                new StartEndCommand(
                        () -> m_cannonMotorSubsystem.setCannonPower(Constants.cannonConstants.SPEAKER_FIRING_POWER),
                        () -> m_cannonMotorSubsystem.setCannonPower(0), m_cannonMotorSubsystem));

        m_driverController.a().whileTrue(
                new StartEndCommand(m_indexerSubsystem::spinMotor, m_indexerSubsystem::stopMotor, m_indexerSubsystem));

        m_driverController.rightBumper().whileTrue(
                new CameraIntakeCmd(m_driveSubsystem, noteCamera, m_controlReversal).alongWith(
                        new StartEndCommand(m_intakeMotorSubsystem::spinMotor, m_intakeMotorSubsystem::stopMotor))
                        .unless(m_indexerSubsystem::isBeamBroken));
        m_driverController.leftTrigger().whileTrue(

                new StartEndCommand(m_intakeMotorSubsystem::spinMotor, m_intakeMotorSubsystem::stopMotor,
                        m_intakeMotorSubsystem)
                        .alongWith(
                                new StartEndCommand(m_indexerSubsystem::spinMotor, m_indexerSubsystem::stopMotor,
                                        m_indexerSubsystem))
                        .until(m_indexerSubsystem::isBeamBroken));

        m_driverController.leftBumper().whileTrue(
                new StartEndCommand(m_intakeMotorSubsystem::reverseSpinMotor, m_intakeMotorSubsystem::stopMotor,
                        m_intakeMotorSubsystem)
                        .alongWith(new StartEndCommand(m_indexerSubsystem::reverseMotor, m_indexerSubsystem::stopMotor,
                                m_indexerSubsystem)));

        m_driverController.x().whileTrue(
                new StartEndCommand(m_intakeMotorSubsystem::shakeItUp, m_intakeMotorSubsystem::stopMotor,
                        m_intakeMotorSubsystem)
                        .alongWith(
                                new StartEndCommand(m_indexerSubsystem::spinMotor, m_indexerSubsystem::stopMotor,
                                        m_indexerSubsystem)));

        // m_driverController.x().whileTrue(
        // new SequentialCommandGroup(new CameraIntakeCmd(m_driveSubsystem, noteCamera,
        // m_controlReversal)));
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

    public Command autoChooser() {

        return m_chooser.getSelected();
    }

    public Command DriveTimeCommand(double time, double speed) {
        return new RunCommand(() -> m_driveSubsystem.driveByVolts(speed, speed), m_driveSubsystem)
                .withTimeout(time);
    }

    public void resetEncoders() {
        m_driveSubsystem.resetEncoders();
    }

}
