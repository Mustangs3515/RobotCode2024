// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.SpinIntakeCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.cannon.CannonMotorSubsystem;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final IntakeMotorSubsystem m_intakeMotorSubsystem = new IntakeMotorSubsystem();
  private final CannonMotorSubsystem m_cannonMotorSubsystem = new CannonMotorSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();


  // Trajectory Generation for auto
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
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
    // Configure the trigger bindings so it intakes a note
    m_driverController.leftTrigger().whileTrue(
        new SpinIntakeCmd(m_intakeMotorSubsystem));

    // Fire at amp power
    m_driverController.leftBumper().onTrue(
       new RunCommand(() -> m_cannonMotorSubsystem.setCannonPower(Constants.cannonConstants.AMP_FIRING_POWER), m_cannonMotorSubsystem)
        .andThen(
          new WaitCommand(2)
        )
        .andThen(
          () -> m_cannonMotorSubsystem.setCannonPower(0)
        ));  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    SmartDashboard.putData(m_chooser);
    return m_chooser.getSelected();
  }
}
