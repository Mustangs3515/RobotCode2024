package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeMotorSubsystem;

public class SpinIntakeCmd extends Command {
    private IntakeMotorSubsystem m_IntakeMotorSubsystem;

    public SpinIntakeCmd(IntakeMotorSubsystem intake_motor_subsystem) {
        this.m_IntakeMotorSubsystem = intake_motor_subsystem;
    }

    @Override
    public void execute() {
        m_IntakeMotorSubsystem.spinMotor();
    }

    @Override
    public void end(boolean interrupted){
         m_IntakeMotorSubsystem.stopMotor();
    }
}
