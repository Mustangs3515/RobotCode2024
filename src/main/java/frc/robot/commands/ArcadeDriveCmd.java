package frc.robot.commands;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class ArcadeDriveCmd extends Command{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> leftSpeedFunction, rightSpeedFunction;
    public ArcadeDriveCmd(DriveSubsystem driveSubsystem, Supplier<Double> leftSpeedFunction, Supplier<Double> rightSpeedFunction) 
    {
        this.driveSubsystem = driveSubsystem;
        this.leftSpeedFunction = leftSpeedFunction;
        this.rightSpeedFunction = rightSpeedFunction;
        addRequirements(driveSubsystem);
    } 

    // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            System.out.println("ArcadeDriveCmd started!");
        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            driveSubsystem.setMotors(leftSpeedFunction.get(), rightSpeedFunction.get());
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
            System.out.println("ArcadeDriveCmd ended!");
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
            return false;
        }
  }
