package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.cannon.CannonMotorSubsystem;

public class CannonCmd extends Command{
    private CannonMotorSubsystem cannon = new CannonMotorSubsystem();
    double speed = 0;

    public CannonCmd(CannonMotorSubsystem theCannon, double theSpeed)
    {
        this.cannon = theCannon;
        this.speed = theSpeed;
    }

    // if the beam is not broken, then the intake motor spins
    @Override
    public void execute() {
        cannon.setCannonPower(speed);
    }

    @Override
    public void initialize() {
        System.out.println("CannonCmd started!");
    }

    // stops motor
    @Override
    public void end(boolean interrupted){
        System.out.println("CannonCmd ended!");
        cannon.stopCannon();
    }

    @Override
    public boolean isFinished() {
        
        return false;
    }
}
