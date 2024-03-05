package frc.robot.subsystems.helpers;

public class SprintSpeedController {
    private double slowMultiplier = 1;


    public void slowDown(){
      this.slowMultiplier = 0.75;
    }
  
    public void stopBeingSlow(){
      this.slowMultiplier = 1;
    }
  
    public double getSlowMultiplier(){
      return this.slowMultiplier;
    }
}
