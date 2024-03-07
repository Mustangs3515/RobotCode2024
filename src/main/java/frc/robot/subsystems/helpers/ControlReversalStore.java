package frc.robot.subsystems.helpers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ControlReversalStore {
    // this class reverses the direction of the robot --> either intake side or shooter side

        private boolean isForwardIntakeSide = true;

        public void setForwardSideToBeIntake(){
            isForwardIntakeSide = true;
        }

        public void setForwardSideToBeShooter(){
            isForwardIntakeSide = false;
        }

        public String getForwardSide(){
            if(isForwardIntakeSide)
            {
                return "intake";
            }
            else{
                return "shooter";
            }
        }

        public void toggleForwardSide() {
            isForwardIntakeSide = !isForwardIntakeSide;
            SmartDashboard.putBoolean("Forward side", isForwardIntakeSide);
        }
    
}
