package frc.robot.subsystems.helpers;

public class ControlReversalStore {
    public static class Store {
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
    }
}
