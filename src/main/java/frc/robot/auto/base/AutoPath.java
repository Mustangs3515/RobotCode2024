package frc.robot.auto.base;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;

public class AutoPath {

    private String p1Path;
    private String p2Path;
    private String p3Path;

    public AutoPath(String p1PathName, String p2PathName, String p3PathName){
        this.p1Path = p1PathName;
        this.p2Path = p2PathName;
        this.p3Path = p3PathName;
    }
    public Trajectory accessTrajectory(String path) throws IOException {
        return TrajectoryUtil.fromPathweaverJson(this.getTrajectoryPath(path));
    }

    public Trajectory p1() throws IOException{
        return this.accessTrajectory(p1Path);
    }

    public Trajectory p2() throws IOException{
        return this.accessTrajectory(p2Path);
    }

    public Trajectory p3() throws IOException{
        return this.accessTrajectory(p3Path);
    }

    private Path getTrajectoryPath(String pathName){
        return Filesystem.getDeployDirectory().toPath().resolve(pathName + ".wpilib.json");
    }
}
