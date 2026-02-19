package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Targeting implements Subsystem {
    private Translation2d target = new Translation2d();

    public Targeting() {}

    public void moveTargetPose(Double x, Double y) {
        target = target.plus(new Translation2d(x, y));
    }

    public Translation2d getTargetPose() {
        return new Translation2d(target.getX(), target.getY());
    }

    public double[] getTargetXY() {
        return new double[] {target.getX(), target.getY()};
    }

    public Double distanceToTarget(Pose2d RobotPose) {
        Translation2d robotPose = RobotPose.getTranslation();
        Translation2d targetPose = this.getTargetPose();
        return robotPose.getDistance(targetPose);
    }
}
