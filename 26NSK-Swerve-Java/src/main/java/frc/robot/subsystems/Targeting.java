
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Targeting implements Subsystem {
    private Pose2d target = new Pose2d();

    public Targeting() {}

    public void moveTargetPose(Double x, Double y) {
        target.transformBy(new Translation2d(x, y));
    }

    public Translation2d getTargetPose() {
        return target.getTranslation()
    }

    public Double distanceToTarget(Pose2d RobotPose) {
        Translation2d robotPose = RobotPose.getTranslation();
        Translation2d targetPose = this.getTargetPose();
        return robotPose.getDistance(targetPose);
    }
}