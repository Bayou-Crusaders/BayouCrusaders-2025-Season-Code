package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.*;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_armMotor = new SparkMax(8, MotorType.kBrushless);
    private final SparkAbsoluteEncoder m_encoder = m_armMotor.getAbsoluteEncoder();
    public boolean m_homingComplete = false;
    private final AbsoluteEncoderConfig config = new AbsoluteEncoderConfig();

    private final ArmFeedforward m_armFeedforward = 
        new ArmFeedforward(
            0.0, 
            0.0, 
            0.0, 
            0
        );
    private final TrapezoidProfile.Constraints m_armProfile = 
        new TrapezoidProfile.Constraints(
                Units.degreesToRadians(90),
                Units.degreesToRadians(180)
            );
    private final ProfiledPIDController m_armPID = new ProfiledPIDController(
        0.0,
        0.0,
        0.0,
        m_armProfile
    );

    public void stopMotor() {
        m_armMotor.stopMotor();
    }
    public void setVoltage(float voltage) {
        m_armMotor.setVoltage(voltage);
    }
    public void zeroEncoder(float offset) {
        config.apply(config.zeroOffset(offset));
    }
    private double getAngleRadians() {
        return Units.rotationsToRadians(m_encoder.getPosition());
    }
    public void goToAngle(double targetAngleDegrees) {
        double targetAngleRadians = Units.degreesToRadians(targetAngleDegrees);

        // Calculate the PID and feedforward outputs based on the current state and target
        double pidOutput = m_armPID.calculate(getAngleRadians(), targetAngleRadians);

        // We need the profiled velocity to accurately calculate the feedforward
        // For simplicity here, we assume acceleration is near zero, and the profile handles velocity
        double feedForwardVoltage = m_armFeedforward.calculate(
            targetAngleRadians,
            m_armPID.getSetpoint().velocity
        );
        m_armMotor.setVoltage(pidOutput + feedForwardVoltage);
    }
    public boolean atSetpoint() {
        return m_armPID.atSetpoint();
    }

}
