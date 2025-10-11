package frc.robot.subsystems;

import edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_armMotor = new SparkMax(8, MotorType.kBrushless);
    private final SparkAbsoluteEncoder m_encoder = m_armMotor.getAbsoluteEncoder();
    public boolean m_homingComplete = false;

    private final ArmFeedforward m_armFeedforward = 
        new ArmFeedforward(
            0.1, 
            0.5, 
            0.8, 
            0
        );
    private final TrapezoidProfile.Constraints m_armProfile = 
        new TrapezoidProfile.Constraints(
                Degrees.of(90),
                Degrees.of(180)
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
        m_encoder.AbsoluteEncoderConfig.zeroOffset(offset);
    }
    private Radians getAngleRadians() {
        return m_encoder.getPosition().in(Radians);
    }
    public void goToAngle(Degrees targetAngleDegrees) {
        private Radians targetAngleRadians = targetAngleDegrees.in(Radians);

        // Calculate the PID and feedforward outputs based on the current state and target
        private Voltage pidOutput = m_armPID.calculate(getAngleRadians(), targetAngleRadians);

        // We need the profiled velocity to accurately calculate the feedforward
        // For simplicity here, we assume acceleration is near zero, and the profile handles velocity
        private Voltage feedForwardVoltage = ArmFeedforward.calculate(
            getAngleRadians(),
            m_armPID.getSetpoint().in(Velocity)
        );
        m_armMotor.setVoltage(pidOutput + feedForwardVoltage);
    }
    public boolean atSetpoint() {
        return m_armPID.atSetpoint();
    }

}
