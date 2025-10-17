// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drive Subsystem. */
  private final int krightLeadId;
  private final int krightFollowId;
  private final int kleftLeadId;
  private final int kleftFollowId;
  private SparkMaxConfig krightConfig;
  private SparkMaxConfig kleftConfig;

  public DriveSubsystem(int rightLeadId, int rightFollowId, int leftLeadId, int leftFollowId) {
    krightLeadId = rightLeadId;
    krightFollowId = rightFollowId;
    kleftLeadId = leftLeadId;
    kleftFollowId = leftFollowId;

    final SparkMax m_rightLead = new SparkMax(krightLeadId, MotorType.kBrushed);
    final SparkMax m_rightFollow = new SparkMax(krightFollowId, MotorType.kBrushed);
    final SparkMax m_leftLead = new SparkMax(kleftLeadId, MotorType.kBrushed);
    final SparkMax m_leftFollow = new SparkMax(kleftFollowId, MotorType.kBrushed);

    final SparkMaxConfig krightConfig = new SparkMaxConfig();
    krightConfig.follow(krightLeadId);

    final SparkMaxConfig kleftConfig = new SparkMaxConfig();
    kleftConfig.follow(kleftLeadId);


    m_rightFollow.configure(krightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollow.configure(kleftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    final DifferentialDrive m_drive = 
    new DifferentialDrive(m_leftLead, m_rightLead);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
