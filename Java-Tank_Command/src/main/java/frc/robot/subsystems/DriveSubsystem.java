// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Class that can interface with a differental drivechain
 */
public class DriveSubsystem extends SubsystemBase {
  /** Creates a new Drive Subsystem. */

  //Initilaze variables
  private int krightLeadId;
  private int krightFollowId;
  private int kleftLeadId;
  private int kleftFollowId;
  private SparkMax m_rightLead;
  private SparkMax m_leftLead;
  private SparkMaxConfig krightConfig;
  private SparkMaxConfig kleftConfig;

  /**
   * Use if one motor per side 
   * 
   * @param krightId CAN ID of right side motor
   * @param kleftId CAN ID of left side motor
  */
  public DriveSubsystem(int krightId, int kleftId) {
    //Initilaze the two SPARKMax motors
    final SparkMax m_rightLead = new SparkMax(krightLeadId, MotorType.kBrushed);
    final SparkMax m_leftLead = new SparkMax(kleftLeadId, MotorType.kBrushed);
  }
  
  /**
   * Use if two motors per side 
   * 
   * @param krightLeadId CAN ID of right side lead motor
   * @param krightFollowId CAN Id of right side follow motor
   * @param kleftLeadId CAN ID of left side lead motor
   * @param kleftFollowId CAN ID of left side follow motor
  */
  public DriveSubsystem(int krightLeadId, int krightFollowId, int kleftLeadId, int kleftFollowId) {
    //Initilaze the four SPARKMax motors
    final SparkMax m_rightLead = new SparkMax(krightLeadId, MotorType.kBrushed);
    final SparkMax m_rightFollow = new SparkMax(krightFollowId, MotorType.kBrushed);
    final SparkMax m_leftLead = new SparkMax(kleftLeadId, MotorType.kBrushed);
    final SparkMax m_leftFollow = new SparkMax(kleftFollowId, MotorType.kBrushed);

    //Create the configs for the follow motors
    final SparkMaxConfig krightConfig = new SparkMaxConfig();
    final SparkMaxConfig kleftConfig = new SparkMaxConfig();
    
    //Makes the motors follow another motor
    krightConfig.follow(krightLeadId);
    kleftConfig.follow(kleftLeadId);

    //Apply the configs to the follow motors
    m_rightFollow.configure(krightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftFollow.configure(kleftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Drive using Tank-Style Commands
   *
   * @param leftSpeed Speed of left side, 1 = 100% Speed
   * @param rightSpeed Speed of right side, 1 = 100% Speed
   * 
   * @return A command to run
   */
  public Command tankDriveCommand(double leftSpeed, double rightSpeed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.runOnce(
      () -> {
        m_leftLead.setVoltage(leftSpeed * 12);
        m_rightLead.setVoltage(rightSpeed * 12);
      },
      this
    );
  }

  /**
   * Drive using Arcade-Style Commands
   *
   * @param speed Speed in the forward/backward plane, 1 = 100% Speed
   * @param rot Speed to rotate in the x/y plane, 1 = 100% Speed
   * 
   * @return A command to run
   */
  public Command arcadeDriveCommand(double speed, double rot) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return Commands.runOnce(
        () -> {
          m_leftLead.setVoltage((speed + rot) * 12);
          m_rightLead.setVoltage((speed - rot) * 12);
        },
        this
    );
  }
}
