package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;

public class Shooter implements Subsystem {
    TalonFX rightFlywheel;
    TalonFX leftFlywheel;
    TalonFX pitchMotor;
    CANBus kCANBus;
    TalonFXConfiguration rightFlywheelConfig = new TalonFXConfiguration();
    TalonFXConfiguration pitchConfig = new TalonFXConfiguration();


    public Shooter(int rightFlywheelCAN, int leftFlywheelCAN, int pitchCAN) {
        kCANBus = new CANBus("", "./logs/example.hoot");
        rightFlywheel = new TalonFX(rightFlywheelCAN, kCANBus);
        leftFlywheel = new TalonFX(leftFlywheelCAN, kCANBus);
        pitchMotor = new TalonFX(pitchCAN, kCANBus);

        // Configure the flywheel motors
        rightFlywheelConfig.withMotorOutput(new Follower(leftFlywheelCAN, null));
    }

}