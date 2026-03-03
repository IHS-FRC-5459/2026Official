// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import org.littletonrobotics.junction.Logger;

public class Belt extends SubsystemBase {
  private SparkMax motor;
  private double commandedSpeed = 0;
  private RelativeEncoder encoder;
  private final String loggingPrefix = "subsystems/belt/";
  // private SimpleMotorFeedforward

  public Belt() {
    motor = new SparkMax(Motors.beltId, MotorType.kBrushless);
    encoder = motor.getEncoder();
  }

  public void setSpeed(double power) {
    motor.set(-power);
    commandedSpeed = -power;
    Logger.recordOutput(loggingPrefix + "commandedSpeed", -power);
  }

  public double getCurrentSpeed() {
    return (encoder.getVelocity() / 10000) / 1.19453; // 1.19453 is the conversion factor
  }

  @Override
  public void periodic() {
    Logger.recordOutput(loggingPrefix + "commandedSpeed", commandedSpeed);
    // This method will be called once per scheduler run
    Logger.recordOutput(loggingPrefix + "integratedEncoderVelocity", getCurrentSpeed());
  }
}
