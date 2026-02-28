// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.Constants.Sensors.Ports;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final SparkMax hoodController;
  private final Encoder hoodEncoder;
  private final ArmFeedforward hoodFeedforward;
  private final PIDController hoodPID;
  private double hoodSetpoint = 0;

  private final String loggingPrefix = "subsystems/hood/";

  public Hood() {
    hoodController = new SparkMax(Motors.hoodId, MotorType.kBrushless);
    hoodEncoder = new Encoder(Ports.HoodEncoderPort1, Ports.HoodEncoderPort2);
    hoodFeedforward = new ArmFeedforward(0, 0.5, 0);
    hoodPID = new PIDController(4, 1, 0.002);
    hoodEncoder.setDistancePerPulse(0.02);
    // This happends to be about encoder dist = degrees of hood
    hoodEncoder.reset();
  }

  public void setGoal(double goal) {
    this.hoodSetpoint = goal;
    hoodPID.setSetpoint(hoodSetpoint * Math.PI / 180);
  }

  public double getGoal() {
    return this.hoodSetpoint;
  }

  public void setVoltage(double volts) {
    hoodController.setVoltage(-volts);
  }

  public double getVoltage() {
    return -hoodController.get();
  }

  public double getEncoderDist() {
    return -hoodEncoder.getDistance();
  }

  public double getEncoderRadians() {
    return getEncoderDist() * Math.PI / 180;
  }

  public void updateMotorOutput() {
    double pidVolts = hoodPID.calculate(getEncoderRadians());
    double ffVolts = hoodFeedforward.calculate(getEncoderRadians(), hoodEncoder.getRate());
    double volts = pidVolts + ffVolts;
    hoodController.setVoltage(volts);
    Logger.recordOutput(loggingPrefix + "pidVolts:", pidVolts);
    Logger.recordOutput(loggingPrefix + "ffVolts", ffVolts);
    Logger.recordOutput(loggingPrefix + "volts", volts);
  }

  public void resetEncoder() {
    hoodEncoder.reset();
  }

  public void resetPID() {
    hoodPID.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput(loggingPrefix + "EncoderReading", getEncoderDist());
    Logger.recordOutput(loggingPrefix + "goal", getGoal());
    updateMotorOutput();
    Logger.recordOutput(loggingPrefix + "debug", hoodController.getOutputCurrent());
  }
}
