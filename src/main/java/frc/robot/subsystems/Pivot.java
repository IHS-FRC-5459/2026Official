// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import frc.robot.Constants.Sensors.Ports;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final TalonFX pivotController;
  private Encoder pivotEncoder;
  private final ArmFeedforward pivotFeedforward;
  private final PIDController pivotPID;
  private double pivotSetpoint = 0;
  private final double setpointDeadspace = 10;
  private final double downSetpoint = 0;
  private final double upSetpoint = 100;
  private final String loggingPrefix = "subsystems/pivot/";

  public Pivot() {
    pivotController = new TalonFX(Motors.pivotId);
    pivotEncoder = new Encoder(Ports.PivotEncoderPort1, Ports.PivotEncoderPort2);
    pivotFeedforward = new ArmFeedforward(0, 0.7, 0);
    pivotPID = new PIDController(1.2, 0.4, 0);
    pivotEncoder.setDistancePerPulse(0.02 * (90 / 10.45));
    // This happends to be about encoder dist = degrees of pivot
    pivotEncoder.reset();
  }

  public void setGoal(double goal) {
    this.pivotSetpoint = goal;
    pivotPID.setSetpoint(pivotSetpoint * Math.PI / 180);
  }

  public double getGoal() {
    return this.pivotSetpoint;
  }

  public void setVoltage(double volts) {
    pivotController.setVoltage(volts);
  }

  public double getVoltage() {
    return pivotController.get();
  }

  public double getEncoderDist() {
    return (pivotEncoder.getDistance()) + 90;
  }

  public double getEncoderRadians() {
    return getEncoderDist() * Math.PI / 180;
  }

  public void resetPID() {
    pivotPID.reset();
  }

  public void resetEncoder() {
    pivotEncoder.reset();
  }

  public void updateMotorOutput() {
    double pidVolts = pivotPID.calculate(getEncoderRadians());
    double ffVolts = pivotFeedforward.calculate(getEncoderRadians(), 0);
    double volts = pidVolts + ffVolts;
    pivotController.setVoltage(volts);
    Logger.recordOutput(loggingPrefix + "pidVolts: ", pidVolts);
    Logger.recordOutput(loggingPrefix + "ffVolts ", ffVolts);
    Logger.recordOutput(loggingPrefix + "volts", volts);
  }

  public void goDown() {
    setGoal(downSetpoint);
  }

  public void goUp() {
    setGoal(upSetpoint);
  }

  public void goOpposite() {
    if (!isAtSetpoint()) {
      return;
    }
    if (getGoal() == downSetpoint) {
      goUp();
    } else {
      goDown();
    }
  }

  public boolean
      isAtSetpoint() { // I putposely don't use the build in function for this because it is too
    // exact
    return pivotPID.getError() < setpointDeadspace;
  }

  @Override
  public void periodic() {
    // pivotPID.setP(SmartDashboard.getNumber("pivotPID_P", 0));
    // pivotPID.setI(SmartDashboard.getNumber("pivotPID_I", 0));
    // pivotPID.setD(SmartDashboard.getNumber("pivotPID_D", 0));
    // pivotFeedforward.setKg(SmartDashboard.getNumber("pivotFF_G", 0));
    Logger.recordOutput(loggingPrefix + "kP", pivotPID.getP());
    Logger.recordOutput(loggingPrefix + "kI", pivotPID.getI());
    Logger.recordOutput(loggingPrefix + "kD", pivotPID.getD());
    Logger.recordOutput(loggingPrefix + "kG", pivotFeedforward.getKg());

    // This method will be called once per scheduler run
    Logger.recordOutput(loggingPrefix + "EncoderReading", getEncoderDist());
    Logger.recordOutput(loggingPrefix + "goal", getGoal());
    Logger.recordOutput(loggingPrefix + "isAtSetpoint", isAtSetpoint());
    Logger.recordOutput(loggingPrefix + "error", pivotPID.getError());
    setGoal(SmartDashboard.getNumber("pivotGoal", 0));
    updateMotorOutput();
    // setVoltage(SmartDashboard.getNumber("pivotGoal", 0));
  }
}
