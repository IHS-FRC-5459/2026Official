// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Motors;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  TalonFX motor;
  // Encoder m_encoder;
  private double goal = 0;
  double kMaxVelocity = 1;
  double kS = 1;
  double kMaxAcceleration = 1;
  double kP = 1;
  double kI = 1;
  double kD = 1;
  double kG = 1;
  double kV = 1;
  double kDt = 1;
  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(kP, kI, kD, m_constraints, kDt);
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);
  private DistanceCaching distanceCacheFront, distanceCacheBack;
  private DistanceSide distanceCacheSide;
  private final String loggingPrefix = "subsystems/climb/";
  /** Creates a new Climb. */
  public Climb() {
    motor = new TalonFX(Motors.climbId, canbus);
    // m_encoder = new Encoder(Ports.ElevatorEncoderPort1, Ports.ElevatorEncoderPort2);
    // m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    // resetEncoder();
    distanceCacheFront =
        new DistanceCaching(
            Sensors.Distance.frontLeftId,
            Sensors.Distance.frontRightId,
            Sensors.Distance.xRobotOffsetFront,
            "front");
    distanceCacheBack =
        new DistanceCaching(
            Sensors.Distance.backLeftId,
            Sensors.Distance.backRightId,
            Sensors.Distance.xRobotOffsetBack,
            "back");
    distanceCacheSide = new DistanceSide();
  }

  public double getEncoderDistance() {
    // Logger.recordOutput(loggingPrefix + "encoder", -m_encoder.getDistance());
    return 0; // -m_encoder.getDistance();
  }

  public void setGoal(double goal) {
    m_controller.setGoal(goal);
    this.goal = goal;
    Logger.recordOutput(loggingPrefix + "goal", goal);
  }

  public double getGoal() {
    return this.goal;
  }

  public void updateMotorOutput() {
    double volts =
        m_controller.calculate(getEncoderDistance())
            + m_feedforward.calculate(m_controller.getSetpoint().velocity);
    volts = MathUtil.clamp(volts, -5, 5); // actually 12
    setVoltage(volts);
  }

  public void setVoltage(double volts) {
    motor.setVoltage(volts);
    Logger.recordOutput(loggingPrefix + "volts", volts);
  }

  public void resetEncoder() {
    // m_encoder.reset();
  }

  public DistanceCaching getDistanceCacheFront() {
    return this.distanceCacheFront;
  }

  public DistanceCaching getDistanceCacheBack() {
    return this.distanceCacheBack;
  }

  public DistanceSide getDistanceSide() {
    return this.distanceCacheSide;
  }

  boolean hasStoppedGrabber = false;

  @Override
  public void periodic() {
    // updateMotorOutput();
    Logger.recordOutput(loggingPrefix + "goal", getGoal());
  }
}
