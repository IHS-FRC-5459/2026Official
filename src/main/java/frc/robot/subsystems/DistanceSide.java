// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.LinkedList;
import java.util.Queue;
import org.littletonrobotics.junction.Logger;

public class DistanceSide extends SubsystemBase {
  TimeOfFlight sensor;
  Queue<Double> queue = new LinkedList<>();
  private final int queueSize = 5;
  int sensorId;
  private String loggingPrefix = "subsystems/distanceCaching/side/";
  /** Creates a new DistanceCaching. */
  // offset is the always posotive x translation of the distance sensors to the centerline (x=0
  // line)
  public DistanceSide() {
    sensor = new TimeOfFlight(Constants.Sensors.Distance.climbSideId);
    sensor.setRangingMode(RangingMode.Short, 24);
  }

  // public double getYDistance() {
  //   return getDistanceFiltered() + Constants.Sensors.Distance.yRobotOffsetSide;
  // }

  public TimeOfFlight.Status getStatus() {
    return sensor.getStatus();
  }

  public double getDistanceFiltered() {
    double sumOfDistanceMeasurements = 0;
    int numZeroes = 0;
    for (double distanceMeasurement : queue) {
      if (distanceMeasurement <= 0 || distanceMeasurement >= 1600) {
        numZeroes++;
      } else {
        sumOfDistanceMeasurements += distanceMeasurement;
      }
    }
    if (sumOfDistanceMeasurements <= 0) {
      return -1;
    }
    return (sumOfDistanceMeasurements / (queue.size() - numZeroes)) / 1000;
  }

  public boolean measurementsValid() {
    return getDistanceFiltered() >= 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double rawDistance = sensor.getRange() - 30;
    if (sensor.getStatus() != TimeOfFlight.Status.Valid) {
      rawDistance = -1;
    }
    queue.add(rawDistance);
    if (queue.size() > queueSize) {
      queue.remove();
    }
    Logger.recordOutput(loggingPrefix + "rawdistancesdistsensor", rawDistance);
    Logger.recordOutput(loggingPrefix + "filtered", getDistanceFiltered());
    Logger.recordOutput(loggingPrefix + "getSampletimeleft", sensor.getSampleTime());
    Logger.recordOutput(loggingPrefix + "isValid", measurementsValid());
    Logger.recordOutput(loggingPrefix + "status", getStatus());
  }
}
