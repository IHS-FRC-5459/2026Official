// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  private LED s_led;
  private Flywheel s_flywheel;
  private Indexer s_indexer;
  private Intake s_intake;
  private Pivot s_pivot;
  private Hood s_hood;
  private Drive s_drive;
  private long lastAgitation;
  private final double agitationIntervalTime = 1000;
  private final double indexerVolts = 7;
  private String loggingPrefix = "commands/shoot/";

  // Format: distance from hub(diagonally) in m, optimized hood goal, optimized flywheel goal
  private final double[][] lookupTable = {
    {1.8288, 16.5, 48},
    {2.4384, 21, 50},
    {3.048, 24, 53},
    {3.6576, 29, 51},
    {4.2672, 31, 56},
  };
  /** Creates a new Outtake. */
  public Shoot(
      LED s_led,
      Flywheel s_flywheel,
      Indexer s_indexer,
      Intake s_intake,
      Pivot s_pivot,
      Hood s_hood,
      Drive s_drive) {
    // Only uding drive for pos, so dont add drive req
    addRequirements(s_flywheel, s_pivot, s_hood, s_indexer);
    this.s_flywheel = s_flywheel;
    this.s_indexer = s_indexer;
    this.s_intake = s_intake;
    this.s_pivot = s_pivot;
    this.s_drive = s_drive;
    this.s_led = s_led;
    this.s_hood = s_hood;
  }

  // Called when the command is initially scheduled.
  double beltSpeedTest = 0;

  @Override
  public void initialize() {
    lastAgitation = System.currentTimeMillis();
    s_led.setShooting(true);
    beltSpeedTest += 0.1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Idk whal algothm we're gonna do for calculating flywheel volts or hood angle yet
    Pose2d currPose = s_drive.getPose();
    // Close-side(blue)
    Pose2d hubPose = new Pose2d(Inches.of(182.11), Inches.of(158.84), new Rotation2d(0));
    if (currPose.getX() > aprilTagLayout.getFieldLength() / 2) { // Far-side(red)
      hubPose = new Pose2d(Inches.of(469.11), Inches.of(158.84), new Rotation2d(0));
    }
    // Pythagorean
    double distToHub =
        Math.sqrt(
            Math.pow(Math.abs(hubPose.getX() - currPose.getX()), 2)
                + Math.pow(Math.abs(hubPose.getY() - currPose.getY()), 2));
    distToHub -= 0.34;
    double[] oneCloserVals = lookupTable[0];
    double[] oneFartherVals = lookupTable[lookupTable.length - 1];
    for (int i = 0; i < lookupTable.length; i++) {
      double tableDist = lookupTable[i][0];
      if (distToHub > tableDist && distToHub - tableDist < distToHub - oneCloserVals[0]) {
        oneCloserVals = lookupTable[i];
      } else if (tableDist > distToHub) {
        oneFartherVals = lookupTable[i];
        break;
      }
    }
    double interpolationConst =
        (distToHub - oneCloserVals[0]) / (oneFartherVals[0] - oneCloserVals[0]);
    double interpolatedHoodGoal =
        (oneFartherVals[1] - oneCloserVals[0]) * interpolationConst + oneCloserVals[1];
    double interpolatedFlywheelGoal =
        (oneFartherVals[2] - oneCloserVals[2]) * interpolationConst + oneCloserVals[2];
    Logger.recordOutput(loggingPrefix + "dist", distToHub);
    Logger.recordOutput(loggingPrefix + "interpolationConst", interpolationConst);
    Logger.recordOutput(loggingPrefix + "interpolatedHood", interpolatedHoodGoal);
    Logger.recordOutput(loggingPrefix + "interpolatedFlywheel", interpolatedFlywheelGoal);

    // s_hood.setGoal(interpolatedHoodGoal);
    // s_flywheel.setGoal(interpolatedFlywheelGoal);
    // Agitation
    long timeSinceLastAgitation = System.currentTimeMillis() - lastAgitation;
    // if (timeSinceLastAgitation > agitationIntervalTime) {
    // if (s_pivot.isAtSetpoint()) {
    //   if (timeSinceLastAgitation >= 5000) {
    //     lastAgitation = System.currentTimeMillis();
    //     if (s_pivot.getGoal() == -15) {
    //       s_pivot.setGoal(90);
    //     } else {
    //       s_pivot.setGoal(-15);
    //     }
    //   }
    // }

    s_pivot.setGoal(-15);
    // s_belt.setSpeed(beltPower);

    s_flywheel.setGoal(SmartDashboard.getNumber("flywheelSpeed", 0));
    if (s_flywheel.isAtSetpoint() && s_hood.isAtSetpoint()) {
      s_indexer.setVoltage(indexerVolts);
    } else {
      s_indexer.setVoltage(0);
    }
    s_hood.setGoal(SmartDashboard.getNumber("hoodAngle", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // s_belt.setSpeed(0);
    s_intake.setSpeed(0);
    s_indexer.setVoltage(0);
    s_flywheel.setGoal(0);
    // s_hood.setGoal(13);
    s_led.setShooting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
