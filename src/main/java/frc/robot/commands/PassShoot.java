// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassShoot extends Command {
  private Flywheel s_flywheel;
  private Indexer s_indexer;
  private Drive s_drive;
  private Hood s_hood;
  private long lastAgitation;
  private final double agitationIntervalTime = 1000;
  private final double indexerVolts = 12;
  private final double hoodGoal = 38;
  private final double flywheelGoal = 60; // 130
  private final int maxNumChanges = 6;
  private long startOfCommand = 0;
  private String loggingPrefix = "commands/pass/";

  /** Creates a new Outtake. */
  public PassShoot(Flywheel s_flywheel, Indexer s_indexer, Hood s_hood, Drive s_drive) {
    // Only uding drive for pos, so dont add drive req
    addRequirements(s_flywheel);
    this.s_flywheel = s_flywheel;
    this.s_indexer = s_indexer;
    this.s_drive = s_drive;
    this.s_hood = s_hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastAgitation = System.currentTimeMillis();
    numChanges = 0;
    startOfCommand = System.currentTimeMillis();
  }

  int numChanges = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Idk whal algothm we're gonna do for calculating flywheel volts or hood angle yet
    // Agitation
    // long timeSinceLastAgitation = System.currentTimeMillis() - lastAgitation;
    // Logger.recordOutput(loggingPrefix + "timeSinceLastAgitation", timeSinceLastAgitation);
    // if (System.currentTimeMillis() - startOfCommand > 500) {
    //   if (timeSinceLastAgitation > agitationIntervalTime) {
    //     numChanges++;
    //     lastAgitation = System.currentTimeMillis();
    //     if (numChanges == maxNumChanges) {
    //       // s_pivot.setGoal(90);
    //       s_intake.setSpeed(0);
    //       if (!s_pivot.isAtSetpoint()) {
    //         numChanges--;
    //       }
    //       Logger.recordOutput(loggingPrefix + "changing", "allUp");
    //     } else if (numChanges >= maxNumChanges + 1) {
    //       s_pivot.setGoal(-15);
    //       Logger.recordOutput(loggingPrefix + "changing", "allUp");
    //     } else if (s_pivot.getGoal() == -15) {
    //       s_pivot.setGoal(50);
    //       Logger.recordOutput(loggingPrefix + "changing", "down");
    //     } else {
    //       s_pivot.setGoal(-15);
    //       Logger.recordOutput(loggingPrefix + "changing", "up");
    //     }
    //   }
    // }

    if (s_flywheel.isAtSetpoint() && s_hood.isAtSetpoint()) {
      s_indexer.setVoltage(indexerVolts);
    } else {
      s_indexer.setVoltage(0);
    }

    // if (System.currentTimeMillis() - startOfCommand > 2000 && numChanges < maxNumChanges) {
    //   s_intake.setSpeed(0.2);
    // } else {
    //   s_intake.setSpeed(0);
    // }
    s_hood.setGoal(hoodGoal);
    s_flywheel.setGoal(flywheelGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_indexer.setVoltage(0);
    s_flywheel.setGoal(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
