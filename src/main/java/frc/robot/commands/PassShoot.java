// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.drive.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassShoot extends Command {
  private LED s_led;
  private Flywheel s_flywheel;
  private Indexer s_indexer;
  private Intake s_intake;
  private Pivot s_pivot;
  private Drive s_drive;
  private Hood s_hood;
  private long lastAgitation;
  private final double agitationIntervalTime = 1000;
  private final double beltSpeed = 0.5;
  private final double indexerVolts = 2;
  private final double intakeSpeed = 0.2;
  private final double hoodGoal = 13;
  private final double flywheelGoal = 2;
  /** Creates a new Outtake. */
  public PassShoot(
      LED s_led,
      Flywheel s_flywheel,
      Indexer s_indexer,
      Intake s_intake,
      Pivot s_pivot,
      Hood s_hood,
      Drive s_drive) {
    // Only uding drive for pos, so dont add drive req
    addRequirements(s_flywheel);
    this.s_flywheel = s_flywheel;
    this.s_indexer = s_indexer;
    this.s_intake = s_intake;
    this.s_pivot = s_pivot;
    this.s_drive = s_drive;
    this.s_led = s_led;
    this.s_hood = s_hood;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastAgitation = System.currentTimeMillis();
    s_led.setShooting(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Idk whal algothm we're gonna do for calculating flywheel volts or hood angle yet
    // Agitation
    long timeSinceLastAgitation = System.currentTimeMillis() - lastAgitation;
    if (timeSinceLastAgitation > agitationIntervalTime) {
      lastAgitation = System.currentTimeMillis();
      s_pivot.goOpposite();
    }
    s_intake.setSpeed(intakeSpeed);
    s_indexer.setVoltage(indexerVolts);
    s_hood.setGoal(hoodGoal);
    s_flywheel.setGoal(flywheelGoal);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_pivot.goDown();
    s_intake.setSpeed(0);
    s_indexer.setVoltage(0);
    s_led.setShooting(false);
    s_flywheel.setGoal(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
