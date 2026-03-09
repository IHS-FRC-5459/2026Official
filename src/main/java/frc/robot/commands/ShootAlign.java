// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.commands.DriveCommands.joystickDriveAtAngleCustom;
import static frc.robot.commands.DriveCommands.setIsFirstCall;
import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAlign extends Command {
  Drive s_drive;
  DoubleSupplier xSupplier, ySupplier;
  LED s_led;
  private static final String loggingPrefix = "commands/shootAlign/";
  /** Creates a new ShootAlign. */
  public ShootAlign(
      Drive s_drive, LED s_led, DoubleSupplier controllerX, DoubleSupplier controllerY) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Note: I putpoosely didnt put dive in addRequirements because I dont want to lock out Rose
    // from driving translation-wise
    this.s_drive = s_drive;
    this.xSupplier = controllerX;
    this.ySupplier = controllerY;
    this.s_led = s_led;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setIsFirstCall(true);
    s_led.setCanShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("workingggggggggg\n\n\n\n\n\n\n\n");
    // Note: Assumes CCW is posotive andle, CW is negative angle
    Pose2d currPose = s_drive.getPose();
    // Close-side(blue)
    Pose2d hubPose = new Pose2d(Inches.of(182.11), Inches.of(158.84), new Rotation2d(0));
    if (currPose.getX() > aprilTagLayout.getFieldLength() / 2) { // Far-side(red)
      hubPose = new Pose2d(Inches.of(469.11), Inches.of(158.84), new Rotation2d(0));
    }
    Logger.recordOutput(loggingPrefix + "Hubpose", hubPose);
    double deltaX = (currPose.getX() - hubPose.getX());
    double deltaY = currPose.getY() - hubPose.getY();
    Logger.recordOutput(loggingPrefix + "deltaX", deltaX);
    Logger.recordOutput(loggingPrefix + "deltaY", deltaY);
    Rotation2d atanCalc = new Rotation2d(Math.atan(deltaY / deltaX));
    Logger.recordOutput(loggingPrefix + "athancalc", atanCalc.getDegrees());
    // in radians
    double desiredRot = atanCalc.getRadians(); // - currPose.getRotation().getRadians();
    Logger.recordOutput(loggingPrefix + "beforechangingdesiredrot", 180 * desiredRot / Math.PI);
    if (desiredRot < -Math.PI) {
      desiredRot = 2 * Math.PI + desiredRot;
    }
    if (desiredRot > Math.PI) {
      desiredRot = -(2 * Math.PI - desiredRot);
    }
    if (currPose.getX()
        >= aprilTagLayout.getFieldLength() / 2) { // If red alliance, go opposite way
      desiredRot += Math.PI;
    }
    double passingOmega = desiredRot;
    // x and y suppliers to be changed for future
    // Rotation2d desiredRot = new Rotation2d(Degrees.of(15));
    Supplier<Rotation2d> omegaSupplier = () -> new Rotation2d(passingOmega); // desiredRot;
    // DoubleSupplier omegaSupplier = () -> desiredRot.getRadians();
    Logger.recordOutput(loggingPrefix + "DesiredRot", 180 * desiredRot / Math.PI);
    joystickDriveAtAngleCustom(s_drive, xSupplier, ySupplier, omegaSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_led.setCanShoot(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
