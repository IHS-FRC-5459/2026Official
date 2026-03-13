// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// Change for github
package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoShootAlign;
import frc.robot.commands.ClimbAlign;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorUp;
import frc.robot.commands.FlywheelSpinUp;
import frc.robot.commands.PassAlign;
import frc.robot.commands.PassShoot;
import frc.robot.commands.Reverse;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
// import frc.robot.commands.ClimbRight;
import frc.robot.commands.ShootAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public Drive drive;
  private Intake s_intake;
  public Climb s_climb;
  public Pivot s_pivot; // On purpose so that we can go down at start of match/auto
  private Indexer s_indexer;
  private Flywheel s_flywheel;
  public Hood s_hood;
  private LED s_led;
  // private final Vision vision
  private Vision vision;
  // Sensors
  private Pigeon2 pigeon;
  private CANdle candle;
  // controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1),
                new VisionIOPhotonVision(camera2Name, robotToCamera2),
                new VisionIOPhotonVision(camera3Name, robotToCamera3));
        pigeon = new Pigeon2(Constants.Sensors.pigeonId, Constants.canbus);
        candle = new CANdle(Constants.Sensors.candleId, Constants.canbus);
        s_led = new LED(candle);
        s_intake = new Intake();
        s_climb = new Climb(s_led);
        s_pivot = new Pivot();
        s_indexer = new Indexer();
        s_flywheel = new Flywheel();
        s_hood = new Hood();
        // vision = new Vision(Constants.Vision.cameraNames, pigeon, drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(camera2Name, robotToCamera2, drive::getPose),
                new VisionIOPhotonVisionSim(camera3Name, robotToCamera3, drive::getPose));

        pigeon = new Pigeon2(Constants.Sensors.pigeonId, Constants.canbus);
        candle = new CANdle(Constants.Sensors.candleId, Constants.canbus);
        s_led = new LED(candle);
        s_intake = new Intake();
        s_climb = new Climb(s_led);
        s_pivot = new Pivot();
        s_indexer = new Indexer();
        s_flywheel = new Flywheel();
        s_hood = new Hood();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        pigeon = new Pigeon2(Constants.Sensors.pigeonId, Constants.canbus);
        candle = new CANdle(Constants.Sensors.candleId, Constants.canbus);
        s_led = new LED(candle);
        s_intake = new Intake();
        s_climb = new Climb(s_led);
        s_pivot = new Pivot();
        s_indexer = new Indexer();
        s_flywheel = new Flywheel();
        s_hood = new Hood();
        break;
    }
    NamedCommands.registerCommand(
        "hi",
        new InstantCommand(
            () -> {
              System.out.println("hi");
            }));
    NamedCommands.registerCommand("climbAlign", new ClimbAlign(drive, s_climb));
    NamedCommands.registerCommand("hubAlign", new AutoShootAlign(drive, s_led, () -> 0, () -> 0));
    NamedCommands.registerCommand("Intake", new RunIntake(s_led, s_intake, s_pivot));
    NamedCommands.registerCommand(
        "Shoot", new Shoot(s_led, s_flywheel, s_indexer, s_intake, s_pivot, s_hood, drive));
    NamedCommands.registerCommand("elevatorDown", new ElevatorDown(s_led, s_climb));
    NamedCommands.registerCommand("elevatorUp", new ElevatorUp(s_led, s_climb));
    NamedCommands.registerCommand(
        "IntakeDown",
        new InstantCommand(
            () -> {
              s_pivot.setGoal(-15);
            }));

    // Set up auto routines
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Y button is pressed
    driver
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    driver
        .rightTrigger(0.2)
        .whileTrue(
            new ShootAlign(drive, s_led, () -> -driver.getLeftY(), () -> -driver.getLeftX()));
    driver.rightBumper().whileTrue(new ClimbAlign(drive, s_climb));
    driver.leftTrigger(0.2).whileTrue(new PassAlign(s_led, drive));
    operator.rightBumper().whileTrue(new ElevatorDown(s_led, s_climb));
    operator.leftBumper().whileTrue(new ElevatorUp(s_led, s_climb));
    operator.leftTrigger(0.2).onTrue(new FlywheelSpinUp(s_flywheel));
    operator
        .rightTrigger(0.2)
        .whileTrue(new Shoot(s_led, s_flywheel, s_indexer, s_intake, s_pivot, s_hood, drive));
    operator
        .b()
        .whileTrue(new PassShoot(s_led, s_flywheel, s_indexer, s_intake, s_pivot, s_hood, drive));
    operator.x().whileTrue(new RunIntake(s_led, s_intake, s_pivot));
    operator
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  s_pivot.setGoal(-15);
                })); // Pivot down
    operator
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  s_pivot.setGoal(90);
                })); // pIOT UP
    operator.back().whileTrue(new Reverse(s_led, s_indexer));
    operator
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  s_flywheel.setGoal(0);
                }));
    // operator.back().onTrue(new InstantCommand()->{
    // s_pivot.resetPID();
    // s_pivot.resetEncoder();
    // }, s_pivot);
    // SmartDashboard.putNumber("yPID_P", 0.6);
    // SmartDashboard.putNumber("yPID_I", 0.03);
    // SmartDashboard.putNumber("yPID_D", 0.1);
    // SmartDashboard.putNumber("yFF_S", 0.1);
    // SmartDashboard.putNumber("xPID_P", 2.5);
    // SmartDashboard.putNumber("xPID_I", 0.01);
    // SmartDashboard.putNumber("xPID_D", 0.03);
    // SmartDashboard.putNumber("xFF_S", 0.12);
    // SmartDashboard.putNumber("omegaPID_P", 2.5);
    // SmartDashboard.putNumber("omgeaPID_I", 0);
    // SmartDashboard.putNumber("omegaPID_D", 0.02);
    // SmartDashboard.putNumber("omegaFF_S", 0.15);
    // SmartDashboard.putBoolean("xEnabled", false);
    // SmartDashboard.putBoolean("yEnabled", false);
    // SmartDashboard.putBoolean("omegaEnabled", false);
    // SmartDashboard.putBoolean("step1Done", true);
    // SmartDashboard.putNumber("stoppingDist", 0.0508);
    // SmartDashboard.putNumber("omegaPID_P", 2.5);
    // SmartDashboard.putNumber("omgeaPID_I", 0);
    // SmartDashboard.putNumber("omegaPID_D", 0.02);
    // SmartDashboard.putNumber("omegaFF_S", 0.15);
    // SmartDashboard.putNumber("climbVolts", 0);
    // SmartDashboard.putNumber("pivotGoal", 0);
    SmartDashboard.putNumber("pivotPID_P", 1);
    SmartDashboard.putNumber("pivotPID_I", 0.1);
    SmartDashboard.putNumber("pivotPID_D", 0);
    SmartDashboard.putNumber("pivotFF_G", 0.7);
    // SmartDashboard.putNumber("hoodPID_P", 0);
    // SmartDashboard.putNumber("hoodPID_I", 0);
    // SmartDashboard.putNumber("hoodPID_D", 0);
    // SmartDashboard.putNumber("hoodFF_G", 0);
    // SmartDashboard.putNumber("hoodGoalTesting", 0);

    SmartDashboard.putNumber("flywheelSpeed", 0);
    SmartDashboard.putNumber("hoodAngle", 0);
    SmartDashboard.putNumber("indexerVolts", 0.7);
    SmartDashboard.putNumber("elevatorGoal", 0);
    SmartDashboard.putBoolean("elevatorManualControl", false);

    // SmartDashboard.putNumber("beltSpeed", 0);
    // SmartDashboard.putNumber("intakeSpeed", 0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
