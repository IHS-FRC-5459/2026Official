// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSiD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Colors;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {
  private final int rainbowFramerate = 100;
  private final int startIndex = 0;
  private final int endIndex = 53 + 8; // 8 is from the 8 on the candle itself
  private boolean isBlinking = false;
  private long lastTransition = 0;
  private boolean isOn = true;
  // status booleans
  private boolean intaking,
      shooting,
      elevatorGoingUp,
      elevatorGoingDown,
      passing,
      canShoot,
      isAngry,
      isHappy,
      isRainbow,
      isBeltBack;
  private CANdle candle;
  private final String loggingPrefix = "subsystems/led/";

  public LED(CANdle candle) {
    this.candle = candle;
    setLEDs(Colors.blank);
  }

  // sets the colors of an LED
  private void setLEDs(RGBWColor color) {
    SolidColor solidColor = new SolidColor(startIndex, endIndex);
    solidColor = solidColor.withColor(color);
    candle.setControl(solidColor);
    this.isBlinking = false;
  }

  private void setLEDs(RGBWColor color, boolean isBlinking) {
    SolidColor solidColor = new SolidColor(startIndex, endIndex);
    solidColor = solidColor.withColor(color);
    candle.setControl(solidColor);
    this.isBlinking = isBlinking;
  }

  private void doRainbow() { // This works, but it messes up other stuff
    // RainbowAnimation animation = new RainbowAnimation(startIndex, endIndex);
    // animation.withFrameRate(rainbowFramerate);
    // candle.setControl(animation);
    System.out.println("Doing rainbow");
  }

  public void setIntaking(boolean val) {
    intaking = val;
  }

  public void setShooting(boolean val) {
    shooting = val;
  }

  public void setElevatorGoingUp(boolean val) {
    elevatorGoingUp = val;
  }

  public void setElevatorGoingDown(boolean val) {
    elevatorGoingDown = val;
  }

  public void setPassing(boolean val) {
    passing = val;
  }

  public void setCanShoot(boolean val) {
    canShoot = val;
  }

  public void setIsAngry(boolean val) {
    isAngry = val;
    System.out.println("setting is angry");
  }

  public void setIsHappy(boolean val) {
    isHappy = val;
  }

  public void setIsRainbow(boolean val) {
    isRainbow = val;
  }

  public void setBeltBack(boolean val) {
    isBeltBack = val;
  }

  private void updateLEDs() {
    candle.clearStickyFaults();
    // This is just an exaple cascenario
    if (shooting) {
      setLEDs(Colors.green, true);
      Logger.recordOutput(loggingPrefix + "led", "greenBlink");
      Logger.recordOutput(loggingPrefix + "state", "shooting");
    } else if (elevatorGoingUp) {
      setLEDs(Colors.purple, true);
      Logger.recordOutput(loggingPrefix + "led", "purpleBlink");
      Logger.recordOutput(loggingPrefix + "state", "elevatorGoingUp");
    } else if (elevatorGoingDown) {
      setLEDs(Colors.red, true);
      Logger.recordOutput(loggingPrefix + "led", "redBlink");
      Logger.recordOutput(loggingPrefix + "state", "elevatorGoingDown");
    } else if (passing) {
      setLEDs(Colors.blue, true);
      Logger.recordOutput(loggingPrefix + "led", "blueBlink");
      Logger.recordOutput(loggingPrefix + "state", "passing");
    } else if (intaking) {
      setLEDs(Colors.yellow, true);
      Logger.recordOutput(loggingPrefix + "led", "yellowBlink");
      Logger.recordOutput(loggingPrefix + "state", "intaking");
    } else if (isBeltBack) {
      setLEDs(Colors.purple, true);
    } else if (canShoot) {
      setLEDs(Colors.green);
      Logger.recordOutput(loggingPrefix + "led", "green");
      Logger.recordOutput(loggingPrefix + "state", "canShot");
    } else if (isAngry) {
      setLEDs(Colors.red, true);
      Logger.recordOutput(loggingPrefix + "led", "red");
      Logger.recordOutput(loggingPrefix + "state", "Angry");
    } else if (isHappy) {
      setLEDs(Colors.yellow, true);
      Logger.recordOutput(loggingPrefix + "led", "yellow");
      Logger.recordOutput(loggingPrefix + "state", "Happy");
    } else if (isRainbow) {
      doRainbow();
      Logger.recordOutput(loggingPrefix + "led", "rainbow");
      Logger.recordOutput(loggingPrefix + "state", "rainbow");
    } else {
      // setLEDs(Colors.blank);
      candle.setControl(new EmptyControl());

      Logger.recordOutput(loggingPrefix + "led", "blank");
      Logger.recordOutput(loggingPrefix + "state", "nothing");
    }
    long currTime = System.currentTimeMillis();
    if (currTime - lastTransition >= 1000) {
      isOn = !isOn;
      lastTransition = currTime;
    }
    if (!isOn && isBlinking) {
      setLEDs(Colors.blank);
    }
  }

  // @Override
  public void periodic() {
    updateLEDs();
    // int[] color = Constants.Colors.red;
    // setLED(color);
  }
}
