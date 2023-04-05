// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class PWMLEDs extends SubsystemBase {
  private AddressableLED leds1;
  private AddressableLED leds2;
  private AddressableLEDBuffer buffer;
  private Timer timer;

  /** Creates a new PWMLEDs. */
  public PWMLEDs() {
    leds1 = new AddressableLED(LEDConstants.pwmLedPort1);
    leds2 = new AddressableLED(LEDConstants.pwmLedPort2);

    buffer = new AddressableLEDBuffer(60);
    leds1.setLength(buffer.getLength());
    leds2.setLength(buffer.getLength());

    leds1.setData(buffer);
    leds2.setData(buffer);

    leds1.start();
    leds2.start();

    timer = new Timer();
  }

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }

    leds1.setData(buffer);
    leds2.setData(buffer);
  }

  public void setBlack() {
    setColor(0, 0, 0);
  }

  public void setPurple() {
    setColor(255, 0, 255);
  }

  public void setYellow() {
    setColor(255, 255, 0);
  }

  public void blinkPurple() {
    timer.reset();
    timer.start();

    if ((int)(timer.get()*10) % 2 == 0) {
      setBlack();
    } else {
      setPurple();
    }
  }

  public void blinkYellow() {
    timer.reset();
    timer.start();

    if ((int)(timer.get()*10) % 2 == 0) {
      setBlack();
    } else {
      setYellow();
    }
  }

  public void turnOff() {
    setBlack();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
