// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED LEDstrip;
  private AddressableLEDBuffer ledBuffer;
  //private AddressableLEDBufferView left = ledBuffer.createView(24, 43);
  //private AddressableLEDBufferView right = ledBuffer.createView(0, 23);
  
  private LEDPattern currentPattern;
  public LED() {
    LEDstrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(6);
    LEDstrip.setLength(ledBuffer.getLength());
    LEDstrip.setData(ledBuffer);
    setPattern(LEDConstants.yellow);
    LEDstrip.start();
    LEDConstants.yellow.applyTo(ledBuffer);
    LEDstrip.setData(ledBuffer);
  }

  public void setPattern(LEDPattern pattern){
    currentPattern = pattern;
  }

  public void setRGB(int r, int g, int b){
    for(int i = 1; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void periodic() {
    //currentPattern.applyTo(right);
    //currentPattern.applyTo(left);
    currentPattern.applyTo(ledBuffer);
    LEDstrip.setData(ledBuffer);
    // This method will be called once per scheduler run
  }
}
