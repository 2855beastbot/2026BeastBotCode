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
  private AddressableLED LEDstrip = new AddressableLED(0);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(3);
  //private AddressableLEDBufferView left = ledBuffer.createView(24, 43);
  //private AddressableLEDBufferView right = ledBuffer.createView(0, 23);
  
  private LEDPattern currentPattern;
  public LED() {
    LEDstrip.setLength(3);
    LEDstrip.setData(ledBuffer);
    
    setPattern(LEDConstants.breatheYellow);
    LEDstrip.start();
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
    currentPattern.applyTo(ledBuffer);
    LEDConstants.yellow.applyTo(ledBuffer);
    setRGB(255,255,255);
    LEDstrip.setData(ledBuffer);
    // This method will be called once per scheduler run
  }
}
