// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDColorState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

  private double m_Color = LEDConstants.SOLID_BLUE;
  private Spark m_ledController = new Spark(LEDConstants.PWM_PORT);

  public LEDController() {
  }

  @Override
  public void periodic() {
    //m_Color = SmartDashboard.getNumber("LEDController/color", m_Color);
    m_ledController.set(m_Color);
  }

  public void setColor(LEDColorState color) {
    switch(color) {
      case NOTE_LESS:
        m_Color = LEDConstants.SOLID_BLUE;
        break;
      case NOTE_DETECTED:
        m_Color = LEDConstants.SOLID_GREEN;
        break;
      case SHOOTING:
        m_Color = LEDConstants.FIXED_PALETTE_PATTERN_FIRE_LARGE;
        break;
      default:
        m_Color = LEDConstants.SOLID_BLUE;
    }
  }

}
