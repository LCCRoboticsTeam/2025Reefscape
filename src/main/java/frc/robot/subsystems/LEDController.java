// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.LEDConstants;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDController extends SubsystemBase {

  private Spark m_ledController = new Spark(LEDConstants.PWM_PORT);
  private double m_color = LEDConstants.COLOR_WAVES_OCEAN_PALETTE;

  private BooleanSupplier m_isRobotEnabled;
  private BooleanSupplier m_isCoralLoaded;
  private BooleanSupplier m_isAlgaeLoaded;

  public LEDController(BooleanSupplier isRobotEnabled, BooleanSupplier isCoralLoaded, BooleanSupplier isAlgaeLoaded) {
    m_isRobotEnabled = isRobotEnabled;
    m_isCoralLoaded = isCoralLoaded;
    m_isAlgaeLoaded = isAlgaeLoaded;

    m_ledController.set(m_color);
  }

  @Override
  public void periodic() {
    if (m_isCoralLoaded.getAsBoolean()) 
      m_color=LEDConstants.SOLID_AQUA_BLUE;
    else if (m_isAlgaeLoaded.getAsBoolean())
      m_color = LEDConstants.SOLID_GREEN;
    else if (m_isRobotEnabled.getAsBoolean())
      m_color = LEDConstants.BEATS_PER_MIN_OCEAN_PALETTE;
    else
      m_color = LEDConstants.COLOR_WAVES_OCEAN_PALETTE;

    m_ledController.set(m_color);
  }

}
