package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Vision_Subsystem extends SubsystemBase {

    private final WPI_VictorSPX led_driver = new WPI_VictorSPX(40);
    double led_voltage = 0;

  public Vision_Subsystem() {
    led_driver.setInverted(true);
  }

  @Override
  public void periodic() {
    led_driver.setVoltage(led_voltage);
  }

  public void open_light() {
    led_voltage = 0;
    
  }

  public void close_light() {
    led_voltage = 0;
  }
  



}