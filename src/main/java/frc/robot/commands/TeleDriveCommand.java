package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive_Subsystem;


public class TeleDriveCommand extends CommandBase {

  private final XboxController driver_Controller;
  private final Drive_Subsystem drive_Subsystem;


  public TeleDriveCommand(XboxController driver_Controller, Drive_Subsystem drive_Subsystem) {
    this.driver_Controller = driver_Controller;
    this.drive_Subsystem = drive_Subsystem;
    addRequirements(drive_Subsystem);

  }

  @Override
  public void execute() {
    drive_Subsystem.arcadeDrive(getSpeed(), getRotation(),true);
  }

  private double getSpeed() {
    double speed =-driver_Controller.getY(Hand.kLeft);
    return speed;
  }

  private double getRotation() {
    double rotation = driver_Controller.getRawAxis(2);
    return rotation;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

}
