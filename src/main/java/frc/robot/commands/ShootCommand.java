package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter_Subsystem;;


public class ShootCommand extends CommandBase {

  private final Shooter_Subsystem shooter_Subsystem;

  public ShootCommand(Shooter_Subsystem shooter_Subsystem) {
    this.shooter_Subsystem = shooter_Subsystem;
    addRequirements(shooter_Subsystem);
  }
  @Override
  public void initialize() {
    shooter_Subsystem.shoot();    
  }

  @Override
  public void execute() {
    
  }


  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    
  }

}
