package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber_Winch_Subsystem;;


public class ClimbDownCommand extends CommandBase {

  private final Climber_Winch_Subsystem climber_Winch_Subsystem;

  public ClimbDownCommand(Climber_Winch_Subsystem climber_Winch_Subsystem) {
    this.climber_Winch_Subsystem = climber_Winch_Subsystem;

    addRequirements(climber_Winch_Subsystem);
  }
  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    climber_Winch_Subsystem.winch_Release();

  }


  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
      climber_Winch_Subsystem.winch_Stop();
    
  }

}
