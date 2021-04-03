package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_Subsystem;

public class IntakeCommand extends CommandBase {

private final Intake_Subsystem intake_Subsystem;

  public IntakeCommand(Intake_Subsystem intake_Subsystem) {
    this.intake_Subsystem = intake_Subsystem;
    addRequirements(intake_Subsystem);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    intake_Subsystem.change_Intake();   

  }


  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    
    
  }

}
