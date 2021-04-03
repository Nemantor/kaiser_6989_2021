package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber_Elevator_Subsystem;


public class ElevatorArmDownCommand extends CommandBase {

  private final Climber_Elevator_Subsystem climber_Elevator_Subsystem;

  public ElevatorArmDownCommand(Climber_Elevator_Subsystem climber_Elevator_Subsystem) {
    this.climber_Elevator_Subsystem = climber_Elevator_Subsystem;

    addRequirements(climber_Elevator_Subsystem);
  }
  @Override
  public void initialize() {
    climber_Elevator_Subsystem.elevator_Up(0);
  }

  @Override
  public void execute() {
    
  }


  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climber_Elevator_Subsystem.elevator_Stop();

    
  }

}
