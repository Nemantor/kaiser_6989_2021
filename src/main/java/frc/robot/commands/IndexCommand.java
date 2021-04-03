package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer_Subsystem;
import frc.robot.subsystems.Intake_Subsystem;
import frc.robot.subsystems.Shooter_Subsystem;


public class IndexCommand extends CommandBase {

  private final Indexer_Subsystem indexer_Subsystem;
  private final Shooter_Subsystem shooter_Subsystem;
  private final Intake_Subsystem intake_Subsystem;

  public IndexCommand(Indexer_Subsystem indexer_Subsystem, Shooter_Subsystem shooter_Subsystem, Intake_Subsystem intake_Subsystem) {
    this.indexer_Subsystem = indexer_Subsystem;
    this.shooter_Subsystem = shooter_Subsystem;
    this.intake_Subsystem = intake_Subsystem;
    addRequirements(indexer_Subsystem);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    indexer_Subsystem.start_Indexer(shooter_Subsystem.shooter_Is_Ready());
    intake_Subsystem.start_Intake();   

  }


  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexer_Subsystem.stop_Indexer();
    intake_Subsystem.stop_Intake();
    
  }

}
