
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;


public class Indexer_Subsystem extends SubsystemBase {

    private final WPI_TalonSRX m_indexer_Motor = new WPI_TalonSRX(IndexerConstants.kIndexer_Motor_Port);

  public Indexer_Subsystem() {

  }

  @Override
  public void periodic() {
  }

  public void start_Indexer(boolean shooter_state) {
    if(shooter_state)m_indexer_Motor.set(IndexerConstants.kIndexer_Motor_speed);
  }

  public void stop_Indexer() {
    m_indexer_Motor.set(0);
  }

  public void start_Indexer_auto(){
    m_indexer_Motor.set(0.6);

  }
  public void stop_Indexer_auto(){
    m_indexer_Motor.set(0);

  }

}