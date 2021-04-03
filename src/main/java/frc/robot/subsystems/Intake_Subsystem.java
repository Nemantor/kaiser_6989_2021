

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class Intake_Subsystem extends SubsystemBase {

    private final WPI_VictorSPX m_intake_Motor = new WPI_VictorSPX(IntakeConstants.kIntake_Motor_Port);

    private final DoubleSolenoid m_intake_solenoid = 
    new DoubleSolenoid(1, 0);

    private boolean intake_pneumatic_state = false;
    private boolean intake_motor_state = false;


  public Intake_Subsystem() {
    open_Intake();
  }

  @Override
  public void periodic() {
  }

  public void change_Intake() {
    if(!intake_motor_state && intake_pneumatic_state){
    start_Intake();
    }
    else {
      stop_Intake();
    }
  }

  public void change_Pneumatic_Intake() {
    if(intake_pneumatic_state){
      close_Intake();
      }
      else open_Intake();
  }

  public void start_Intake() {
    m_intake_Motor.set(IntakeConstants.kIntake_Motor_speed);
    intake_motor_state = true;
  }

  public void stop_Intake() {
    m_intake_Motor.set(0);
    intake_motor_state = false;
  }

  public void open_Intake() {
   // m_intake_solenoid.set(Value.kForward);
    intake_pneumatic_state = true;
  }

  public void close_Intake() {
    m_intake_solenoid.set(Value.kReverse);
    intake_pneumatic_state = false;
  }


}