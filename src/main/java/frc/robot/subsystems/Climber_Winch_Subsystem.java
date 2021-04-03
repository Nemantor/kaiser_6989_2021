package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;




public class Climber_Winch_Subsystem extends SubsystemBase {

    private final WPI_VictorSPX m_climber_winch = new WPI_VictorSPX(ClimberConstants.kClimber_Winch_Motor_Port);

    public Climber_Winch_Subsystem() {

    }

    public void periodic() {

    }

    public void winch_Pull() {
        m_climber_winch.set(1);
    }

    public void winch_Release() {
        m_climber_winch.set(-1);
    }

    public void winch_Stop() {
        m_climber_winch.set(0);
    }


}