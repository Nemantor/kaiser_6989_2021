/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;


import edu.wpi.first.wpilibj.Compressor;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Compressor c = new Compressor(0);

  public final Drive_Subsystem m_drive = new Drive_Subsystem();
  public final Shooter_Subsystem m_shooter = new Shooter_Subsystem();
  public final Indexer_Subsystem m_indexer = new Indexer_Subsystem();
  public final Intake_Subsystem m_intake = new Intake_Subsystem();
  public final Climber_Elevator_Subsystem m_elevator= new Climber_Elevator_Subsystem();
  public final Climber_Winch_Subsystem m_winch = new Climber_Winch_Subsystem();
  public final Vision_Subsystem m_vision = new Vision_Subsystem();

  private final AutoGenerator autoGenerator = new AutoGenerator(m_drive, m_shooter,
  m_indexer, m_intake);

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  
  public XboxController driver_Controller = new XboxController(0);
  public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");



  public RobotContainer() {
    m_drive.setDefaultCommand(new TeleDriveCommand(driver_Controller, m_drive));
    //m_elevator.setDefaultCommand(new ElevatorCommand(m_elevator, driver_Controller));
    autoGenerator.configureAutonomous();
    configureButtonBindings();
    configureDriverDashboard();
  }


  private void configureButtonBindings() {
    new JoystickButton(driver_Controller, XboxController.Button.kX.value).whenPressed(new ShootCommand(m_shooter));
    new JoystickButton(driver_Controller, XboxController.Button.kBumperLeft.value).whileHeld(new IndexCommand(m_indexer, m_shooter, m_intake));

    new JoystickButton(driver_Controller, XboxController.Button.kA.value).whenPressed(new IntakeCommand(m_intake));
    new JoystickButton(driver_Controller, XboxController.Button.kBumperRight.value).whenPressed(() -> m_intake.change_Pneumatic_Intake());

    new JoystickButton(driver_Controller, 10).whenPressed(() -> m_drive.changeSlowMode());

   // new JoystickButton(driver_Controller,XboxController.Button.kY.value).whileHeld(new ClimbUpCommand(m_winch));
    //new JoystickButton(driver_Controller,XboxController.Button.kB.value).whileHeld(new ClimbDownCommand(m_winch));


  }

  private void configureDriverDashboard() {
    // Auto chooser
    autoGenerator.addDashboardWidgets(driverTab);
  }


  public Command getAutonomousCommand() {
    return autoGenerator.getAutonomousCommand();
  }

}
