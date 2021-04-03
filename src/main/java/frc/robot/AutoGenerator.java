
package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class AutoGenerator {
  private final Drive_Subsystem m_drive;
  private final Shooter_Subsystem m_shooter;
  private final Indexer_Subsystem m_indexer;
  private final Intake_Subsystem m_intake;


  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public AutoGenerator(Drive_Subsystem m_drive, Shooter_Subsystem m_shooter, Indexer_Subsystem m_indexer, Intake_Subsystem m_intake) {
    this.m_drive = m_drive;
    this.m_shooter = m_shooter;
    this.m_intake = m_intake;
    this.m_indexer = m_indexer;
  }

  public void configureAutonomous() {
    configureCenter3BallAuto();
    configureTrenchSteal();
    configurepathfromJSON();
  }

  public void addDashboardWidgets(ShuffleboardTab dashboard) {
    dashboard.add("Auto Chooser", autoChooser).withSize(2, 1).withPosition(0, 0);
  }

  private void configureCenter3BallAuto() {
      var autoCommandGroup = makeCenter3BallAuto();
      autoChooser.setDefaultOption("Center 3 Ball", autoCommandGroup);
  }

  private void configureTrenchSteal() {
      var autoCommandGroup = steal_trench();
      autoChooser.addOption("Trench Steal", autoCommandGroup);

  }

  private void configurepathfromJSON() {
    var autoCommandGroup = pathfromJSON();
    autoChooser.addOption("pathfromJSON", autoCommandGroup);

}

  private Command makeCenter3BallAuto() {

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kS,
                                   DriveConstants.kV,
                                   DriveConstants.kA),
        Constants.DriveConstants.kDrive_Kinematics,
        10);

    TrajectoryConfig config = 
    new TrajectoryConfig(DriveConstants.kMax_Speed_Meters_Per_Second,
                         DriveConstants.kMax_Acceleration_Meters_Per_Second_Squared)
        .setKinematics(DriveConstants.kDrive_Kinematics)
        .addConstraint(autoVoltageConstraint)
        .setStartVelocity(0)
        .setEndVelocity(0)
        .setReversed(false)
        ;

       var trajectory = TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)),
          new Pose2d(1, 0, new Rotation2d(0)),
          new Pose2d(1.6,0, new Rotation2d(0))),
          config
    );

    return m_drive.createCommandForTrajectory(trajectory)
    .alongWith(new InstantCommand(m_shooter::shoot_Start, m_shooter)).andThen(new InstantCommand(m_drive::stop, m_drive))    
    .andThen(new IndexCommand(m_indexer, m_shooter, m_intake), new InstantCommand(m_intake::open_Intake, m_intake))
    .withTimeout(5).andThen(new InstantCommand(m_shooter::shoot_Stop, m_shooter));

  }



  private Command steal_trench() {

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.kS,
                                   DriveConstants.kV,
                                   DriveConstants.kA),
        Constants.DriveConstants.kDrive_Kinematics,
        10);

    TrajectoryConfig config = 
    new TrajectoryConfig(DriveConstants.kMax_Speed_Meters_Per_Second,
                         DriveConstants.kMax_Acceleration_Meters_Per_Second_Squared)
        .setKinematics(DriveConstants.kDrive_Kinematics)
        .addConstraint(autoVoltageConstraint)
        .setStartVelocity(0)
        .setEndVelocity(0)
        .setReversed(false)
        .addConstraint(new CentripetalAccelerationConstraint(0.7))
        ;

    TrajectoryConfig r_config = 
     new TrajectoryConfig(DriveConstants.kMax_Speed_Meters_Per_Second,
                         DriveConstants.kMax_Acceleration_Meters_Per_Second_Squared)
        .setKinematics(DriveConstants.kDrive_Kinematics)
        .addConstraint(autoVoltageConstraint)
        .setStartVelocity(0)
        .setEndVelocity(0)
        .setReversed(true)
        .addConstraint(new CentripetalAccelerationConstraint(0.7))
        ;

    var trajectory_straight = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0, new Rotation2d(0))
    , Collections.emptyList()
    , new Pose2d(3.3, 0, new Rotation2d(0))
    , config);

    var trajectory_straight_r = TrajectoryGenerator.generateTrajectory(new Pose2d(3.3, 0, new Rotation2d(0))
    , Collections.emptyList(), new Pose2d(-2, 0, new Rotation2d(0)), r_config);

    var trajectory_curved_final = TrajectoryGenerator.generateTrajectory(new Pose2d(-2, 0, new Rotation2d(0))
    , Collections.emptyList()
    , new Pose2d(-2.3, -4.59, new Rotation2d(-3.12)), config);

    var trajectory_1 = m_drive.createCommandForTrajectory(trajectory_straight).alongWith(new InstantCommand(m_intake::start_Intake, m_intake));
    var trajectory_2 = m_drive.createCommandForTrajectory(trajectory_straight_r);
    var trajectory_3 = m_drive.createCommandForTrajectory(trajectory_curved_final).alongWith(new ShootCommand(m_shooter));

        


return new InstantCommand(m_intake::open_Intake, m_intake)
.andThen(trajectory_1).withTimeout(3)
.andThen(new InstantCommand(m_intake::stop_Intake, m_intake))
.andThen(trajectory_2)
.andThen(trajectory_3)
.andThen(m_indexer::start_Indexer_auto, m_indexer)
.andThen(m_intake::start_Intake, m_intake)
;

  }


  private Command pathfromJSON() {

    String trajectoryJSON = "output/Unnamed_1.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch(IOException ex) {
      DriverStation.reportError("Unable to opne trajectory" + trajectoryJSON, ex.getStackTrace());
    }


    var center_go = m_drive.createCommandForTrajectory(trajectory);

    return center_go

    ;
    

  }





  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}


