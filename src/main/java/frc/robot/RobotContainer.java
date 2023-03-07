// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.RangeCommand;
import frc.robot.subsystems.DriveSystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();
  private final DriveSystem m_driveSystem;
  
  private final DriveCommand driveCommand;
  

  
  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  public RobotContainer() {
    
    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);


    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
    m_driveSystem.setDefaultCommand(driveCommand);




    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    stick.Y.whileTrue(new DockingCommand(m_driveSystem));
    stick.X.whileTrue(new AimCommand(m_driveSystem, () -> getAimCommand()));
    stick.A.whileTrue(new RangeCommand(m_driveSystem, () -> getRangeCommand()));
    stick.B.onTrue(new InstantCommand(() -> m_driveSystem.setLEDoutput(), m_driveSystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
   public Vector2 getDriveControl() {
    double throttle = DoubleUtils.normalize(stick.getLeftY());
    double turn = DoubleUtils.normalize(-stick.getRightX());

    Vector2 control = new Vector2(turn, throttle);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.scaleStick(control, Constants.Config.Input.Stick.kInputScale);
    control = JoystickHelper.clampStick(control);

    return control;
    }

    public Double getAimCommand() {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      double control = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      // System.out.println(control);
      return control;
    }

    public Double getRangeCommand() {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);

      // angle between limelight and target
      double targetOffsetAngle_Vertical = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

      // angle of elevation of limelight
      double limeLightAngleDegrees = 0.0;

      // vertical height of limelight from ground
      double limeLightHeightCentimetres = 34.3;

      // veritcal height of april tag from ground
      double aprilTagHeightCentimetres = 31.1;

      double angleToAprilTagDegrees = targetOffsetAngle_Vertical + limeLightAngleDegrees;
      double angleToAprilTagRadians = angleToAprilTagDegrees * (Math.PI / 180.0);
      double distanceCentimetres = (aprilTagHeightCentimetres - limeLightHeightCentimetres)/Math.tan(angleToAprilTagRadians);

      return Math.abs(distanceCentimetres);
    }
  // public Command getAutonomousCommand() {

  //  }
}