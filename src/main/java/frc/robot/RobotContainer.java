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
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeSlideCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.PipelineCommand;

import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.RangeCommand;
import frc.robot.subsystems.ArmRotateSystem;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSlideSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ClawSystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();
  private final DriveSystem m_driveSystem;
  private final ArmRotateSystem m_armSystem;
  private final ArmExtendSystem m_armExtendSystem;
  private final IntakeSystem m_intakeSystem;
  private final IntakeSlideSystem m_intakeSlideSystem;
  private final ClawSystem m_clawSystem;
  
  private final DriveCommand driveCommand;
  private final ArmRotateCommand armRotateCommand;
  private final ArmExtendCommand armExtendCommand;
  private final IntakeCommand intakeCommand;
  private final IntakeSlideCommand intakeSlideCommand;
  private final AimCommand aimCommand;
  private final PipelineCommand pipelineCommand;


  
  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  public RobotContainer() {
    
    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);
    m_armSystem = new ArmRotateSystem();
    subsystems.add(m_armSystem);
    m_armExtendSystem = new ArmExtendSystem();
    subsystems.add(m_armExtendSystem);
    m_intakeSystem = new IntakeSystem();
    subsystems.add(m_intakeSystem);
    m_intakeSlideSystem = new IntakeSlideSystem();
    subsystems.add(m_intakeSlideSystem);
    m_clawSystem = new ClawSystem();
    subsystems.add(m_clawSystem);

    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
    m_driveSystem.setDefaultCommand(driveCommand);

    armRotateCommand = new ArmRotateCommand(m_armSystem, () -> getArmRotateControl());
    m_armSystem.setDefaultCommand(armRotateCommand);

    armExtendCommand = new ArmExtendCommand(m_armExtendSystem, () -> getArmExtendControl());
    m_armExtendSystem.setDefaultCommand(armExtendCommand);

    intakeCommand = new IntakeCommand(m_intakeSystem, () -> 0.0);
    m_intakeSystem.setDefaultCommand(intakeCommand);

    intakeSlideCommand = new IntakeSlideCommand(m_intakeSlideSystem, () -> 0.0);
    m_intakeSlideSystem.setDefaultCommand(intakeSlideCommand);

    pipelineCommand = new PipelineCommand(m_driveSystem, () -> getPipelineCommand());

    aimCommand = new AimCommand(m_driveSystem, () -> getAimCommand());


    

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
    
    stick2.Y.whileTrue(new IntakeCommand(m_intakeSystem, ()-> 1.0));
    stick2.A.whileTrue(new IntakeCommand(m_intakeSystem, ()-> -1.0));
    stick2.LB.onTrue(new InstantCommand(() -> m_clawSystem.extend(), m_clawSystem));
    stick2.RB.onTrue(new InstantCommand(() -> m_clawSystem.retract(), m_clawSystem));

    stick2.X.whileTrue(new IntakeSlideCommand(m_intakeSlideSystem, ()-> -1.0));
    stick2.B.whileTrue(new IntakeSlideCommand(m_intakeSlideSystem, ()-> 1.0));
    stick2.Back.onTrue(new InstantCommand(() -> m_armExtendSystem.resetSensors(), m_armExtendSystem));

    
    stick.A.whileTrue(new RangeCommand(m_driveSystem, () -> getRangeCommand()));
    stick.B.onTrue(new PipelineCommand(m_driveSystem, () -> getPipelineCommand()));
    stick.X.whileTrue(new AimCommand(m_driveSystem, () -> getAimCommand()));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   
   public Vector2 getDriveControl() {
    double throttle = DoubleUtils.normalize(stick.getLeftY());
    double turn = DoubleUtils.normalize(stick.getRightX());

    Vector2 control = new Vector2(turn, throttle);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.scaleStick(control, Constants.Config.Input.Stick.kInputScale);
    control = JoystickHelper.clampStick(control);

    return control;
    }
   
    public Double getArmRotateControl() {
      if(stick2.getLeftTrigger() > 0){
        return stick2.getLeftTrigger()*0.4;
      }else if(stick2.getRightTrigger() > 0){
        return -stick2.getRightTrigger()*0.4;
      }
      return 0.0;
    }

    // if the arm doesn't extend it's probably the getAsBoolean
    // when kevin was making the XBoxWrapper file he accidentally switched trigger and bumper methods or something along those lines
    public Double getArmExtendControl() {
        double y = DoubleUtils.normalize(stick2.getLeftY());
        Vector2 control = new Vector2(0.0, y);
        control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
        control = JoystickHelper.clampStick(control);
        return control.y;
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

    public Double getAimCommand() {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
      double control = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      // System.out.println(control);
      return control;
    }

    public Double getPipelineCommand() {
      System.out.println("e");
      return 0.0;
    }

  // public Command getAutonomousCommand() {

  //  }
}