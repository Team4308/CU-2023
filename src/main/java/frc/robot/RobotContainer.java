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

import edu.wpi.first.wpilibj2.command.RepeatCommand;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeSlideCommand;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.RangeCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.PipelineCommand;

import frc.robot.subsystems.ArmRotateSystem;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.IntakeSlideSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.LimelightSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.groups.Balance;
import frc.robot.commands.auto.groups.Basic;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public final ArrayList<LogSubsystem> subsystems = new ArrayList<LogSubsystem>();
  private final DriveSystem m_driveSystem;
  private final ArmRotateSystem m_armRotateSystem;
  private final ArmExtendSystem m_armExtendSystem;
  private final IntakeSystem m_intakeSystem;
  private final IntakeSlideSystem m_intakeSlideSystem;
  private final ClawSystem m_clawSystem;
  private final LimelightSystem m_limelightSystem;


  // Commands
  private final DriveCommand driveCommand;
  private final ArmRotateCommand armRotateCommand;
  private final ArmExtendCommand armExtendCommand;
  private final IntakeCommand intakeCommand;
  // private final IntakeSlideCommand intakeSlideCommand;

  // Controllers
  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  // Auto
  private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

  private final Balance balance;
  private final Basic basic;

  public RobotContainer() {

    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);
    m_armRotateSystem = new ArmRotateSystem();
    subsystems.add(m_armRotateSystem);
    m_armExtendSystem = new ArmExtendSystem();
    subsystems.add(m_armExtendSystem);
    m_intakeSystem = new IntakeSystem();
    subsystems.add(m_intakeSystem);
    m_intakeSlideSystem = new IntakeSlideSystem();
    subsystems.add(m_intakeSlideSystem);
    m_clawSystem = new ClawSystem();
    subsystems.add(m_clawSystem);
    m_limelightSystem = new LimelightSystem();
    subsystems.add(m_limelightSystem);

    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
    m_driveSystem.setDefaultCommand(driveCommand);

    armRotateCommand = new ArmRotateCommand(m_armRotateSystem, () -> getArmRotateControl());
    m_armRotateSystem.setDefaultCommand(armRotateCommand);

    armExtendCommand = new ArmExtendCommand(m_armExtendSystem, () -> getArmExtendControl());
    m_armExtendSystem.setDefaultCommand(armExtendCommand);

    intakeCommand = new IntakeCommand(m_intakeSystem, () -> 0.0);
    m_intakeSystem.setDefaultCommand(intakeCommand);

    // intakeSlideCommand = new IntakeSlideCommand(m_intakeSlideSystem, () -> getIntakeSlideControl());
    // m_intakeSlideSystem.setDefaultCommand(intakeSlideCommand);

    // Auto

    balance = new Balance(m_driveSystem);

    basic = new Basic(m_driveSystem);

    autoCommandChooser.setDefaultOption("Dock Immediately", balance);

    autoCommandChooser.addOption("Backwards 0.5m", basic);

    SmartDashboard.putData(autoCommandChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Controller #0

    // Limelight Functions
    stick.A.whileTrue(new RangeCommand(m_driveSystem, () -> getRangeCommand()));
    stick.B.whileTrue(new AimCommand(m_driveSystem, () -> getAimCommand()));
    stick.X.whileTrue(new PipelineCommand(m_limelightSystem));
    stick.Y.onTrue(new InstantCommand(() -> m_limelightSystem.toggleCamera(), m_limelightSystem));
    stick.LB.whileTrue(new DockingCommand(m_driveSystem));
    stick.RB.onTrue(new InstantCommand(() -> m_driveSystem.resetAngle(), m_driveSystem));
    stick.Back.onTrue(new InstantCommand(() -> m_driveSystem.resetSensors(), m_driveSystem));

    // Controller #1

    // Intake
    stick2.Y.whileTrue(new IntakeCommand(m_intakeSystem, () -> 1.0));
    stick2.A.whileTrue(new IntakeCommand(m_intakeSystem, () -> -1.0));
    stick2.B.whileTrue(new IntakeSlideCommand(m_intakeSlideSystem, () -> 1.0));
    stick2.X.whileTrue(new IntakeSlideCommand(m_intakeSlideSystem, () -> -1.0));

    // Pneumatic Claw
    stick2.LB.onTrue(new InstantCommand(() -> m_clawSystem.toggle(), m_clawSystem));
    stick2.RB.whileTrue(new RepeatCommand(new InstantCommand(() -> m_clawSystem.BBclose(), m_clawSystem)));
  

    //Arm Auto-Position

    //High Node
    // stick2.B.onTrue(new ParallelDeadlineGroup(new WaitCommand(2), new ArmRotateCommand(m_armRotateSystem, () -> 0.0),  new ArmExtendCommand(m_armExtendSystem, ()->0.0)));
    //Middle Node
    // stick2.X.onTrue(new ParallelDeadlineGroup(new WaitCommand(2), new ArmRotateCommand(m_armRotateSystem, () -> 0.0),  new ArmExtendCommand(m_armExtendSystem, ()->0.0)));

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

  public Double getArmExtendControl() {
    double y = DoubleUtils.normalize(stick2.getLeftY());
    Vector2 control = new Vector2(0.0, y);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.clampStick(control);
    return control.y;
  }

  public Double getArmRotateControl() {
    double y = (DoubleUtils.normalize(stick2.getRightY()))*-0.45;
    Vector2 control = new Vector2(0.0, y);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.clampStick(control);
    return control.y;
  }

  public Double getIntakeSlideControl() {
    double y = DoubleUtils.normalize(stick2.getRightY());
    Vector2 control = new Vector2(0.0, y);
    control = JoystickHelper.ScaledAxialDeadzone(control, Constants.Config.Input.kInputDeadband);
    control = JoystickHelper.clampStick(control);
    return control.y;
  }

  public Double getRangeCommand() {
    m_limelightSystem.setPipeline(2);
    return m_limelightSystem.getXAngle();
  }

  public Double getAimCommand() {
    return m_limelightSystem.getXAngle();
  }
  
  public Command getAutonomousCommand() {
      return autoCommandChooser.getSelected();
  }


}