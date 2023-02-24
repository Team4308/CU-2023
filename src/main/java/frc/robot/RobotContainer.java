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
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.subsystems.ArmRotateSystem;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.DriveSystem;
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
  
  private final DriveCommand driveCommand;
  private final ArmRotateCommand armRotateCommand;
  private final ArmExtendCommand armExtendCommand;

  
  public final XBoxWrapper stick = new XBoxWrapper(0);
  public final XBoxWrapper stick2 = new XBoxWrapper(1);

  public RobotContainer() {
    
    m_driveSystem = new DriveSystem();
    subsystems.add(m_driveSystem);
    m_armSystem = new ArmRotateSystem();
    subsystems.add(m_armSystem);
    m_armExtendSystem = new ArmExtendSystem();
    subsystems.add(m_armExtendSystem);


    driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
    m_driveSystem.setDefaultCommand(driveCommand);

    armRotateCommand = new ArmRotateCommand(m_armSystem, () -> getArmRotateControl());
    m_armSystem.setDefaultCommand(armRotateCommand);

    armExtendCommand = new ArmExtendCommand(m_armExtendSystem, () -> getArmExtendControl());
    m_armExtendSystem.setDefaultCommand(armExtendCommand);

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
  
  // public Command getAutonomousCommand() {

  //  }
}