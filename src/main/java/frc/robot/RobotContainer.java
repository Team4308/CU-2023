// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import ca.team4308.absolutelib.control.JoystickHelper;
import ca.team4308.absolutelib.control.XBoxWrapper;
import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;
import ca.team4308.absolutelib.wrapper.LogSubsystem;

import edu.wpi.first.wpilibj2.command.RepeatCommand;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.HoldInPlace;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.LeftHoldInPlace;
import frc.robot.commands.ArmRotateCommand;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.ArmExtendCommand;
import frc.robot.commands.AimCommand;
import frc.robot.commands.PipelineCommand;
import frc.robot.commands.RightHoldInPlace;
import frc.robot.subsystems.ArmRotateSystem;
import frc.robot.subsystems.ClawSystem;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.LEDSystem;
import frc.robot.subsystems.LimelightSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.commands.auto.TurnDistance;
import frc.robot.commands.auto.groups.PreloadDock;
import frc.robot.commands.auto.groups.PreloadDockBackward;
import frc.robot.commands.auto.groups.PreloadDockHigh;
import frc.robot.commands.auto.groups.PreloadMobDock;
import frc.robot.commands.auto.groups.PreloadMob;
import frc.robot.commands.auto.groups.PreloadMobHigh;
import frc.robot.commands.auto.groups.PreloadMobHighStart;
import frc.robot.commands.auto.groups.Basic;
import frc.robot.commands.auto.groups.DockOnly;
import frc.robot.commands.auto.groups.DockOnlyArmPreload;
import frc.robot.commands.auto.groups.DockOnlyBumpPreload;

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
    private final ClawSystem m_clawSystem;
    private final LimelightSystem m_limelightSystem;
    private final LEDSystem m_ledSystem;

    // Commands
    private final DriveCommand driveCommand;
    private final ArmRotateCommand armRotateCommand;
    private final ArmExtendCommand armExtendCommand;
    private final LEDCommand ledCommand;

    // Controllers
    public final XBoxWrapper stick = new XBoxWrapper(0);
    public final XBoxWrapper stick2 = new XBoxWrapper(1);
    private Integer humanMode = 0;

    // Auto
    private final SendableChooser<Command> autoCommandChooser = new SendableChooser<Command>();

    private final PreloadDock preloadDock;
    private final PreloadDockBackward preloadDockBackward;
    private final PreloadMobDock preloadMobDock;
    private final PreloadMob preloadMob;
    private final PreloadDockHigh preloadDockHigh;
    private final Basic basic;
    private final DockOnly dockOnly;
    private final DockOnlyBumpPreload dockBumpPreload;
    private final DockOnlyArmPreload dockOnlyArmPreload;
    private final PreloadMobHigh preloadMobHigh;
    private final PreloadMobHighStart preloadMobHighStart;
    public Boolean armOut = false;

    public RobotContainer() {

        m_driveSystem = new DriveSystem();
        subsystems.add(m_driveSystem);
        m_armRotateSystem = new ArmRotateSystem();
        subsystems.add(m_armRotateSystem);
        m_armExtendSystem = new ArmExtendSystem();
        subsystems.add(m_armExtendSystem);
        m_clawSystem = new ClawSystem();
        subsystems.add(m_clawSystem);
        m_limelightSystem = new LimelightSystem();
        subsystems.add(m_limelightSystem);
        m_ledSystem = new LEDSystem();
        subsystems.add(m_ledSystem);

        driveCommand = new DriveCommand(m_driveSystem, () -> getDriveControl());
        m_driveSystem.setDefaultCommand(driveCommand);

        armRotateCommand = new ArmRotateCommand(m_armRotateSystem, () -> getArmRotateControl());
        m_armRotateSystem.setDefaultCommand(armRotateCommand);

        armExtendCommand = new ArmExtendCommand(m_armExtendSystem, () -> getArmExtendControl());
        m_armExtendSystem.setDefaultCommand(armExtendCommand);

        ledCommand = new LEDCommand(m_ledSystem, () -> getLEDCommand());
        m_ledSystem.setDefaultCommand(ledCommand);

        // Auto

        preloadDock = new PreloadDock(m_driveSystem, m_armExtendSystem, m_armRotateSystem, m_clawSystem);
        preloadDockBackward = new PreloadDockBackward(m_driveSystem, m_armExtendSystem, m_armRotateSystem,
                m_clawSystem);
        preloadMobDock = new PreloadMobDock(m_driveSystem, m_armExtendSystem, m_armRotateSystem, m_clawSystem);
        preloadDockHigh = new PreloadDockHigh(m_driveSystem, m_armExtendSystem, m_armRotateSystem, m_clawSystem);
        preloadMob = new PreloadMob(m_driveSystem, m_armExtendSystem, m_armRotateSystem, m_clawSystem, armOut);
        basic = new Basic(m_driveSystem, m_clawSystem);
        dockOnly = new DockOnly(m_driveSystem, m_clawSystem);
        dockBumpPreload = new DockOnlyBumpPreload(m_driveSystem, m_clawSystem);
        dockOnlyArmPreload = new DockOnlyArmPreload(m_driveSystem, m_clawSystem, m_armRotateSystem);
        preloadMobHigh = new PreloadMobHigh(m_driveSystem, m_armExtendSystem, m_armRotateSystem, m_clawSystem);
        preloadMobHighStart = new PreloadMobHighStart(m_driveSystem, m_armExtendSystem, m_armRotateSystem,
                m_clawSystem);

        autoCommandChooser.setDefaultOption("Score & Dock", preloadDock);
        autoCommandChooser.setDefaultOption("Score & Dock Backwards", preloadDockBackward);

        autoCommandChooser.addOption("Score, Mobility, Dock", preloadMobDock);
        autoCommandChooser.addOption("Preloaod + Mobility (mid)", preloadMob);

        autoCommandChooser.addOption("High Pre-Load + dock", preloadDockHigh);

        autoCommandChooser.addOption("Gyro Turning", basic);
        autoCommandChooser.addOption("Dock Only", dockOnly);
        autoCommandChooser.addOption("Dock Only w/ Backwards Preload", dockBumpPreload);
        autoCommandChooser.addOption("Dock Only w/ Arm Preload", dockOnlyArmPreload);

        autoCommandChooser.addOption("Pre-load + Mobility (High)", preloadMobHigh);
        autoCommandChooser.addOption("High Preload without Drive Forward", preloadMobHighStart);

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

        // stick.B.whileTrue(new RepeatCommand(new InstantCommand(() ->
        // m_driveSystem.BBAlign(), m_driveSystem)));

        // Limelight Functions
        stick.A.whileTrue(new AimCommand(m_driveSystem, () -> getAimCommand()));
        // stick.X.whileTrue(new PipelineCommand(m_limelightSystem));
        stick.Y.onTrue(new InstantCommand(() -> m_limelightSystem.toggleCamera(), m_limelightSystem));
        // stick.RB.onTrue(new InstantCommand(() -> toggleBrakeMode()));
        // stick.LB.whileTrue(new DockingCommand(m_driveSystem));
        // stick.RB.onTrue(new InstantCommand(() -> m_driveSystem.resetAngle(),
        // m_driveSystem));
        stick.Start.onTrue(new InstantCommand(() -> m_driveSystem.resetAngle(), m_driveSystem));
        stick.LB.whileTrue(new HoldInPlace(m_driveSystem, () -> getHoldControl()));

        // LEFT RIGHT TURNS
        // stick.X.whileTrue(new TurnDistance(0.5,-0.5, m_driveSystem));
        // stick.B.whileTrue(new TurnAngle(-90, m_driveSystem));

        // Controller #1

        // Pneumatic Claw
        stick2.LB.onTrue(new InstantCommand(() -> m_clawSystem.toggle(), m_clawSystem));
        stick2.RB.whileTrue(new RepeatCommand(new InstantCommand(() -> m_clawSystem.BBclose(), m_clawSystem)));

        // Arm Auto-Position

        // Set Middle
        stick2.B.whileTrue(new ArmRotate(16000, m_armRotateSystem));
        // Middle Node
        stick2.X.whileTrue(new ArmRotate(30000, m_armRotateSystem));
        // Human Player Lineup
        stick2.A.whileTrue(new ArmRotate(250000, m_armRotateSystem));

        stick2.Start.onTrue(new InstantCommand(() -> playerWantMode()));

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
        control = JoystickHelper.precisionScaleStick(control, Constants.Config.Input.Stick.kInputScale, 0.6);
        control = JoystickHelper.clampStick(control);

        return control;
    }

    public Vector2 getHoldControl() {
        Vector2 control = new Vector2(m_driveSystem.masterLeft.getSelectedSensorPosition(),
                m_driveSystem.masterRight.getSelectedSensorPosition());
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
        double y = (DoubleUtils.normalize(stick2.getRightY())) * -0.35;
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

    public Integer getLEDCommand() {
        Value clawState = m_clawSystem.solenoid1.get(); // kForward or kReverse or kOff
        // Boolean armExtended = m_armExtendSystem.checkIfExtend();
        if (RobotController.getBatteryVoltage() <= 10.00)
            return 7; // low voltage
        if (humanMode == 1) {
            return 3;
        }
        if (humanMode == 2) {
            return 4;
        }

        if (clawState == Value.kReverse)
            return 1;
        return 8;
    }

    public void playerWantMode() {
        humanMode++;
        if (humanMode == 3) {
            humanMode = 0;
        }
    }

    public Command getAutonomousCommand() {
        return autoCommandChooser.getSelected();
    }

}
