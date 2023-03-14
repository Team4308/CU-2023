package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class DriveDistance extends CommandBase {
    private DriveSystem m_subsystem;
    private double meters;

    int withinThresholdLoops;

    public DriveDistance(double meters, DriveSystem subsystem) {
        this.meters = meters;
        this.m_subsystem = subsystem;

        withinThresholdLoops = 0;

        addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.resetSensors();
        m_subsystem.stopControllers();
        m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.MotionMagic.profileSlot, 0);
        m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.MotionMagic.profileSlot, 0);
        m_subsystem.masterLeft.setNeutralMode(NeutralMode.Brake);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void execute() {
        double encoderDistance = (Units.metersToInches(this.meters)
                / Constants.Config.Drive.Kinematics.kEncoderInchesPerCount);
        encoderDistance /= Constants.Config.Drive.Kinematics.kGearRatio;

        m_subsystem.masterLeft.set(TalonFXControlMode.MotionMagic, encoderDistance);
        m_subsystem.masterRight.set(TalonFXControlMode.MotionMagic, encoderDistance);
        if (m_subsystem.masterLeft.getActiveTrajectoryPosition() < encoderDistance + 1
                && m_subsystem.masterLeft.getActiveTrajectoryPosition() > encoderDistance - 1
                && m_subsystem.masterRight.getActiveTrajectoryPosition() < encoderDistance + 1
                && m_subsystem.masterRight.getActiveTrajectoryPosition() > encoderDistance - 1) {
            withinThresholdLoops++;
        } else {
            withinThresholdLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}