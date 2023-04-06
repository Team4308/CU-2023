package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class TurnDistance extends CommandBase {
    private DriveSystem m_subsystem;
    private double metersL;
    private double metersR;

    int withinThresholdLoops;

    public TurnDistance(double metersL, double metersR, DriveSystem subsystem) {
        this.metersL = metersL;
        this.metersR = metersR;
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
        double encoderDistanceL = (Units.metersToInches(this.metersL)
                / Constants.Config.Drive.Kinematics.kEncoderInchesPerCount);
        encoderDistanceL /= Constants.Config.Drive.Kinematics.kGearRatio;

        double encoderDistanceR = (Units.metersToInches(this.metersR)
                / Constants.Config.Drive.Kinematics.kEncoderInchesPerCount);
        encoderDistanceR /= Constants.Config.Drive.Kinematics.kGearRatio;

        m_subsystem.masterLeft.set(TalonFXControlMode.MotionMagic, encoderDistanceL);
        m_subsystem.masterRight.set(TalonFXControlMode.MotionMagic, encoderDistanceR);
        if (m_subsystem.masterLeft.getActiveTrajectoryPosition() < encoderDistanceL + 1
            && m_subsystem.masterLeft.getActiveTrajectoryPosition() > encoderDistanceL - 1
            && m_subsystem.masterRight.getActiveTrajectoryPosition() < encoderDistanceR + 1
            && m_subsystem.masterRight.getActiveTrajectoryPosition() > encoderDistanceR - 1) {
            withinThresholdLoops++;
        } else {
            withinThresholdLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterLeft.setNeutralMode(NeutralMode.Coast);
        this.m_subsystem.masterRight.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}