package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class DriveTillTipped extends CommandBase {
    private DriveSystem m_subsystem;
    private double angle, speed;

    int withinThresholdLoops = 0;

    public DriveTillTipped(double angle, double speed, DriveSystem subsystem) {
        this.angle = angle;
        this.speed = speed;

        this.m_subsystem = subsystem;

        addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.resetSensors();
        m_subsystem.stopControllers();

        m_subsystem.masterLeft.setNeutralMode(NeutralMode.Brake);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void execute() {
        m_subsystem.masterLeft.set(TalonFXControlMode.PercentOutput, speed);
        m_subsystem.masterRight.set(TalonFXControlMode.PercentOutput, speed);

        if (Math.abs(angle - m_subsystem.gyro.getRoll()) < 5) {
            withinThresholdLoops++;
        } else {
            withinThresholdLoops = 0;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();

        m_subsystem.masterLeft.setNeutralMode(NeutralMode.Coast);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 1);
    }
}
