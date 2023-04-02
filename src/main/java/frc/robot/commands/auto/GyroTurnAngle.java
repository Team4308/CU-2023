package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class GyroTurnAngle extends CommandBase {
    private DriveSystem m_subsystem;
    private double originalAngle;
    private double targetAngle;
    private final PIDController turn_controller = new PIDController(Constants.Config.Drive.GyroTurnControl.kP,
     Constants.Config.Drive.GyroTurnControl.kI, Constants.Config.Drive.GyroTurnControl.kD);
    private double tolerance = Constants.Config.Drive.GyroTurnControl.kTolerance;

    int withinThresholdLoops = 0;

    public GyroTurnAngle(double angle, DriveSystem subsystem) {
        DriverStation.reportWarning("Turning "+angle, null);
        this.m_subsystem = subsystem;
        originalAngle = m_subsystem.gyro.getAngle();
        targetAngle = angle + originalAngle;
        withinThresholdLoops = 0;
        turn_controller.setSetpoint(targetAngle);
        addRequirements(this.m_subsystem);
    }

    @Override
    public void initialize() {
        m_subsystem.stopControllers();
        this.m_subsystem.masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
        this.m_subsystem.masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);
    }

    @Override
    public void execute() {
        double output = DoubleUtils.clamp(turn_controller.calculate(m_subsystem.gyro.getAngle()), -1.0, 1.0);


        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), output,
                -output);

        if (Math.abs(m_subsystem.gyro.getAngle() - targetAngle) < tolerance) {
            withinThresholdLoops += 1;
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
        return (withinThresholdLoops > 10);
    }
}