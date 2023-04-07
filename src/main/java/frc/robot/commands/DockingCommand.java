package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class DockingCommand extends CommandBase {
    private final DriveSystem m_subsystem;

    private final PIDController pitchController = new PIDController(Constants.Config.Drive.PitchControl.kP,
            Constants.Config.Drive.PitchControl.kI, Constants.Config.Drive.PitchControl.kD);

    // Init
    public DockingCommand(DriveSystem subsystem) {
        this.m_subsystem = subsystem;

        pitchController.setSetpoint(0.0);
        pitchController.setTolerance(Constants.Config.Drive.PitchControl.kTolerance);

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
        pitchController.reset();

        m_subsystem.masterLeft.setNeutralMode(NeutralMode.Brake);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Brake);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double pitch = m_subsystem.gyro.getRoll() - 1.5;

        double output = -DoubleUtils.clamp(pitchController.calculate(pitch), -0.15, 0.15);

        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), output, output);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
