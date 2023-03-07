package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class RangeCommand extends CommandBase {
    private final DriveSystem m_subsystem;
    private final Supplier<Double> control;

    private final PIDController range_controller = new PIDController(Constants.Config.Drive.RangeControl.kP,
            Constants.Config.Drive.RangeControl.kI, Constants.Config.Drive.RangeControl.kD);

    // Init
    public RangeCommand(DriveSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        range_controller.setSetpoint(0.0);
        range_controller.setTolerance(Constants.Config.Drive.RangeControl.kTolerance);

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double output = DoubleUtils.clamp(range_controller.calculate(control.get()), -1.0, 1.0);
        m_subsystem.setMotorOutput(TalonSRXControlMode.PercentOutput.toControlMode(), -output, -output);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}