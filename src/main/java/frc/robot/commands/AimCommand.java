package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class AimCommand extends CommandBase {
    private final DriveSystem m_subsystem;
    private final Supplier<Double> control;

    private final PIDController angle_controller = new PIDController(Constants.Config.Drive.AngleControl.kP,
            Constants.Config.Drive.AngleControl.kI, Constants.Config.Drive.AngleControl.kD);

    // Init
    public AimCommand(DriveSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        angle_controller.setSetpoint(0.0);
        angle_controller.setTolerance(Constants.Config.Drive.AngleControl.kTolerance);

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
        double output = DoubleUtils.clamp(angle_controller.calculate(control.get()), -1.0, 1.0);
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), -output, output);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}