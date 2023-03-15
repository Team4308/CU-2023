package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotateSystem;

public class ArmRotateCommand extends CommandBase {
    private final ArmRotateSystem m_subsystem;
    private final Supplier<Double> control;
    private Double initialValue;

    private final PIDController angle_controller = new PIDController(Constants.Config.Arm.AngleControl.kP,
            Constants.Config.Arm.AngleControl.kI, Constants.Config.Arm.AngleControl.kD);

    // Init
    public ArmRotateCommand(ArmRotateSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;

        angle_controller.setSetpoint(subsystem.getArmPosition());
        initialValue = subsystem.getArmPosition();

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
        angle_controller.setSetpoint(m_subsystem.getArmPosition());
        initialValue = m_subsystem.getArmPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double control = this.control.get();

        angle_controller.setSetpoint(
                DoubleUtils.clamp(angle_controller.getSetpoint() + control * 1000, initialValue, initialValue + 40000));
        double output = DoubleUtils.clamp(angle_controller.calculate(m_subsystem.getArmPosition()), -1.0, 1.0);

        m_subsystem.setArmOutput(TalonSRXControlMode.PercentOutput, output);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}