package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmExtendSystem;
import ca.team4308.absolutelib.math.DoubleUtils;

public class ArmExtendCommand extends CommandBase {
    private final ArmExtendSystem m_subsystem;
    private final Supplier<Double> control;
    private Double initialValue;
    private final PIDController extension_controller = new PIDController(Constants.Config.Arm.ExtensionControl.kP,
            Constants.Config.Arm.ExtensionControl.kI, Constants.Config.Arm.ExtensionControl.kD);

    // Init
    public ArmExtendCommand(ArmExtendSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);

        extension_controller.setSetpoint(subsystem.getSensorPosition());
        initialValue = subsystem.getSensorPosition();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double control = this.control.get();
        if (m_subsystem.getSensorPosition() <= 100){
            m_subsystem.resetSensors();
            initialValue = (double)0;
        }
        if (control == 0.0) {
            // stop it at current
            extension_controller.setSetpoint(initialValue + 3000);
            double output = DoubleUtils.clamp(extension_controller.calculate(m_subsystem.getSensorPosition()), -1.0,
                    1.0);
            m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, output);
        } else {
            m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, control);
            initialValue = m_subsystem.getSensorPosition();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
}