package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmExtendSystem;
import ca.team4308.absolutelib.math.DoubleUtils;

public class ArmExtendCommand extends CommandBase {
    private final ArmExtendSystem m_subsystem;
    private final Supplier<Double> control;
    private final PIDController extension_controller = new PIDController(Constants.Config.Arm.ExtensionControl.kP,
        Constants.Config.Arm.ExtensionControl.kI, Constants.Config.Arm.ExtensionControl.kD);
    // Init
    public ArmExtendCommand(ArmExtendSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
        extension_controller.setSetpoint(subsystem.getSensorPosition());
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

        if (!m_subsystem.armExtendBreak.get()) { // If arm is backed all the way in
            m_subsystem.motor2.setSelectedSensorPosition(0);
            extension_controller.setSetpoint(m_subsystem.getSensorPosition());
            if(control > 0) {
                return;
            }
        }
        if (control == 0.0) {
            // stop it at current
            double output = DoubleUtils.clamp(extension_controller.calculate(m_subsystem.getSensorPosition()), -1.0,
                    1.0);
            m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, output);
            return;
        }
        extension_controller.setSetpoint(m_subsystem.getSensorPosition());
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, control);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
}