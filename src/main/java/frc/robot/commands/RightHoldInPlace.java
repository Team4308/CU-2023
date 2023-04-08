package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import ca.team4308.absolutelib.math.Vector2;
import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class RightHoldInPlace extends CommandBase {
    private final DriveSystem m_subsystem;
    private final Supplier<Double> control;
    private final PIDController extension_controller = new PIDController(Constants.Config.Drive.HoldInPlace.kP,
        Constants.Config.Drive.HoldInPlace.kI, Constants.Config.Drive.HoldInPlace.kD);

    // Init
    public RightHoldInPlace(DriveSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        extension_controller.setSetpoint(subsystem.masterRight.getSelectedSensorPosition());
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Brake);
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Double control = this.control.get();
        double output = DoubleUtils.clamp(extension_controller.calculate(m_subsystem.masterLeft.getSelectedSensorPosition()), -1.0,
                1.0);
        m_subsystem.masterRight.set(TalonFXControlMode.PercentOutput, control);

    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Coast);
    }

}