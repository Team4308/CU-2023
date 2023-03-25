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

public class HoldInPlace extends CommandBase {
    private final DriveSystem m_subsystem;
    private final Supplier<Vector2> control;
    private final PIDController left_controller = new PIDController(Constants.Config.Drive.HoldInPlace.kP,
            Constants.Config.Drive.HoldInPlace.kI, Constants.Config.Drive.HoldInPlace.kD);
    private final PIDController right_controller = new PIDController(Constants.Config.Drive.HoldInPlace.kP,
            Constants.Config.Drive.HoldInPlace.kI, Constants.Config.Drive.HoldInPlace.kD);

    // Init
    public HoldInPlace(DriveSystem subsystem, Supplier<Vector2> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        m_subsystem.masterLeft.setNeutralMode(NeutralMode.Brake);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Brake);
        left_controller.setSetpoint(control.get().x);
        right_controller.setSetpoint(control.get().y);
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double outputLeft = DoubleUtils.clamp(left_controller.calculate(m_subsystem.masterLeft.getSelectedSensorPosition()), -1.0,
                1.0);
        double outputRight = DoubleUtils.clamp(right_controller.calculate(m_subsystem.masterRight.getSelectedSensorPosition()), -1.0,
                1.0);
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), outputLeft, outputRight);
        
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.masterLeft.setNeutralMode(NeutralMode.Coast);
        m_subsystem.masterRight.setNeutralMode(NeutralMode.Coast);
    }

}