package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotateSystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmRotateHold extends CommandBase {
    private final ArmRotateSystem m_subsystem;
    private final double encoderDistance;

    private final PIDController angle_controller = new PIDController(Constants.Config.Arm.AutoAngleControlHold.kP,
            Constants.Config.Arm.AutoAngleControlHold.kI, Constants.Config.Arm.AutoAngleControlHold.kD);

    public ArmRotateHold(double encoderDistance, ArmRotateSystem subsystem) {
        m_subsystem = subsystem;
        this.encoderDistance = encoderDistance;
        addRequirements(this.m_subsystem);
        angle_controller.setSetpoint(encoderDistance);
    }

    @Override
    public void initialize() {
        m_subsystem.motor1.setNeutralMode(NeutralMode.Brake);
        angle_controller.setSetpoint(encoderDistance);
    }

    @Override
    public void execute() {
        double output = DoubleUtils.clamp(angle_controller.calculate(m_subsystem.getArmPosition()), -1.0, 1.0);

        m_subsystem.setArmOutput(TalonSRXControlMode.PercentOutput, output * 0.3);
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
