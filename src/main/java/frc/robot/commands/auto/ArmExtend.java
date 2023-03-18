package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmExtendSystem;
import ca.team4308.absolutelib.math.DoubleUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmExtend extends CommandBase {
    private final ArmExtendSystem m_subsystem;
    private final double encoderDistance;
    private final PIDController extension_controller = new PIDController(Constants.Config.Arm.ExtensionControl.kP, Constants.Config.Arm.ExtensionControl.kI, Constants.Config.Arm.ExtensionControl.kD);

    int withinThresholdLoops;

    public ArmExtend(double encoderDistance, ArmExtendSystem subsystem) {
        m_subsystem = subsystem;
        this.encoderDistance = encoderDistance;
        addRequirements(this.m_subsystem);
        extension_controller.setSetpoint(encoderDistance);
        withinThresholdLoops = 0;
    }

    @Override
    public void initialize() {
        m_subsystem.motor2.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void execute() {
        double output = DoubleUtils.clamp(extension_controller.calculate(m_subsystem.getSensorPosition()), -1.0, 1.0);
        m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, output);

        if(m_subsystem.getSensorPosition() < encoderDistance + 1000 
            && m_subsystem.getSensorPosition() > encoderDistance - 1000)withinThresholdLoops++;
        else withinThresholdLoops = 0;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}