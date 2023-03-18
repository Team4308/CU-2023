package frc.robot.commands.auto;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.ArmRotateSystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmRotate extends CommandBase {
    private final ArmRotateSystem m_subsystem;
    private final double encoderDistance;

    private final PIDController angle_controller = new PIDController(Constants.Config.Arm.AutoAngleControl.kP, Constants.Config.Arm.AutoAngleControl.kI, Constants.Config.Arm.AutoAngleControl.kD);

    int withinThresholdLoops;

    public ArmRotate(double encoderDistance, ArmRotateSystem subsystem) {
        m_subsystem = subsystem;
        this.encoderDistance = encoderDistance;

        angle_controller.setSetpoint(encoderDistance);

        withinThresholdLoops = 0;
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
        
        if(m_subsystem.getArmPosition() < encoderDistance + 1000 
            && m_subsystem.getArmPosition() > encoderDistance - 1000)withinThresholdLoops++;
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