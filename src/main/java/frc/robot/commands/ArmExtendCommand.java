package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtendSystem;

public class ArmExtendCommand extends CommandBase {
    private final ArmExtendSystem m_subsystem;
    private final Supplier<Double> control;
    

    // Init
    public ArmExtendCommand(ArmExtendSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
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
        if(!control){
            m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, -0.1);
        }
        else{
            m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput, control / 4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}