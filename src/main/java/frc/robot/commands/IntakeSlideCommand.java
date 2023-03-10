package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSlideSystem;

public class IntakeSlideCommand extends CommandBase {
    private final IntakeSlideSystem m_subsystem;
    private final Supplier<Double> control;
    

    // Init
    public IntakeSlideCommand(IntakeSlideSystem subsystem, Supplier<Double> control) {
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
        m_subsystem.setSlideOutput(VictorSPXControlMode.PercentOutput, control*0.5);
 
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}