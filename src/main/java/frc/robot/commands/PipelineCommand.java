package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSystem;

public class PipelineCommand extends CommandBase {
    private final LimelightSystem m_subsystem;
    public boolean isClicked;

    // Init
    public PipelineCommand(LimelightSystem subsystem) {
        m_subsystem = subsystem;
        addRequirements(m_subsystem);
        isClicked = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        while (!isClicked) {
            double pipeline = m_subsystem.getPipeline();
            pipeline++;
            if (pipeline == 4) {
                pipeline = 0;
                m_subsystem.setPipeline(0);
            } else {
                m_subsystem.setPipeline(pipeline);
            }
            isClicked = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        isClicked = false;
    }
}