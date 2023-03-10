package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class BBAlignCommand {
    private final DriveSystem m_subsystem;

    // Init
    public BBAlignCommand(DriveSystem subsystem) {
        m_subsystem = subsystem;
    }

    // Called when the command is initially scheduled.
    public void initialize() {
        m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute() {
        Boolean leftLineBreak = DriveSystem.leftLineBreak.get();
        Boolean rightLineBreak = DriveSystem.rightLineBreak.get();
        double output = 0.01;

        if (!(!leftLineBreak == true && !rightLineBreak == true)) {
            if (!leftLineBreak == true) {
                m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), -output, output);
            }
            if (!rightLineBreak == true) {
                m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), output, -output);
            }
        }
    }
    
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
    
}
