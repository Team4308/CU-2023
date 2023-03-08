package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

        if (!leftLineBreak == true && rightLineBreak == true) {
            if (leftLineBreak == true) {
                double output = 0.01;
                m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), -output, output);
            }
            if (rightLineBreak == true) {
                double output = 0.01;
                m_subsystem.setMotorOutput(TalonFXControlMode.PercentOutput.toControlMode(), output, -output);
            }
        }
    }
    
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }
    
}
