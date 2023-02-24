package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class DockingCommand extends CommandBase {
    private final DriveSystem m_subsystem;
    private final Supplier<Double> control;

    private final PIDController pitchController = new PIDController(Constants.Config.Drive.PitchControl.kP,
            Constants.Config.Drive.PitchControl.kI, Constants.Config.Drive.PitchControl.kD);

    // Init
    public DockingCommand(DriveSystem subsystem, Supplier<Double> control) {
        m_subsystem = subsystem;
        this.control = control;
        pitchController.setSetpoint(0.0);
        pitchController.setTolerance(Constants.Config.Drive.PitchControl.kTolerance);

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        m_subsystem.stopControllers();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
         // 1. get the roll/pitch and read that value in some variable here
        double pitch = frc.robot.subsystems.DriveSystem.getPitch();
        double output = DoubleUtils.clamp(pitchController.calculate(pitch), -1.0, 1.0);
        m_subsystem.setMotorOutput(TalonSRXControlMode.PercentOutput.toControlMode(), -output, -output);

        //3. doa basic pid to move the robot forwards and nbackwards

    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

}