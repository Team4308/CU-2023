package frc.robot.commands.auton;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import ca.team4308.absolutelib.math.DoubleUtils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSystem;

public class SoloBalance extends CommandBase {
    private DriveSystem m_subsystem;
    private final PIDController pitchController = new PIDController(Constants.Config.Drive.PitchControl.kP,
            Constants.Config.Drive.PitchControl.kI, Constants.Config.Drive.PitchControl.kD);

    int withinThresholdLoops = 0;


    public SoloBalance(DriveSystem subsystem) {
        this.m_subsystem = subsystem;
        withinThresholdLoops = 0;
        addRequirements(this.m_subsystem);
        pitchController.setSetpoint(0.0);
        pitchController.setTolerance(Constants.Config.Drive.PitchControl.kTolerance);
    }

    @Override
    public void initialize() {
        this.m_subsystem.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot);
        this.m_subsystem.stopControllers();
    }

    @Override
    public void execute() {
        
        double roll = DriveSystem.gyro.getAngle();
        double output = DoubleUtils.clamp(pitchController.calculate(roll), -1.0, 1.0);
        m_subsystem.setMotorOutput(TalonSRXControlMode.PercentOutput.toControlMode(), -output, -output);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopControllers();
    }

    @Override
    public boolean isFinished() {
        return (withinThresholdLoops > 5);
    }
}

