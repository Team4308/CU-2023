package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class ArmRotateSystem extends MotoredSubsystem {
    public final TalonSRX motor1;

    private ArrayList<TalonSRX> controllersSRX = new ArrayList<TalonSRX>();

    public ArmRotateSystem() {
        // Setup and Add Controllers

        // for rotating arm up and down
        motor1 = new TalonSRX(Constants.Mapping.Arm.motor1);

        controllersSRX.add(motor1);

        // Reset Config for all
        for (TalonSRX talon : controllersSRX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Change Config For All Controllers
        for (TalonSRX talon : controllersSRX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
            talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp, Constants.Generic.timeoutMs);
            talon.setNeutralMode(NeutralMode.Brake);
            talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            talon.changeMotionControlFramePeriod(5);
            talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            talon.enableVoltageCompensation(true);
        }

        motor1.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.QuadEncoder, 0, Constants.Generic.timeoutMs);

        // Reset
        resetSensors();
        stopControllers();
    }
    public void resetSensors() {
        motor1.setSelectedSensorPosition(0);
    }

    /**
     * Getters And Setters
     */

    /**
     * Misc Stuff
     */

    public double getArmPosition() {
        return motor1.getSelectedSensorPosition(0);
    }

    public void setArmOutput(TalonSRXControlMode mode, double val) {
        motor1.set(mode, val);
    }

    public void stopControllers() {
        motor1.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public Sendable log() {
        Shuffleboard.getTab("Log").addNumber("Arm Rotation Pos", () -> getArmPosition());
        return this;
    }
}