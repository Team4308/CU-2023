package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class ArmExtendSystem extends MotoredSubsystem {
    public final TalonFX motor2;

    // Beambreak
    public static DigitalInput extendSensor;
    public static DigitalInput retractSensor;

    private ArrayList<TalonFX> controllersFX = new ArrayList<TalonFX>();

    public ArmExtendSystem() {
        // Setup and Add Controllers

        // for extending and retracting arm
        motor2 = new TalonFX(Constants.Mapping.Arm.motor2);

        controllersFX.add(motor2);

        extendSensor = new DigitalInput(5);
        retractSensor = new DigitalInput(6);

        // Reset Config for all
        for (TalonFX talon : controllersFX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Change Config For All Controllers
        for (TalonFX talon : controllersFX) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
            talon.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            talon.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp, Constants.Generic.timeoutMs);
            talon.setNeutralMode(NeutralMode.Brake);
            talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            talon.changeMotionControlFramePeriod(5);
            talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            talon.enableVoltageCompensation(true);
        }

        motor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Generic.timeoutMs);

        // Reset
        resetSensors();
        stopControllers();
    }

    public void resetSensors() {
        motor2.setSelectedSensorPosition(0);
    }

    /**
     * Getters And Setters
     */

    /**
     * Misc Stuff
     */

    public double getSensorPosition() {
        return motor2.getSelectedSensorPosition(0);
    }

    public void setMotorOutput(TalonFXControlMode mode, double val) {
        motor2.set(mode, val);
    }

    public void stopControllers() {
        motor2.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public boolean checkIfExtend() {
        return extendSensor.get();
    }

    public boolean checkIfRetracted() {
        return !retractSensor.get();
    }
    @Override
    public Sendable log() {
        Shuffleboard.getTab("Log").addNumber("Arm Extension Pos", () -> getSensorPosition());
        Shuffleboard.getTab("Log").addNumber("Arm Extension Current", () -> motor2.getStatorCurrent());
        return this;
    }
}