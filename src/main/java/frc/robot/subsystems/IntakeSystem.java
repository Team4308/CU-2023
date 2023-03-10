package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import ca.team4308.absolutelib.wrapper.MotoredSubsystem;

import edu.wpi.first.util.sendable.Sendable;

import frc.robot.Constants;

public class IntakeSystem extends MotoredSubsystem {
    public final VictorSPX intakeMotor;

    private ArrayList<VictorSPX> controllersSPX = new ArrayList<VictorSPX>();

    public IntakeSystem() {
        // Setup and Add Controllers
        intakeMotor = new VictorSPX(Constants.Mapping.Intake.intakeMotor);

        controllersSPX.add(intakeMotor);

        // Reset Config for all
        for (VictorSPX SPX : controllersSPX) {
            SPX.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Change Config For All Controllers
        for (VictorSPX SPX : controllersSPX) {
            SPX.configFactoryDefault(Constants.Generic.timeoutMs);
            SPX.configOpenloopRamp(Constants.Config.Drive.Power.kOpenLoopRamp, Constants.Generic.timeoutMs);
            SPX.configClosedloopRamp(Constants.Config.Drive.Power.kClosedLoopRamp, Constants.Generic.timeoutMs);
            SPX.setNeutralMode(NeutralMode.Coast);
            SPX.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            SPX.changeMotionControlFramePeriod(5);
            SPX.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            SPX.enableVoltageCompensation(true);
        }

        // Reset
        stopControllers();
    }

    /**
     * Getters And Setters
     */

    /**
     * Misc Stuff
     */

    public void setIntakeOutput(VictorSPXControlMode mode, double val) {
        intakeMotor.set(mode, val);
    }

    public void stopControllers() {
        intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);

    }

    @Override
    public Sendable log() {
        return this;
    }
}