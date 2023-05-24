package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.drive.TankDriveSubsystem;

import frc.robot.Constants;

import edu.wpi.first.util.sendable.Sendable;

public class DriveSystem extends TankDriveSubsystem {

        Double control = 0.6;

// Master Controllers
    

    public final TalonSRX masterLeft, masterRight;
    // Slave Controllers
    private final TalonSRX slaveLeft, slaveRight;

    // Controllers
    private ArrayList<TalonSRX> controllers = new ArrayList<TalonSRX>();

    // Init
    public DriveSystem() {
        
        // Setup and Add Controllers
        masterLeft = new TalonSRX(Constants.Mapping.Drive.frontLeft);
        controllers.add(masterLeft);
        masterRight = new TalonSRX(Constants.Mapping.Drive.frontRight);
        controllers.add(masterRight);
        slaveLeft = new TalonSRX(Constants.Mapping.Drive.backLeft);
        controllers.add(slaveLeft);
        slaveRight = new TalonSRX(Constants.Mapping.Drive.backRight);
        controllers.add(slaveRight);

        // Reset Config for all
        for (TalonSRX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
        }

        // Set Invert Mode

        // Set slaves to follow masters
        slaveLeft.follow(masterLeft);
        // slaveLeft.setInverted(TalonFXInvertType.FollowMaster);
        slaveRight.follow(masterRight);
        // slaveRight.setInverted(TalonFXInvertType.FollowMaster);

        // Change Config For All Controllers
        for (TalonSRX talon : controllers) {
            talon.configFactoryDefault(Constants.Generic.timeoutMs);
            talon.setNeutralMode(NeutralMode.Coast);
            talon.configNeutralDeadband(0.001, Constants.Generic.timeoutMs);
            talon.changeMotionControlFramePeriod(5);
            talon.configVoltageCompSaturation(12.5, Constants.Generic.timeoutMs);
            talon.enableVoltageCompensation(true);
        }

        // Configure Primary Closed Loop Sensor
        // masterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Constants.Generic.timeoutMs);
        // masterRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
        //        Constants.Generic.timeoutMs);

        // Set Sensor Phase for all loops
        masterLeft.setSensorPhase(false);
        // masterRight.setSensorPhase(false);

        // Set Left Velocity PIDF values
        masterLeft.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kP, Constants.Generic.timeoutMs);
        masterLeft.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kI, Constants.Generic.timeoutMs);
        masterLeft.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kD, Constants.Generic.timeoutMs);
        masterLeft.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Left.kF, Constants.Generic.timeoutMs);
        masterLeft.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

        // Set Right Velocity PIDF values
        masterRight.config_kP(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kP, Constants.Generic.timeoutMs);
        masterRight.config_kI(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kI, Constants.Generic.timeoutMs);
        masterRight.config_kD(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kD, Constants.Generic.timeoutMs);
        masterRight.config_kF(Constants.Config.Drive.VelocityControl.profileSlot,
                Constants.Config.Drive.VelocityControl.Right.kF, Constants.Generic.timeoutMs);
        masterRight.selectProfileSlot(Constants.Config.Drive.VelocityControl.profileSlot, 0);

        masterLeft.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
        masterRight.configMotionCruiseVelocity(Constants.Config.Drive.MotionMagic.maxVel);
        masterLeft.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);
        masterRight.configMotionAcceleration(Constants.Config.Drive.MotionMagic.maxAcc);

        masterLeft.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
        masterLeft.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
        masterLeft.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
        masterLeft.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);

        masterRight.config_kP(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kP, Constants.Generic.timeoutMs);
        masterRight.config_kI(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kI, Constants.Generic.timeoutMs);
        masterRight.config_kD(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kD, Constants.Generic.timeoutMs);
        masterRight.config_kF(Constants.Config.Drive.MotionMagic.profileSlot,
                Constants.Config.Drive.MotionMagic.Right.kF, Constants.Generic.timeoutMs);


        // Reset
        stopControllers();
        resetSensors();
    }

    /**
     * Getters And Setters
     */

    public void changeMotorSpeed() {
        if (control == 0.7) {
                control = 0.5;
        } else {
                control += 0.1;
        }
    }

    public double getLeftSensorPosition() {
        return masterLeft.getSelectedSensorPosition(0) * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
    }

    public double getRightSensorPosition() {
        return masterRight.getSelectedSensorPosition(0) * Constants.Config.Drive.Kinematics.kEncoderInchesPerCount;
    }

    public double getLeftSensorVelocity() {
        return masterLeft.getSelectedSensorVelocity(0);
    }

    public double getRightSensorVelocity() {
        return masterRight.getSelectedSensorVelocity(0);
    }

    /**
     * Misc Stuff
     */

    public void setMotorOutput(ControlMode mode, double left, double right) {
        masterLeft.set(mode, left*control);
        masterRight.set(mode, right*control);
    }

    public void selectProfileSlot(int slot) {
        masterLeft.selectProfileSlot(slot, 0);
        masterRight.selectProfileSlot(slot, 0);
    }

    public void resetAngle(){
        // gyro.reset();
    }

    public void stopControllers() {
        masterLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
        masterRight.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    // Sensor Reset
    public void resetSensors() {
        masterLeft.setSelectedSensorPosition(0);
        masterRight.setSelectedSensorPosition(0);
    }

    @Override
    public Sendable log() { 
        return this;
    }

}