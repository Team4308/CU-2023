package frc.robot.subsystems;

import java.util.ArrayList;
import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import ca.team4308.absolutelib.wrapper.drive.TankDriveSubsystem;

import frc.robot.Constants;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.DigitalOutput;


public class DriveSystem extends TankDriveSubsystem {

    public final DigitalOutput ledR;
    public final DigitalOutput ledG;
    public final DigitalOutput ledB;
    public boolean toggle; 
        // Master Controllers
    

    public final TalonSRX masterLeft, masterRight;
    // Slave Controllers
    private final TalonSRX slaveLeft, slaveRight;

    // Controllers
    private ArrayList<TalonSRX> controllers = new ArrayList<TalonSRX>();

    // IMU
    // public static ADIS16470_IMU gyro = new ADIS16470_IMU();

    //Beambreaks
    public static DigitalInput leftLineBreak;
    public static DigitalInput rightLineBreak;

    // Init
    public DriveSystem() {
        ledR = new DigitalOutput(1);
        ledG = new DigitalOutput(2);
        ledB = new DigitalOutput(0);
        ledR.setPWMRate(1000);
        ledG.setPWMRate(1000);
        ledB.setPWMRate(1000);
        ledR.enablePWM(0);
        ledG.enablePWM(0);
        ledB.enablePWM(0);
        toggle = false;
        // gyro.setYawAxis(IMUAxis.kX); // changes the yaw axis; kZ by default
        
        // Setup and Add Controllers
        masterLeft = new TalonSRX(Constants.Mapping.Drive.frontLeft);
        controllers.add(masterLeft);
        masterRight = new TalonSRX(Constants.Mapping.Drive.frontRight);
        controllers.add(masterRight);
        slaveLeft = new TalonSRX(Constants.Mapping.Drive.backLeft);
        controllers.add(slaveLeft);
        slaveRight = new TalonSRX(Constants.Mapping.Drive.backRight);
        controllers.add(slaveRight);

        leftLineBreak = new DigitalInput(4); // DIO 3
        rightLineBreak = new DigitalInput(5); // DIO 4

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
        masterLeft.set(mode, left);
        masterRight.set(mode, right);
    }

    public void setLEDoutput(){
        if(toggle){
                ledR.updateDutyCycle(0.5);
                ledG.updateDutyCycle(0.5);
                ledB.updateDutyCycle(0.5);
                toggle = false;
        }else{
                ledR.updateDutyCycle(0);
                ledG.updateDutyCycle(0);
                ledB.updateDutyCycle(0);
                toggle = true;
        }
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

    public Double getDistance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");

        // angle between limelight and target
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // angle of elevation of limelight
        double limeLightAngleDegrees = 0.0;

        // vertical height of limelight from ground
        double limeLightHeightCentimetres = 34.3;

        // veritcal height of april tag from ground
        double aprilTagHeightCentimetres = 31.1;

        double angleToAprilTagDegrees = targetOffsetAngle_Vertical + limeLightAngleDegrees;
        double angleToAprilTagRadians = angleToAprilTagDegrees * (Math.PI / 180.0);
        double distanceCentimetres = (aprilTagHeightCentimetres - limeLightAngleDegrees)/Math.tan(angleToAprilTagRadians);

        return Math.abs(distanceCentimetres);
    }

    public void BBAlign(){
        Boolean leftLineBreak = DriveSystem.leftLineBreak.get();
        Boolean rightLineBreak = DriveSystem.rightLineBreak.get();
        final double output = 0.2;

        if (!leftLineBreak && rightLineBreak) {
                setMotorOutput(TalonSRXControlMode.PercentOutput.toControlMode(), -output, output);
        }
        else if (!rightLineBreak && leftLineBreak) {
                setMotorOutput(TalonSRXControlMode.PercentOutput.toControlMode(), output, -output);
        }
        else {
                setMotorOutput(TalonSRXControlMode.PercentOutput.toControlMode(), output, output);
        }
    }

    @Override
    public Sendable log() { 
        // IMU 
        /* 
        Shuffleboard.getTab("Log").addDouble("AccelX", ()-> gyro.getAccelX());
        Shuffleboard.getTab("Log").addDouble("AccelY", ()-> gyro.getAccelY());
        Shuffleboard.getTab("Log").addDouble("AccelZ", ()-> gyro.getAccelZ());
        Shuffleboard.getTab("Log").addDouble("Rate", ()-> gyro.getRate());
        Shuffleboard.getTab("Log").addDouble("Angle", ()-> gyro.getAngle());
        Shuffleboard.getTab("Log").addDouble("Pitch",()-> getPitch());
        Shuffleboard.getTab("Log").addDouble("Roll",()-> getRoll());
        Shuffleboard.getTab("Log").addDouble("XFilteredAngle",()-> gyro.getXFilteredAccelAngle());
        Shuffleboard.getTab("Log").addDouble("YFilteredAngle",()-> gyro.getYFilteredAccelAngle()); */
        Shuffleboard.getTab("Log").addDouble("Distance",()-> getDistance());
        Shuffleboard.getTab("Log").addBoolean("LED",()-> toggle);
        Shuffleboard.getTab("Log").addBoolean("LeftLineBreak", ()-> leftLineBreak.get());
        Shuffleboard.getTab("Log").addBoolean("RightLineBreak", ()-> rightLineBreak.get());

        return this;
    }

}