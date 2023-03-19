package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import ca.team4308.absolutelib.wrapper.LogSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSystem extends LogSubsystem {

    public static NetworkTable limelight;

    // Init
    public LimelightSystem() {
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    /**
     * Built-in Limelight functions
     */
    public boolean hasTarget() {
        // 1 if limelight has target, else 0
        return limelight.getEntry("tv").getBoolean(false);
    }

    public double getXAngle() {
        // horizontal offset
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getYAngle() {
        // vertical offset
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getPipeline() {
        // returns a value from 0-9
        return limelight.getEntry("getpipe").getDouble(0.0);
    }

    public void setPipeline(double num) {
        limelight.getEntry("pipeline").setNumber(num);
    }

    /**
     * Custom Functions
     */

    public void toggleCamera() {
        double mode = limelight.getEntry("camMode").getDouble(0);
        if (mode == 0) {
            limelight.getEntry("camMode").setNumber(1);
            mode++;
            SmartDashboard.putString("Camera Mode", "Drive Mode");
        } else {
            mode = 0;
            limelight.getEntry("camMode").setNumber(0);
            SmartDashboard.putString("Camera Mode", "Normal Mode");
        }
    }

    @Override
    public Sendable log() {
        return this;
    }
}