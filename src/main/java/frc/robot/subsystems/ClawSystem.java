package frc.robot.subsystems;


import bbb.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;



public class ClawSystem extends LogSubsystem {

    //Solenoid
    public final DoubleSolenoid solenoid;
    public final DoubleSolenoid solenoid2;

    // Init
    public ClawSystem() {
        solenoid = new DoubleSolenoid(9, PneumaticsModuleType.CTREPCM, 0, 1);
        solenoid2 = new DoubleSolenoid(9, PneumaticsModuleType.CTREPCM, 2, 3);
        // Reset
        stopControllers();
    }
    public void stopControllers() {
    }


    public void retract() {
        solenoid.set(Value.kReverse);
        solenoid2.set(Value.kReverse);
    }

    public void extend() {
        solenoid.set(Value.kForward);
        solenoid2.set(Value.kForward);
    }
    

    @Override
    public Sendable log() {/* 
        Shuffleboard.getTab("Log").addNumber("Left Vel",
                () -> ((getLeftSensorVelocity() / Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation) * 600));
        Shuffleboard.getTab("Log").addNumber("Right Vel",
                () -> ((getRightSensorVelocity() / Constants.Config.Drive.Kinematics.kSensorUnitsPerRotation) * 600));
        Shuffleboard.getTab("Log").addNumber("Left Pos", () -> getLeftSensorPosition());
        Shuffleboard.getTab("Log").addNumber("Right Pos", () -> getRightSensorPosition()); */
        return this;
    }
}