package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class ClawSystem extends LogSubsystem {
    public final DoubleSolenoid solenoid1;
    public final DoubleSolenoid solenoid2;
    

    public ClawSystem() {
       
        solenoid1 = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, 0, 1);
        solenoid1.set(Value.kForward);

        solenoid2 = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
        solenoid2.set(Value.kForward);

       
    }

    /**
     * Getters And Setters
     */

   
    public void retract() {
        solenoid1.set(Value.kReverse);
        solenoid2.set(Value.kReverse);
    }

    public void extend() {
        solenoid1.set(Value.kForward);
        solenoid2.set(Value.kForward);
    }
    

    @Override
    public Sendable log() {
        return this;
    }
}