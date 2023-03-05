package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import ca.team4308.absolutelib.wrapper.LogSubsystem;

public class ClawSystem extends LogSubsystem {
    public final DoubleSolenoid solenoid1;

    public ClawSystem() {
        solenoid1 = new DoubleSolenoid(8, PneumaticsModuleType.CTREPCM, 1, 2);
    }

    /**
     * Getters And Setters
     */

    public void retract() {
        solenoid1.set(Value.kReverse);
    }

    public void extend() {
        solenoid1.set(Value.kForward);
    }

    @Override
    public Sendable log() {
        return this;
    }
}