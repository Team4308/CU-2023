package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;

import ca.team4308.absolutelib.wrapper.LogSubsystem;

public class ClawSystem extends LogSubsystem {
    public final DoubleSolenoid solenoid1;

    // Beambreak
    public static DigitalInput clawLineBreak;

    public ClawSystem() {
        solenoid1 = new DoubleSolenoid(8, PneumaticsModuleType.CTREPCM, 1, 2);
        solenoid1.set(Value.kForward); // in

        clawLineBreak = new DigitalInput(3);
    }

    /**
     * Getters And Setters
     */

    public void toggle() {
        if (solenoid1.get() == Value.kForward) {
            solenoid1.set(Value.kReverse);
        } else {
            solenoid1.set(Value.kForward);
        }

    }

    public boolean checkForObject() {
        return !clawLineBreak.get();
    }

    public void BBclose() {
        if (solenoid1.get() == Value.kForward && checkForObject()) {
            solenoid1.set(Value.kReverse);
        }
    }

    @Override
    public Sendable log() {
        return this;
    }
}