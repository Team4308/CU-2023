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
        solenoid1.set(Value.kForward);
    }

    /**
     * Getters And Setters
     */

    public void toggle() {
        Value state = solenoid1.get();
        if(state==Value.kForward){
            solenoid1.set(Value.kReverse);
        }
        else{
            solenoid1.set(Value.kForward);
        }
        
    }

    

    @Override
    public Sendable log() {
        return this;
    }
}