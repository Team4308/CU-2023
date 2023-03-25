package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class LEDSystem extends LogSubsystem {

    //INIT
    public final DigitalOutput ledR;
    public final DigitalOutput ledG;
    public final DigitalOutput ledB;

    public LEDSystem() {
        // On the final bot I'm going to put R on 0, G on 1, B on 2
        ledR = new DigitalOutput(0);
        ledG = new DigitalOutput(1);
        ledB = new DigitalOutput(2);
        ledR.setPWMRate(1000);
        ledG.setPWMRate(1000);
        ledB.setPWMRate(1000);
        ledR.enablePWM(0);
        ledG.enablePWM(0);
        ledB.enablePWM(0);
    }

    //Helper function, takes in rgb and maps 0-255 to 0-1
    public void setRGB(int R, int G, int B){
        double mapR = (1.0/255)*R;
        double mapG = (1.0/255)*G;
        double mapB = (1.0/255)*B; 
        ledR.updateDutyCycle(mapR);
        ledG.updateDutyCycle(mapG);
        ledB.updateDutyCycle(mapB); 
    }

    public Sendable log() {
        // Shuffleboard.getTab("Log").addString("LED Current Command", () -> this.getCurrentCommand().getName());
        // Shuffleboard.getTab("Log").addString("LED Default Command", () -> this.getDefaultCommand().getName());
        return this;
    }
}
