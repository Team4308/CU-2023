package frc.robot.subsystems;

import ca.team4308.absolutelib.wrapper.LogSubsystem;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDSystem extends LogSubsystem {

    //INIT
    public final DigitalOutput ledR;
    public final DigitalOutput ledG;
    public final DigitalOutput ledB;

    public LEDSystem() {
        ledR = new DigitalOutput(1);
        ledG = new DigitalOutput(2);
        ledB = new DigitalOutput(0);
        ledR.setPWMRate(1000);
        ledG.setPWMRate(1000);
        ledB.setPWMRate(1000);
        ledR.enablePWM(0);
        ledG.enablePWM(0);
        ledB.enablePWM(0);
    }

    //Helper function, takes in rgb and maps 0-255 to 0-1
    public void setRGB(int R, int G, int B){
        double mapR = (1/255)*R;
        double mapG = (1/255)*G;
        double mapB = (1/255)*B;
      
        ledR.updateDutyCycle(mapR);
        ledG.updateDutyCycle(mapG);
        ledB.updateDutyCycle(mapB);
                
    }

    @Override
    public Sendable log() {
        return null;
    }
}
