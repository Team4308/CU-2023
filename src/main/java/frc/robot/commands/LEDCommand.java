package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSystem;

public class LEDCommand extends CommandBase {
    private final LEDSystem m_subsystem;
    private final Supplier<Integer> control;

    // Init
    public LEDCommand(LEDSystem subsystem, Supplier<Integer> control) {
        m_subsystem = subsystem;
        this.control = control;
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        int control = this.control.get();
        switch (control){
            case 1: // claw is closed
                m_subsystem.setRGB(0, 0, 255); // blue
                break;
            case 2: // brake mode
                m_subsystem.setRGB(0, 255, 0); // green
                break;
            case 3: // cone
                m_subsystem.setRGB(255, 204, 0); // cyan
                break;
            case 4: // cube
                m_subsystem.setRGB(153, 51, 255); // purple
                break;
            case 5: // claw is closed and arm is fully back
                m_subsystem.setRGB(255, 255, 0); // yellow
                break;
            case 6: // claw is open and arm is fully back
                m_subsystem.setRGB(255, 255, 255); // white
                break;
            case 9: // one of the beambreaks detects something
                m_subsystem.setRGB(255, 128, 0); // orange
                break;
            // error/warning states
            case 7: // battery is low voltage
                m_subsystem.setRGB(255, 0, 0); // red
                break;
            
            case 8: // default (to save voltage)
                m_subsystem.setRGB(0,0,0); // black

        }

    }

    @Override
    public void end(boolean interrupted) {
    }

}