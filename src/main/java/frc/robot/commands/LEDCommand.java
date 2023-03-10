package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

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
            case 1: // claw is closed, arm is in some other state
                m_subsystem.setRGB(0, 0, 255);
                break;
            case 2: // claw is open, arm is in some other state
                m_subsystem.setRGB(0, 255, 0);
                break;
            case 3: // claw is closed and arm is fully extended
                m_subsystem.setRGB(0, 255, 255);
                break;
            case 4: // claw is open and fully extended
                m_subsystem.setRGB(255, 0, 255);
                break;
            case 5: // claw is closed and arm is fully back
                m_subsystem.setRGB(255, 255, 0);
                break;
            case 6: // claw is open and arm is fully back
                m_subsystem.setRGB(255, 255, 255);
                break;
            // error/warning states
            case 7: // battery is low voltage
                m_subsystem.setRGB(255, 0, 0);
                break;
            
            case 8: // everything else (theoretically never reaches this state)
                m_subsystem.setRGB(0,0,0);
        }

    }

    @Override
    public void end(boolean interrupted) {
    }

}