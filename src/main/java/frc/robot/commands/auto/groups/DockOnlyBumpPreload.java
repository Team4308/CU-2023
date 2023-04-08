package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.DockingCommand;

import frc.robot.commands.auto.DriveDistance;

import frc.robot.subsystems.DriveSystem;
import frc.robot.subsystems.ClawSystem;


public class DockOnlyBumpPreload extends SequentialCommandGroup {

    public DockOnlyBumpPreload(DriveSystem driveSystem, ClawSystem clawSystem) {
        //Starts facing charging station, moves forward and docks
        addCommands(
            new SequentialCommandGroup(
                new DriveDistance(-1, driveSystem),
                new DriveDistance(1.5, driveSystem),
                new ParallelDeadlineGroup(new WaitCommand(10), new DockingCommand(driveSystem))
            )

        );
    }
}