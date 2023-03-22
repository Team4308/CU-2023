package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadDock extends SequentialCommandGroup {

    public PreloadDock(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        addCommands(
            //places game piece then immediately docks

            //game piece
            new SequentialCommandGroup(
                new ArmRotate(24000, armRotateSystem),
                new ArmExtend(80000, armExtendSystem),
                new InstantCommand(() -> clawSystem.toggle(), clawSystem),
                new ArmExtend(-80000, armExtendSystem),
                new ArmRotate(-24000, armRotateSystem)
            ),

            //docking/mobility line
            new ParallelDeadlineGroup(new WaitCommand(3), new DriveDistance(-0.5, driveSystem)),
            new ParallelDeadlineGroup(new WaitCommand(2), new DockingCommand(driveSystem))
        );
    }
}