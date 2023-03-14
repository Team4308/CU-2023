package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.commands.auto.DriveDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class NoBalance extends SequentialCommandGroup {

    public NoBalance(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        addCommands(
            //places game piece, skips docking, then passes mobility bonus line

            //game piece
            new SequentialCommandGroup(
                new ArmExtend(-10000, armExtendSystem),
                new ArmRotate(7500, armRotateSystem),
                new ArmExtend(15000, armExtendSystem),
                new InstantCommand(() -> clawSystem.toggle(), clawSystem),
                new ArmExtend(-15000, armExtendSystem),
                new ArmRotate(-7500, armRotateSystem)
            ),

            //docking/mobility line
            new ParallelDeadlineGroup(new WaitCommand(4), new DriveDistance(-2, driveSystem))
        );
    }
}