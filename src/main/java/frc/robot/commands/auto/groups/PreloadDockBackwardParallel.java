package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.auto.DriveDistance;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.commands.auto.TurnDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.ArmRotateHold;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadDockBackwardParallel extends SequentialCommandGroup {

    public PreloadDockBackwardParallel(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
        driveSystem.resetAngle();
        addCommands(
            //places game piece, skips docking, then passes mobility bonus line

            //game piece
            // new SequentialCommandGroup(
            //     new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
            //     new WaitCommand(0.5),
            //     new ArmRotate(29000, armRotateSystem)
            // ),
            // new ParallelRaceGroup(
            //     new SequentialCommandGroup(
            //         new ParallelRaceGroup(
            //             new WaitCommand(2),
            //             new ArmExtend(-250000, armExtendSystem)
            //         ),
            //         //Why put this in a parallel group when theres one command
            //         new ParallelCommandGroup(
            //             new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem)
            //         ),
            //         new SequentialCommandGroup(
            //             new WaitCommand(1),
            //             new ParallelDeadlineGroup(
            //                 new WaitCommand(2),
            //                 new ArmExtend(-20000, armExtendSystem)
            //             )
            //         )
            //     ),
            //     new RepeatCommand(new ArmRotateHold(29000, armRotateSystem))
            // ),
                        //game piece
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.25),
                new ArmRotate(31000, armRotateSystem),
                new ParallelRaceGroup(
                    new SequentialCommandGroup(
                        new ArmExtend(-500000, armExtendSystem),
                        new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem)
                       
                        
                    ),
                    new RepeatCommand(new ArmRotateHold(31000, armRotateSystem))
                )
            ),

            //Movement and docking (guessed values)

                // new WaitCommand(0.5),
            new ParallelCommandGroup(
                new ArmExtend(-12000, armExtendSystem),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(2),
                        new ArmRotateHold(31000, armRotateSystem)),
                    new ArmRotate(3000, armRotateSystem)),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new DriveDistance(-2.5, driveSystem),
                    new DockingCommand(driveSystem))
                    )
            // new TurnDistance(1.2, -1.2, driveSystem),
            // new ParallelDeadlineGroup(
            //         new WaitCommand(4),
            // )

        );
    }
}