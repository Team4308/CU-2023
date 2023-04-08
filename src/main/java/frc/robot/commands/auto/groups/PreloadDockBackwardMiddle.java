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
import frc.robot.commands.auto.DriveTillTipped;
import frc.robot.commands.auto.TurnAngle;
import frc.robot.commands.auto.TurnDistance;
import frc.robot.subsystems.DriveSystem;

import frc.robot.commands.auto.ArmExtend;
import frc.robot.commands.auto.ArmRotate;
import frc.robot.commands.auto.ArmRotateHold;
import frc.robot.subsystems.ArmExtendSystem;
import frc.robot.subsystems.ArmRotateSystem;

import frc.robot.subsystems.ClawSystem;

public class PreloadDockBackwardMiddle extends SequentialCommandGroup {

    public PreloadDockBackwardMiddle(DriveSystem driveSystem, ArmExtendSystem armExtendSystem, ArmRotateSystem armRotateSystem, ClawSystem clawSystem) {
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
                        //game 
            new SequentialCommandGroup(
                new InstantCommand(() -> clawSystem.solenoid1.set(Value.kReverse), clawSystem),
                new WaitCommand(0.5),
                new ArmRotate(29000, armRotateSystem)
            ),
            new ParallelRaceGroup(
                //=== extend and drop piece ===
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitCommand(2),
                        new ArmExtend(-250000, armExtendSystem)
                    ),
                    //Why put this in a parallel group when theres one command
                    new ParallelCommandGroup(
                        new InstantCommand(() -> clawSystem.solenoid1.set(Value.kForward), clawSystem)
                    ),
                //=== retract ===
                    new SequentialCommandGroup(
                        new WaitCommand(0.25),
                        new ParallelDeadlineGroup(
                            new WaitCommand(2),
                            new ArmExtend(-20000, armExtendSystem)
                        )
                    )
                ),
                new RepeatCommand(new ArmRotateHold(29000, armRotateSystem))
            ),      


            
            new SequentialCommandGroup(
                new DriveDistance(-2.0, driveSystem),
                /* new ParallelRaceGroup(
                    new DriveTillTipped(16, -0.6, driveSystem),
                    new WaitCommand(3.5)
                ), */
                new ParallelDeadlineGroup(
                    //new WaitCommand(3.5),
                    new DockingCommand(driveSystem)
                )

            )
        );
    }
}