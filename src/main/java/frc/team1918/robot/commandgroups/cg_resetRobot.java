/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team1918.robot.subsystems.ClimberSubsystem;
import frc.team1918.robot.subsystems.CollectorSubsystem;

import frc.team1918.robot.commands.collector.collector_retractIntake;
import frc.team1918.robot.commands.climber.climber_whirlygigDown;
import frc.team1918.robot.commands.climber.climber_resetClimb;

public class cg_resetRobot extends SequentialCommandGroup {
  private final CollectorSubsystem m_collector;
  private final ClimberSubsystem m_climber;
  
  /**
   * This command groups issues all the different robot reset items that have to get reset on disable
   * <ol>
   * <li>set whirlygig to down</li>
   * <li>retract intake</li>
   * </ol>
   * <br>
   * @param coll Collector Subsystem
   * @param climb Climber Subsystem
  */
  public cg_resetRobot(CollectorSubsystem coll, ClimberSubsystem climb) {
    m_collector = coll;
    m_climber = climb;
    addRequirements(m_collector, m_climber);

    /**
     * Creates a sequential command group with the objects to run in sequence.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new climber_whirlygigDown(m_climber),
        new collector_retractIntake(m_collector),
        new climber_resetClimb(m_climber)
    );
  }
}