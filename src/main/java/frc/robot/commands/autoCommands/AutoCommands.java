// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class AutoCommands {
    public static Command placeReefL4Command = new Command() {

        @Override
        public void initialize() {
            RobotContainer.positionManager.setReefTarget(false, 4, 4, false);
        }
    };
}
