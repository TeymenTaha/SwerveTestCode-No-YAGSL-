// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.DriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final DriveTrain drivetrain = new DriveTrain();

    // Sürücü kumandası (Xbox)
    public static final CommandXboxController primary = new CommandXboxController(OIConstants.primaryPort);

    private static final double slowFactor = 1;

  public RobotContainer() {
    configureBindings();
    
        // Varsayılan Sürüş Komutu: Robot her zaman joystick verilerini dinler
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    // Joystick verilerini al (WPILib standartları için Y ters çevrilir)
                    double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(), OIConstants.driveDeadband);
                    double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(), OIConstants.driveDeadband);
                    double rot = -MathUtil.applyDeadband(primary.getRightX(), OIConstants.driveDeadband);
    
                    // B butonuna basılıyorsa hızı %50'ye düşür
                    if (primary.b().getAsBoolean()) {
                        ySpeed *= slowFactor;
                        xSpeed *= slowFactor;
                        rot *= slowFactor;
                    }
    
                    // Sürüşü başlat (fieldRelative: true -> Saha odaklı sürüş)
                    drivetrain.drive(ySpeed, xSpeed, rot, true);
                },
                drivetrain));
  }


  private void configureBindings() {
    // Start butonuna basıldığında Gyro'yu (ön yönü) o anki bakış yönüne göre sıfırla
    primary.start().onTrue(Commands.runOnce(() -> drivetrain.zeroHeading()));

    // X butonuna basılı tutarken tekerlekleri X şeklinde kilitle (Defans modu)
    primary.x().whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

    primary.rightTrigger().whileTrue(new RunCommand(() -> drivetrain.driveAtTarget(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
    primary.rightBumper().whileTrue(new RunCommand(() -> drivetrain.lockFront(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
    primary.leftBumper().whileTrue(new RunCommand(() -> drivetrain.lockBack(-MathUtil.applyDeadband(primary.getLeftY(),0.1), -MathUtil.applyDeadband(primary.getLeftX(), 0.1)), drivetrain));
  }

  public Command getAutonomousCommand(){
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");

      return new SequentialCommandGroup(new InstantCommand(() -> {
              drivetrain.zeroHeading();
              drivetrain.autoResetOdometry(path.getStartingHolonomicPose().orElseThrow());
      }), AutoBuilder.followPath(path));
      
    } catch (Exception e) {
        DriverStation.reportError("otonom patladı la", true);
        return Commands.none();
    }
  }

}
  
