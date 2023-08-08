package frc.robot.Auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines {

    private final DriveSubsystem m_driveTrain;



    public AutoRoutines(DriveSubsystem m_driveTrain) {
        this.m_driveTrain = m_driveTrain;
    }

    public Command DriveBetweenPoints(
                    Translation2d startingPosition,
                    Translation2d endingPosition, 
                    DriveSubsystem m_driveTrain
                ) {

        Translation2d movement = endingPosition.minus(startingPosition);

        TrajectoryConfig configuration = new TrajectoryConfig(
                                                AutoConstants.kMaxSpeedMetersPerSecond,
                                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                            .setKinematics(DriveConstants.kDriveKinematics)
                                            ;

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                                        new Pose2d(
                                                                startingPosition, 
                                                                new Rotation2d(
                                                                        movement.getX(), movement.getY()
                                                                )
                                                        ),
                                                        //it takes the movement vector and gets the cos and sin components.
                                                        //it uses the cos and sin components to get the angle of movement.
                                                        List.of(), 
                                                        new Pose2d(
                                                                endingPosition, 
                                                                new Rotation2d(
                                                                        movement.getX(), movement.getY()
                                                                )
                                                        ), 
                                                        configuration
                                                    );

        var thetaController = new ProfiledPIDController(0, 0, 0, AutoConstants.kThetaControllerConstraints);
        var xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
        var yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);

        thetaController.enableContinuousInput(-1*Math.PI, Math.PI);

        Command swerveContollerCommand = new SwerveControllerCommand(
                                                trajectory, 
                                                m_driveTrain::getPose, 
                                                //it gives the method getPose to the command, so it can call it whenever it wants,
                                                //this is required because robot only calls the command once, but the command needs
                                                //to know its position continuously.
                                                DriveConstants.kDriveKinematics, 
                                                xPIDController, 
                                                yPIDController, 
                                                thetaController, 
                                                m_driveTrain::setModuleStates, 
                                                //again, it needs to do this continuously after being called only once
                                                m_driveTrain
                                            ).andThen(() -> m_driveTrain.drive(0, 0, 0, 0));
                                                        //using lambda instead of :: because it has parameters

        return swerveContollerCommand;
    }

                                    //Meters??? idk
    public Command driveStraightAuto(double xDistance, double yDistance) {
        return new SequentialCommandGroup(
                        new InstantCommand(
                                () -> m_driveTrain.resetOdometry(
                                                     new Pose2d(0, 0, new Rotation2d(0))
                                                   )
                                        //it resets the robot's relative position, so that all calculations after
                                        //this are based on the robot's position right now
                            )
                        , 
                        this.DriveBetweenPoints(
                                new Translation2d(0, 0), 
                                new Translation2d(xDistance, yDistance), 
                                m_driveTrain
                            )
                        );
    }

    public Command autoBalance() {
        return new SequentialCommandGroup(
            driveStraightAuto(3.75, 0),
            new RunCommand(m_driveTrain::balance, m_driveTrain)
        );
    }


}