package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.security.PublicKey;

@Autonomous
public class AUTOTest extends robotBase{
    @Override
    protected void robotInit() {
        waitForStart();


    }

    @Override
    protected void robotStart() throws InterruptedException {
        Pose2d startPose = new Pose2d(-8, 60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .back(20)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeLeft(20)

                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(-38, 12))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeTo(new Vector2d(-45, 50))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeTo(new Vector2d(-45, 12))
                .build();


        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);



    }
}
