package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "HPSideRed", group = "Autonomous")
public class HPSideRed extends LinearOpMode {
    public void runOpMode() {
        robot = new Robot(hardwareMap, true);
        Pose2d beginPose = new Pose2d(12, -60, Math.toRadians(90));

        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Action auton = drive.actionBuilder(drive.pose)

                .setTangent(Math.toRadians(110))
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(90)), Math.toRadians(110))
                .waitSeconds(0.5)

                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(58, -50, Math.toRadians(90)), -Math.PI/10)
                .waitSeconds(0.5)

                .turn(Math.toRadians(20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(0.5)

                .turn(Math.toRadians(-20))
                .waitSeconds(1)

                // gonna see me cycling
                .setTangent(9 * Math.PI/10)
                .splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(-90)), 9 * Math.PI/10)
                .waitSeconds(0.5)
                .setTangent(-Math.PI/10)
                .splineToLinearHeading(new Pose2d(37, -40, Math.toRadians(-90)), -Math.PI/10)
                .waitSeconds(0.5)

                .build();
        telemetry.addData("is","starting");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
    }
}