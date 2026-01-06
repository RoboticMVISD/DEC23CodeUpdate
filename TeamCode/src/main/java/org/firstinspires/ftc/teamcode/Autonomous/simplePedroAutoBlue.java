package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Intake;
import org.firstinspires.ftc.teamcode.Teleop.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "3ballAutoBlue")
public class simplePedroAutoBlue extends OpMode {

    Follower follower;
    Timer pathTimer, opModeTimer;
    Boolean isShooting;

    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS,
        SHOOT_PRELOAD,
    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(21.004207573632538, 123.60168302945301, Math.toRadians(135));
    private final Pose shootPose = new Pose(43.22019635343619, 100.17391304347825, Math.toRadians(135));
    private final Pose park = new Pose(18.378681626928472, 100.77980364656382, Math.toRadians(270));
    private PathChain shootFirstThree, parkBot;

    private void setPathStateOne(PathStateOne newState) {
        pathStateOne = newState;
        pathTimer.resetTimer();
    }
    private void pathStateUpdateOne(){
        switch (pathStateOne) {
            case DRIVE_GETTING_INTO_SHOOT_POS:
                follower.followPath(shootFirstThree, true);
                setPathStateOne(PathStateOne.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    turnOffSystems();
                    follower.followPath(parkBot, true);
                } break;
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }

    private void shootFromMedium() {
        Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE);

        if ((Shooter.rightShooter.getVelocity() >= Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 20 && Shooter.rightShooter.getVelocity() <= Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE + 20)) {
            isShooting = true;
            Intake.setBothIntakePower(1);
        } else {
            isShooting = false;
            Intake.setBothIntakePower(0);
        }
    }
    private void turnOffSystems(){
        Intake.setBothIntakePower(0);
        Shooter.setShooterPower(0);
    }



    private PathChain newPathLine(Pose start, Pose end){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        return path;
    }


    private void buildPaths(){
        //For Preload Shoot and Park
        shootFirstThree = newPathLine(startPose, shootPose);
        parkBot = newPathLine(shootPose, park);
    }

    public void init(){
        pathStateOne = PathStateOne.DRIVE_GETTING_INTO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        isShooting = false;
        follower = Constants.createFollower(hardwareMap);
        Intake.init(this);
        Shooter.init(this);
        MovementSystem.init(this);

        buildPaths();
        follower.setPose(startPose);
    }

    public void loop(){
        follower.update();
        pathStateUpdateOne();

        telemetry.addData("Path State: ", pathStateOne.toString());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTime());
    }
}
