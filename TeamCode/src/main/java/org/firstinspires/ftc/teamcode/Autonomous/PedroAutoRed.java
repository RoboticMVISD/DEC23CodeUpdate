package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Teleop.Intake;
import org.firstinspires.ftc.teamcode.Teleop.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.util.Timer;

@Autonomous (name = "MainAutoRed", group = "Red")
public class PedroAutoRed extends OpMode {

    /*
    How this auto will work code wise is that it will have three separate methods which hold case statements.
    This is to symbolize the stages of the auto and what each stage completes/does.
     */
    private Follower follower;
    private Boolean isShooting;


    //Enum For Shooting/Getting Preload and First Row of Balls. As well as all the variables used for it
    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_1ST_ROW_POS,
        DRIVE_RESET_MID_ONE,
        SHOOT_FIRST_ROW;

    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(123.72119487908962, 122.9018492176387, Math.toRadians(45));
    private final Pose shootPose = new Pose(99.34566145092461, 98.73115220483643, Math.toRadians(45));
    private final Pose rowOneStart = new Pose(100.57784011220197, 84.62272089761572, Math.toRadians(0));
    private final Pose rowOneEnd = new Pose(127.84291725105189, 83.4109396914446, Math.toRadians(0));
    private PathChain shootFirstThree, getIntoRowOnePos, getFirstRow, resetBackOne;




    //Enum To Shoot Second Row
    private enum PathStateTwo {
        DRIVE_2ND_ROW_POS,
        INTAKE_2ND_ROW,
        DRIVE_RESET_MID_TWO,
        SHOOT_SECOND_ROW
    }
    private PathStateTwo pathStateTwo;
    private final Pose rowTwoStart = new Pose(102.19354838709678, 59.3772791023843, Math.toRadians(0));
    private final Pose rowTwoEnd = new Pose(128.6507713884993, 59.3772791023843, Math.toRadians(0));
    private  PathChain getIntoRowTwo, getRowTwo, resetBackTwo;




    //Enum to shoot 3rd Row and Park
    private enum PathStateThree {
        DRIVE_3RD_ROW_POS,
        DRIVE_RESET_MID_THREE,
        INTAKE_THIRD_ROW,
        SHOOT_THIRD_ROW,
        DRIVE_PARK
    }
    private PathStateThree pathStateThree;
    private final Pose rowThreeStart = new Pose(102.19354838709678, 35.74754558204769, Math.toRadians(0));
    private final Pose rowThreeEnd = new Pose(134.30575035063114, 35.74754558204769, Math.toRadians(0));
    private final Pose parkPose = new Pose(94.72089761570828, 63.82047685834502, Math.toRadians(270));
    private PathChain getIntoRowThree, getRowThree, resetBackThree, park;

    private Timer pathTimer, opModeTimer;



    private PathChain newPathLine(Pose start, Pose end){
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
        return path;
    }
    private void buildPaths(){
        //For Preload and Row One
        shootFirstThree = newPathLine(startPose, shootPose);
        getIntoRowOnePos = newPathLine(shootPose, rowOneStart);
        getFirstRow = newPathLine(rowOneStart, rowOneEnd);
        resetBackOne = newPathLine(rowOneEnd, shootPose);

        //For Row Two
        getIntoRowTwo = newPathLine(shootPose, rowTwoStart);
        getRowTwo = newPathLine(rowTwoStart, rowTwoEnd);
        resetBackTwo = newPathLine(rowOneEnd, shootPose);

        //For Row Three and Park
        getIntoRowThree = newPathLine(shootPose,rowThreeStart);
        getRowThree = newPathLine(rowThreeStart, rowThreeEnd);
        resetBackThree = newPathLine(rowThreeEnd, shootPose);
        park = newPathLine(shootPose, parkPose);
    }

    //Method/Switch statement for shooting preload and getting row one & shooting
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
                    follower.followPath(getIntoRowOnePos, true);
                    setPathStateOne(PathStateOne.DRIVE_1ST_ROW_POS);
                } break;
            case DRIVE_1ST_ROW_POS:
                if (!follower.isBusy()){
                    turnOffSystems();
                    intakeBalls(pathTimer);
                    follower.followPath(getFirstRow, true);
                    setPathStateOne(PathStateOne.DRIVE_RESET_MID_ONE);
                } break;
            case DRIVE_RESET_MID_ONE:
                if (!follower.isBusy()) {
                    turnOffSystems();
                    follower.followPath(resetBackOne, true);
                    setPathStateOne(PathStateOne.SHOOT_FIRST_ROW);
                } break;
            case SHOOT_FIRST_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    shootFromMedium();
                } else if (isShooting){
                    turnOffSystems();
                    setPathStateTwo(PathStateTwo.DRIVE_2ND_ROW_POS);
                    pathStateUpdateTwo();
                } break;
            default:
                telemetry.addLine("No State Commanded");
                break;

        }
    }

    //Method/Case statement meant for the 2nd row of balls and shooting them
    private void pathStateUpdateTwo(){
        switch(pathStateTwo){
            case DRIVE_2ND_ROW_POS:
                if (!follower.isBusy()){
                    follower.followPath(getIntoRowTwo);
                    setPathStateTwo(PathStateTwo.INTAKE_2ND_ROW);
                } break;
            case INTAKE_2ND_ROW:
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getRowTwo);
                    setPathStateTwo(PathStateTwo.DRIVE_RESET_MID_TWO);
                } break;
            case DRIVE_RESET_MID_TWO:
                if (!follower.isBusy()) {
                    turnOffSystems();
                    follower.followPath(resetBackTwo);
                    setPathStateTwo(PathStateTwo.SHOOT_SECOND_ROW);
                } break;
            case SHOOT_SECOND_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    turnOffSystems();
                    setPathStateThree(PathStateThree.DRIVE_3RD_ROW_POS);
                    pathStateUpdateThree();
                } break;
            default:
                break;
        }
    }

    //The last method that grabs the last row, shoots and parks in the middle
    private void pathStateUpdateThree(){
        switch (pathStateThree){
            case DRIVE_3RD_ROW_POS:
                if (!follower.isBusy()){
                    follower.followPath(getIntoRowThree);
                    setPathStateThree(PathStateThree.INTAKE_THIRD_ROW);
                } break;
            case INTAKE_THIRD_ROW:
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getRowThree);
                    setPathStateThree(PathStateThree.DRIVE_RESET_MID_THREE);
                } break;
            case DRIVE_RESET_MID_THREE:
                if (!follower.isBusy()){
                    turnOffSystems();
                    follower.followPath(resetBackThree);
                    setPathStateThree(PathStateThree.SHOOT_THIRD_ROW);
                } break;
            case SHOOT_THIRD_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    turnOffSystems();
                    follower.followPath(park);
                    setPathStateThree(PathStateThree.DRIVE_RESET_MID_THREE);
                }
            case DRIVE_PARK:
                telemetry.addLine("All Paths Done");
                break;
            default:
                break;
        }
    }




    private void setPathStateOne(PathStateOne newState) {
        pathStateOne = newState;
        pathTimer.resetTimer();
    }
    private void setPathStateTwo(PathStateTwo newState) {
        pathStateTwo = newState;
        pathTimer.resetTimer();
    }
    private void setPathStateThree(PathStateThree newState) {
        pathStateThree = newState;
        pathTimer.resetTimer();
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
    private void turnOffSystems() {
        Shooter.setShooterPower(0);
        Intake.setBothIntakePower(0);
    }
    private void intakeBalls(Timer time) {
        Shooter.setShooterPower(Shooter.BALL_REVERSE_SPEED * 1.5);

        Intake.setBothIntakePower(1);
    }



    public void init() {
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

