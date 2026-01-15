package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.onbotjava.handlers.file.TemplateFile;
import org.firstinspires.ftc.teamcode.AutoAim;
import org.firstinspires.ftc.teamcode.Teleop.Intake;
import org.firstinspires.ftc.teamcode.Teleop.MovementSystem;
import org.firstinspires.ftc.teamcode.Teleop.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


//TIMES:
@Autonomous (name = "check ")
public class check extends OpMode{

    /*
    How this auto will work code wise is that it will have three separate methods which hold case statements.
    This is to symbolize the stages of the auto and what each stage completes/does.
     */
    private Follower follower;
    private Boolean isShooting;
    private Boolean stageOneBusy;


    private double timeToCompleteStageOne = 0;
    private double timeToCompleteStageTwo = 0;
    private double timeToCompleteStageThree = 0;


    //Enum For Shooting/Getting Preload and First Row of Balls. As well as all the variables used for it
    private enum PathStateOne {
        DRIVE_GETTING_INTO_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_1ST_ROW_POS,
        DRIVE_RESET_MID_ONE,
        SHOOT_FIRST_ROW;

    }
    private PathStateOne pathStateOne;
    private final Pose startPose = new Pose(128.073732718894, 111.92626728110596, Math.toRadians(0));
    private final Pose shootPose = new Pose(102.230, 101.032, Math.toRadians(0));
    private final Pose rowOneStart = new Pose(102.820, 86.17972350230413, Math.toRadians(0));
    private final Pose rowOneEnd = new Pose(130, 86.17972350230413  , Math.toRadians(0));
    private PathChain shootFirstThree, getIntoRowOnePos, getFirstRow, resetBackOne;



    //Enum To Shoot Second Row
    private enum PathStateTwo {
        DRIVE_2ND_ROW_POS,
        INTAKE_2ND_ROW,
        DRIVE_BACK_ROW_TWO,
        DRIVE_RESET_MID_TWO,
        SHOOT_SECOND_ROW
    }
    private PathStateTwo pathStateTwo;
    private final Pose rowTwoStart = new Pose(99.76036866359446, 56.866359447004605, Math.toRadians(0));
    private final Pose rowTwoEnd = new Pose(133.04608294930875, 56.39631336405527, Math.toRadians(0));
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
    private final Pose rowThreeStart = new Pose(99.76036866359446, 36.5529953917, Math.toRadians(0));
    private final Pose rowThreeEnd = new Pose(133.04608294930875, 38.5529953917, Math.toRadians(0));
    private final Pose parkPose = new Pose(94.72089761570828, 63.82047685834502, Math.toRadians(270));
    private PathChain getIntoRowThree, getRowThree, backUpRowTwo,resetBackThree, park;

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
        backUpRowTwo = newPathLine(rowTwoEnd, rowTwoStart);
        resetBackTwo = newPathLine(rowTwoStart, shootPose);

        //For Row Three and Park
        getIntoRowThree = newPathLine(shootPose,rowThreeStart);
        getRowThree = newPathLine(rowThreeStart, rowThreeEnd);
        resetBackThree = newPathLine(rowThreeEnd, shootPose);
        park = newPathLine(shootPose, parkPose);
    }

    //Method/Switch statement for shooting preload and getting row one & shooting. WORKS
    private void pathStateUpdateOne(){
        switch (pathStateOne) {
            case DRIVE_GETTING_INTO_SHOOT_POS: //Works
                follower.followPath(shootFirstThree, true);
                setPathStateOne(PathStateOne.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 4.8){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 4.8){
                    turnOffSystems();
                    follower.followPath(getIntoRowOnePos, true);
                    setPathStateOne(PathStateOne.DRIVE_1ST_ROW_POS);
                } break;
            case DRIVE_1ST_ROW_POS: //Works
                if (!follower.isBusy()){
                    turnOffSystems();
                    intakeBalls(pathTimer);
                    follower.followPath(getFirstRow, true);
                    setPathStateOne(PathStateOne.DRIVE_RESET_MID_ONE);
                } break;
            case DRIVE_RESET_MID_ONE: // Works
                if (!follower.isBusy()) {
                    turnOffSystems();
                    follower.followPath(resetBackOne, true);
                    setPathStateOne(PathStateOne.SHOOT_FIRST_ROW);
                } break;
            case SHOOT_FIRST_ROW: //Works
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    shootFromMedium();
                } else {
                    setTimeForCompletion(timeToCompleteStageOne);
                    break;
                }break;
            default:
                turnOffSystems();
                break;
        }
    } // WORKS

    //Method/Case statement meant for the 2nd row of balls and shooting them
    private void pathStateUpdateTwo(){
        switch(pathStateTwo){
            case DRIVE_2ND_ROW_POS:  //Works
                if (!follower.isBusy()){
                    turnOffSystems();
                    follower.followPath(getIntoRowTwo, true);
                    setPathStateTwo(PathStateTwo.INTAKE_2ND_ROW);
                } break;
            case INTAKE_2ND_ROW:
                if (!follower.isBusy()){
                    intakeBalls(pathTimer);
                    follower.followPath(getRowTwo, true);
                    setPathStateTwo(PathStateTwo.DRIVE_BACK_ROW_TWO);
                } break;
            case DRIVE_BACK_ROW_TWO:
                if (!follower.isBusy()){
                    follower.followPath(backUpRowTwo, true);
                    setPathStateTwo(PathStateTwo.DRIVE_RESET_MID_TWO);
                }break;
            case DRIVE_RESET_MID_TWO:
                if (!follower.isBusy()) {
                    turnOffSystems();
                    follower.followPath(resetBackTwo, true);
                    setPathStateTwo(PathStateTwo.SHOOT_SECOND_ROW);
                } break;
            case SHOOT_SECOND_ROW:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() < 5){
                    shootFromMedium();
                } else if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5){
                    turnOffSystems();
                    pathStateUpdateThree();
                    setTimeForCompletion(timeToCompleteStageTwo);
                } break;
            default:
                turnOffSystems();
                break;
        }
    } // WORKS

    //The last method that grabs the last row, shoots and parks in the middle. WORKS
    private void pathStateUpdateThree(){
        switch (pathStateThree){
            case DRIVE_3RD_ROW_POS:
                if (!follower.isBusy()){
                    turnOffSystems();
                    follower.followPath(getIntoRowThree );
                    setPathStateThree(PathStateThree.INTAKE_THIRD_ROW);
                } break;
            case INTAKE_THIRD_ROW:
                if (!follower.isBusy()){
                    turnOffSystems();
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
                    setPathStateThree(PathStateThree.DRIVE_PARK);
                }
            case DRIVE_PARK:
                telemetry.addLine("All Paths Done");
                setTimeForCompletion(timeToCompleteStageThree);
                break;
            default:
                turnOffSystems();
                break;
        }
    } //WORKS




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
        Shooter.setShooterPower(Shooter.SPIN_UP_VELOCITY_MEDIUMRANGE - 50);
    }
    private void turnOffSystems() {
        Shooter.setShooterPower(0);
        Intake.setBothIntakePower(0);
    }

    private void setTimeForCompletion(double timeForCompletionHolder){
        timeForCompletionHolder = opModeTimer.getElapsedTimeSeconds();
    }
    public void intakeBalls(Timer time){
        Intake.intake.setPower(1);
    }





    public void init() {
        pathStateOne = PathStateOne.DRIVE_GETTING_INTO_SHOOT_POS;
        pathStateTwo = PathStateTwo.DRIVE_2ND_ROW_POS;
        pathStateThree = PathStateThree.DRIVE_3RD_ROW_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        isShooting = false;
        follower = Constants.createFollower(hardwareMap);
        Intake.init(this);
        Shooter.init(this);
        MovementSystem.init(this);
        AutoAim.init(this);
        AutoAim.launcherRequested = false;
        AutoAim.aimEnabled = false;

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opModeTimer.resetTimer();
    }

    public void loop(){
        follower.update();
        timerStages(opModeTimer);
        telemetry.addData("Path State: ", pathStateOne.toString());
        telemetry.addData("Path Time: ", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Total Time: ", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("Phase One Completion Time: ", timeToCompleteStageOne);
        telemetry.addData("Phase Two Completion Time: ", timeToCompleteStageTwo);
        telemetry.addData("Phase Three Completion Time: ", timeToCompleteStageThree);

        AutoAim.loop();
    }

    public void timerStages(Timer time){
        if (opModeTimer.getElapsedTimeSeconds() < 11){
            pathStateUpdateOne();
        }else if (opModeTimer.getElapsedTimeSeconds() > 11 && opModeTimer.getElapsedTimeSeconds() < 19){
            pathStateUpdateTwo();
        }else if (opModeTimer.getElapsedTimeSeconds() > 19 && opModeTimer.getElapsedTimeSeconds() < 28){
            pathStateUpdateThree();
        }else {
            turnOffSystems();
        }
    }

    /*

     */
}