package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutoAim;
import org.firstinspires.ftc.teamcode.AutoAimJ;
import org.firstinspires.ftc.teamcode.Teleop.Main;

public class Shooter {
    //Initializes all variables required to shoot. Motors on turret, blocker servos, and turret CRServo
    static OpMode op;
    public static DcMotorEx leftShooter, rightShooter;
    public static CRServo turretRotator;
    static Servo blocker;

    //Velocity Tester increments for shooterTesterConTwo()
    double TESTVELOCITY = 500;
    int SMALL_INCREMENT = 50;
    int LARGE_INCREMENT = 100;

    //Max Velocity range is -2500 to 2500
    public static double SPIN_UP_VELOCITY_SHORTRANGE = 950;
    public static double SPIN_UP_VELOCITY_MEDIUMRANGE = 1050;
    public static double SPIN_UP_VELOCITY_LONGRANGE = 1225;
    public static double SPIN_UP_VELOCITY_XLRANGE = 1550;
    public static double BALL_REVERSE_SPEED = -500;
    public static double TURRET_ROTATE_SPEED = 0.15;
    public static double TURRET_ROTATE_SPEED_FAST = .4;

    //PIDF Variables
    public static double proportional = 300;
    public static double integral = 0;
    public static double derivative = 0.001;
    public static double feedForward = 10;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(proportional, integral, derivative, feedForward);

    public static boolean autoAimEnabled;

    public static void init(OpMode OP) {
        op = OP;

        leftShooter = op.hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        rightShooter = op.hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        turretRotator = op.hardwareMap.crservo.get("turretRotator");
        turretRotator.setDirection(DcMotorSimple.Direction.REVERSE);

        blocker = op.hardwareMap.servo.get("blocker");
        blocker.setDirection(Servo.Direction.REVERSE);
    }

    public void loop() throws InterruptedException {
        shooterConTwo();
        rotateTurret();
    }

    public void shooterConTwo() {
        if (op.gamepad2.y) {
            rightShooter.setVelocity(SPIN_UP_VELOCITY_LONGRANGE);
            leftShooter.setVelocity(SPIN_UP_VELOCITY_LONGRANGE);
        } else if (op.gamepad2.b) {
            rightShooter.setVelocity(SPIN_UP_VELOCITY_MEDIUMRANGE);
            leftShooter.setVelocity(SPIN_UP_VELOCITY_MEDIUMRANGE);
        } else if (op.gamepad2.a) {
            rightShooter.setVelocity(SPIN_UP_VELOCITY_SHORTRANGE);
            leftShooter.setVelocity(SPIN_UP_VELOCITY_SHORTRANGE);
        } else if (op.gamepad2.x) {
            leftShooter.setVelocity(-SPIN_UP_VELOCITY_SHORTRANGE * 0.5);
            rightShooter.setVelocity(-SPIN_UP_VELOCITY_SHORTRANGE * 0.5);
        } else {
            rightShooter.setPower(0);
            leftShooter.setPower(0);
        }
        /*
        else if (op.gamepad2.dpad_right) {
            AutoAimJ.autoDistancingRequested = true;
        } else if (op.gamepad2.dpad_left) {
            AutoAimJ.autoDistancingRequested = false;
        }
         */


    }

    public void rotateTurret() {
        if (op.gamepad2.right_trigger > 0) {
            turretRotator.setPower(TURRET_ROTATE_SPEED_FAST);
        } else if (op.gamepad2.left_trigger > 0) {
            turretRotator.setPower(-TURRET_ROTATE_SPEED_FAST);
        } else if (op.gamepad2.left_bumper){
            turretRotator.setPower(-TURRET_ROTATE_SPEED);
        }else if (op.gamepad2.right_bumper){
            turretRotator.setPower(TURRET_ROTATE_SPEED);
        }else {
            turretRotator.setPower(0);
        }
    }

    public void shooterTesterConTwo() throws InterruptedException {
        if (op.gamepad2.dpadUpWasPressed()) {
            TESTVELOCITY += LARGE_INCREMENT;
        } else if (op.gamepad2.dpadDownWasPressed()) {
            TESTVELOCITY -= LARGE_INCREMENT;
        } else if (op.gamepad2.dpadLeftWasPressed()) {
            TESTVELOCITY -= SMALL_INCREMENT;
        } else if (op.gamepad2.dpadRightWasPressed()) {
            TESTVELOCITY += SMALL_INCREMENT;
        } else if (op.gamepad2.right_bumper) {
            rightShooter.setVelocity(TESTVELOCITY);
            leftShooter.setVelocity(TESTVELOCITY);
        } else if (op.gamepad2.left_bumper) {
            rightShooter.setVelocity(-TESTVELOCITY);
            leftShooter.setVelocity(-TESTVELOCITY);
        } else {
            leftShooter.setVelocity(0);
            rightShooter.setVelocity(0);
        }

        op.telemetry.addLine("Spin Power Left: " + leftShooter.getVelocity() + " \nSpin Power Right: " + leftShooter.getVelocity() + "\nWhat Power Should be: " + TESTVELOCITY);
    }

    public static void setShooterPower(double Vel) {
        leftShooter.setVelocity(Vel);
        rightShooter.setVelocity(Vel);
    }
}