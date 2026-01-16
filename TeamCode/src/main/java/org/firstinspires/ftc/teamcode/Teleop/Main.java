package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.AutoAim;
import org.firstinspires.ftc.teamcode.AutoAimJ;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp (name = "Teleop 2025")
public class Main extends OpMode {

   MovementSystem movementSystem = new MovementSystem();
   Intake intake = new Intake();
   Shooter shooter = new Shooter();

   AutoAimJ autoAimJ = new AutoAimJ();


   @Override
   public void init()
    {
        intake.init(this);
        movementSystem.init(this);
        shooter.init(this);
        autoAimJ.init(this);
    }

    @Override
    public void loop(){
        intake.loop();
        movementSystem.loop();
        autoAimJ.loop();

        try {
            shooter.loop();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Is AutoAim on? ", AutoAim.aimEnabled);
        telemetry.addData("Is AutoDistancing on?", AutoAim.launcherRequested);
    }
    public void stop(){
       autoAimJ.stop();
    }
}
