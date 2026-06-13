package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;

import java.util.ArrayList;

@TeleOp
@Config
public class MotorTest extends LinearOpMode {
    public static int motor = 0;
    public static double power = 0;
    public static ServoTest.Hubs hub = ServoTest.Hubs.ControlHub;
    public static ElapsedTime time = new ElapsedTime();

    public ArrayList<Integer> lastvalues;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);
        lastvalues = new ArrayList<>(4);
        for (int i = 0; i < 4; i++) {
            lastvalues.add(0); 
        }

        waitForStart();

        while (opModeIsActive()){
            if(hub == ServoTest.Hubs.ControlHub){
                RobotInitializers.ControlHubMotors.setMotorPower(motor, power);
            }



            for(int i = 0; i < 4; i++){
                int currentTicks = RobotInitializers.ControlHubMotors.getMotorCurrentPosition(i);
                RobotInitializers.Dashtelemetry.addData("c" + i, RobotInitializers.ControlHubMotors.getMotorCurrentPosition(i));

                /*RobotInitializers.Dashtelemetry.addData("c vel" + i, RobotInitializers.ControlHubMotors.getMotorVelocity(i));
                double raw_ticks_vel = RobotInitializers.ControlHubMotors.getMotorVelocity(i);
                RobotInitializers.Dashtelemetry.addData("raw cv" + i, raw_ticks_vel);
                RobotInitializers.Dashtelemetry.addData("crpm" + i, raw_ticks_vel / 33 * 60);*/

                //lastvalues.set(i,currentTicks);
            }
            time.reset();
            RobotInitializers.clearCache();

            //Localizer.Update();
            telemetry.update();
        }
    }
}