package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TuneChassisPIDS extends LinearOpMode {

    public static double x = 0;
    public static double y = 0;
    public static double hdeg = 0;
    public static double h = Math.toRadians(hdeg);

    public static PIDCoefficients strafe= new PIDCoefficients(0,0,0);
    public static PIDCoefficients forward = new PIDCoefficients(0,0,0);
    public static PIDCoefficients heading = new PIDCoefficients(0,0,0);


    @Override
    public void runOpMode() throws InterruptedException {

                RobotInitializers.InitializeFull(hardwareMap);
                AsyncScheduler tasks = new AsyncScheduler()
                        .addTask(new Task() {
                            @Override
                            protected void Actions() {
                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0,0,0));
                            }
                            @Override
                            protected boolean Conditions() {
                                return true;
                            }
                        })
                        .waitSeconds(5)
                        .addTask(new Task() {
                            @Override
                            protected void Actions() {
                                Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(x,y,h));
                            }

                            @Override
                            protected boolean Conditions() {
                                return true;
                            }
                        })
                        .waitSeconds(5);

                AsyncScheduler currenttasks = new AsyncScheduler();
                currenttasks.AddAnotherAsyncScheduler(tasks);
                while(opModeInInit()){
                    Localizer.Reset();
                }

                waitForStart();

                while(opModeIsActive())
                {
                    h = Math.toRadians(hdeg);
                    Chassis.Strafe.setPidCoefficients(strafe);
                    Chassis.Forward.setPidCoefficients(forward);
                    Chassis.Heading.setPidCoefficients(heading);

                    currenttasks.update();
                    if(currenttasks.IsSchedulerDone())
                        currenttasks.AddAnotherAsyncScheduler(tasks);

                    Localizer.Update();
                    Chassis.update();
                    RobotInitializers.clearCache();
                }
            }
}
