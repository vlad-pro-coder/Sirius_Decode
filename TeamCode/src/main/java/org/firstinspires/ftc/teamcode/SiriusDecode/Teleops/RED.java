package org.firstinspires.ftc.teamcode.SiriusDecode.Teleops;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.TEAM;
import org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter;

@TeleOp(name = ".peppersRED \uD83D\uDFE5")
public class RED extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleopsStarter opmode = new TeleopsStarter(hardwareMap, TEAM.RED);
        TeleopsStarter.team = TEAM.RED;


        while(opModeInInit()){
            TeleopsStarter.gm1.copy(gamepad1);
            TeleopsStarter.gm2.copy(gamepad2);
            opmode.InitUpdate();
        }

        waitForStart();
        RobotInitializers.enable();
        while (opModeIsActive()){

            TeleopsStarter.gm1.copy(gamepad1);
            TeleopsStarter.gm2.copy(gamepad2);
            opmode.update();

            if(TeleopsStarter.storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING && TeleopsStarter.storage.LastBallAction != Storage.StorageStates.DOWAITACTIONSFORSHOOTING)
                gamepad1.rumble(700);

            //for(int i = 0;i<4;i++)
            //    telemetry.addData("c motorcurrent " + i,RobotInitializers.ControlHubMotors.getMotorCurrent(i, CurrentUnit.AMPS));


            telemetry.update();
        }
    }

}
