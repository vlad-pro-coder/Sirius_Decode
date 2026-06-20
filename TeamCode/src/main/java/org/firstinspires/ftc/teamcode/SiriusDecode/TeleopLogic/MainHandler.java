package org.firstinspires.ftc.teamcode.SiriusDecode.TeleopLogic;

import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.gm1;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.gm2;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.intake;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.prevgm1;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.prevgm2;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.shooter;
import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.storage;


import org.firstinspires.ftc.teamcode.SiriusDecode.ActionsCommandLineImplementation.AsyncScheduler;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Turret;
import org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter;

public class MainHandler {

    AsyncScheduler tasks,cameraTasks;
    public MainHandler()
    {
        tasks = new AsyncScheduler();
        cameraTasks = new AsyncScheduler();
    }

    public void ActionIntake()
    {
        if(gm1.right_bumper && !gm1.left_bumper)
        {
            intake.takeIn();
        }
        else if(gm1.left_bumper && !gm1.right_bumper)
        {
            intake.spit();
        }
        else
        {
            intake.stop();
        }
    }

    public void update()
    {
        if(!Storage.ProgramActivatedIntake)
            ActionIntake();

        if(gm1.left_trigger > 0.05 && prevgm1.left_trigger < 0.05)
            Turret.ManualOffset += Math.toRadians(2);
        if(gm1.right_trigger > 0.05 && prevgm1.right_trigger < 0.05)
            Turret.ManualOffset -= Math.toRadians(2);

        if (gm1.square != prevgm1.square && gm1.square && storage.CurrentState == Storage.StorageStates.DOWAITACTIONSFORSHOOTING && storage.tasks.IsSchedulerDone() && shooter.RPMError(200))
            storage.CurrentState = Storage.StorageStates.DOSHOOTACTIONS;

        if(gm1.circle != prevgm1.circle && gm1.circle)
            storage.CurrentState = Storage.StorageStates.DOWAITACTIONSFORSHOOTING;

        tasks.update();
    }

}
