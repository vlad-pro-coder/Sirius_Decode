package org.firstinspires.ftc.teamcode.SiriusDecode.CameraPipelines;

import static org.firstinspires.ftc.teamcode.SiriusDecode.TeleopsStarter.team;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SiriusDecode.MathHelpers.Colors;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.SiriusDecode.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.SiriusDecode.TEAM;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AprilTagVision {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public static ArrayList<Integer> revelantIDs = new ArrayList<>(
            Arrays.asList(
                    20,
                    24)
    );

    public static double DistanceFromCenterRobotToCam = 147.902;

    /**
     * Constructor: set up AprilTag detection with a webcam.
     * @param hwMap FTC hardwareMap
     * @param webcamName name of the webcam in robot config
     */
    public AprilTagVision(HardwareMap hwMap, String webcamName) {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setLensIntrinsics(807.567f, 807.567f,345.549f, 267.084f)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, webcamName))
                .setCameraResolution(new Size(800,600))
                .addProcessor(aprilTag)
                .build();
        //FtcDashboard.getInstance().startCameraStream(visionPortal, 10);
    }

    /**
     * Get the latest AprilTag detections.
     * @return list of detections
     */

    /*public SparkFunOTOS.Pose2D CameraOffsets(){
        return new SparkFunOTOS.Pose2D(DistanceFromCenterRobotToCam * Math.cos(),DistanceFromCenterRobotToCam);
    }*/

    public void getAnyQRCodeID(){
        int i = 0;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            RobotInitializers.Dashtelemetry.addData(i+"",detection.id);
            i++;
        }
    }

    public static int id = -1;
    public boolean GetMotif(){//true successful read false not good
        boolean found = false;
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if(detection.id >= 21 && detection.id <= 23) {
                id = detection.id;
                found = true;
            };
            //if(detection.id == 21)
                /*Storage.Motif = new ArrayList<>(
                        Arrays.asList(
                                Colors.ColorType.GREEN,
                                Colors.ColorType.PURPLE,
                                Colors.ColorType.PURPLE
                        )
                );
                else if(detection.id == 22)
                Storage.Motif = new ArrayList<>(
                        Arrays.asList(
                                Colors.ColorType.PURPLE,
                                Colors.ColorType.GREEN,
                                Colors.ColorType.PURPLE
                        )
                );
                    else if(detection.id == 23)
                Storage.Motif = new ArrayList<>(
                        Arrays.asList(
                                Colors.ColorType.PURPLE,
                                Colors.ColorType.PURPLE,
                                Colors.ColorType.GREEN
                        )
                );*/
        }
        return found;
    }

    public double GetTeamTagBearing(){

        boolean wasRelevantTagDetected = false;
        AprilTagDetection tagOfInterest = null;

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if(team == TEAM.BLUE && detection.id == revelantIDs.get(0)) {
                tagOfInterest = detection;
                wasRelevantTagDetected = true;
            }
            else if(team == TEAM.RED && detection.id == revelantIDs.get(1)) {
                tagOfInterest = detection;
                wasRelevantTagDetected = true;
            }
        }

        RobotInitializers.Dashtelemetry.addData("tag",tagOfInterest);

        if(wasRelevantTagDetected && tagOfInterest != null)
        {
            RobotInitializers.Dashtelemetry.addData("",tagOfInterest.ftcPose.bearing);
            return tagOfInterest.ftcPose.bearing;
        }

        return -1e9;
    }
    public void LocalizerRecalibrate() {

        boolean wasRelevantTagDetected = false;
        AprilTagDetection tagOfInterest = null;

        //vlad este frumos - maza gaoaza

        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if(team == TEAM.BLUE && detection.id == revelantIDs.get(0)) {
                tagOfInterest = detection;
                wasRelevantTagDetected = true;
            }
            else if(team == TEAM.RED && detection.id == revelantIDs.get(1)) {
                tagOfInterest = detection;
                wasRelevantTagDetected = true;
            }
        }

        RobotInitializers.Dashtelemetry.addData("tag",tagOfInterest);

        if(wasRelevantTagDetected && tagOfInterest != null)
        {
            RobotInitializers.Dashtelemetry.addData("x",tagOfInterest.rawPose);
            RobotInitializers.Dashtelemetry.addData("y",tagOfInterest.robotPose);
            RobotInitializers.Dashtelemetry.addData("yaw april tag",tagOfInterest.ftcPose);

            //tagOfInterest.ftcPose.bearing

            SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(0,0,0);

            /*
            calcule
             */

            //Localizer.setPosition(pos);
        }
    }

    /**
     * Close the vision system to free resources.
     */

    public boolean IsCameraOpen(){
        return visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }
    public void open() {
        if (visionPortal != null) {
            visionPortal.resumeStreaming();
        }
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }
}
