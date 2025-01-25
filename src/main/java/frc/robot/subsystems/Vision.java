package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.elastic.Reef;
import frc.robot.other.DetectedObject;

public class Vision extends SubsystemBase {
    private AprilTagFieldLayout fieldLayout;

    private AprilTagCamera[] aprilTagCameras;
    private ObjectCamera[] objectCameras;

    private VisionSystemSim visionSim;

    private Reef reef;

    // Data log
    private DataLog dataLog;

    public Vision(Reef reef,DataLog dataLog) {
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        AprilTagCamera frontCamera = new AprilTagCamera("frontCamera", VisionConstants.robotToFrontCamera, fieldLayout);
        AprilTagCamera leftCamera = new AprilTagCamera("leftCamera", VisionConstants.robotToLeftCamera, fieldLayout);
        aprilTagCameras = new AprilTagCamera[]{frontCamera, leftCamera};

        ObjectCamera frontObjectCamera = new ObjectCamera("frontObjectCamera",VisionConstants.robotToFrontObjectCamera);
        objectCameras = new ObjectCamera[]{frontObjectCamera};

        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        for(AprilTagCamera cam:aprilTagCameras){
            cam.startSim(visionSim);
        }

        this.reef = reef;
        this.dataLog = dataLog;
    }

    public List<EstimatedRobotPose> getNewPoses(){
        var res=new ArrayList<EstimatedRobotPose>();
        for(AprilTagCamera cam:aprilTagCameras){
            res.addAll(cam.getNewPoses());
        }
        return res;
    }

    private List<DetectedObject> getNewObjects(Pose2d robotPose) {
        var res=new ArrayList<DetectedObject>();
        for(ObjectCamera cam:objectCameras){
            res.addAll(cam.getNewObjects(robotPose));
        }
        return res;
    }

    public void processNewObjects(Pose2d robotPose){
        List<DetectedObject> objects=getNewObjects(robotPose);
        for(DetectedObject object:objects){
            if(object.getClassName()!="Coral"){
                continue;
            }
            Translation3d coralPos=object.getPos();
            Pair<Integer,Integer> coralLoc=checkObjectOnReef(coralPos);
            if(coralLoc!=null){
                reef.setCoralPlaced(coralLoc.getFirst(),coralLoc.getSecond(),true);
            }
        };
    }

    public Pair<Integer,Integer> checkObjectOnReef(Translation3d target3d) {
        double closestDist=VisionConstants.objectMarginOfError;
        int closestBranch=-1;
        int closestLevel=-1;
        for(int branch=0;branch<VisionConstants.coralPositions.length;branch++){
            for(int level=0;level<VisionConstants.coralPositions[branch].length;level++){
                Translation3d pos=VisionConstants.coralPositions[branch][level];
                double dist=target3d.minus(pos).getNorm();
                if(dist<closestDist){
                    closestDist=dist;
                    closestBranch=branch;
                    closestLevel=level;
                }
            }
        }
        if(closestBranch==-1){
            return null;
        }else{
            return Pair.of(closestLevel, closestBranch);
        }
    }

    public void updateSim(Pose2d currentPose) {
        visionSim.update(currentPose);
    }

    public void log(){
        // TODO add logs
    }
}
