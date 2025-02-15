package frc.robot.gamepieces;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GamepieceConstants;
import frc.robot.constants.GeneralConstants;

public class Gamepieces extends SubsystemBase{
    private StructArrayPublisher<Pose3d> algaePublisher=NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/sim/algae",Pose3d.struct).publish();
    private StructArrayPublisher<Pose3d> coralPublisher=NetworkTableInstance.getDefault().getStructArrayTopic("SmartDashboard/sim/coral",Pose3d.struct).publish();
    private List<Gamepiece> coral;
    private List<Gamepiece> algae;
    public Gamepieces(){
        Translation3d[] pos=GeneralConstants.getAlgaePositions();
        algae=new ArrayList<>();
        for(Translation3d t:pos){
            algae.add(new Gamepiece(GamepieceConstants.algae,new Pose3d(t, new Rotation3d())));
        }
        coral=new ArrayList<>();
        Pose3d start=new Pose3d(0.65, 0.65, 1.12, new Rotation3d(0,Math.PI/4,0.9));
        for(int i=-10;i<10;i++){
            coral.add(new Gamepiece(GamepieceConstants.coral, start.transformBy(new Transform3d(0, i/10., 0, new Rotation3d()))));
        }
        Pose3d start2=new Pose3d(0.65, GeneralConstants.fieldWidth-0.65, 1.12, new Rotation3d(0,Math.PI/4,-0.9));
        for(int i=-10;i<10;i++){
            coral.add(new Gamepiece(GamepieceConstants.coral, start2.transformBy(new Transform3d(0, i/10., 0, new Rotation3d()))));
        }
        Pose3d start3=new Pose3d(GeneralConstants.fieldLength-0.65, 0.65, 1.12, new Rotation3d(0,Math.PI/4,Math.PI-0.9));
        for(int i=-10;i<10;i++){
            coral.add(new Gamepiece(GamepieceConstants.coral, start3.transformBy(new Transform3d(0, i/10., 0, new Rotation3d()))));
        }
        Pose3d start4=new Pose3d(GeneralConstants.fieldLength-0.65, GeneralConstants.fieldWidth-0.65, 1.12, new Rotation3d(0,Math.PI/4,0.9-Math.PI));
        for(int i=-10;i<10;i++){
            coral.add(new Gamepiece(GamepieceConstants.coral, start4.transformBy(new Transform3d(0, i/10., 0, new Rotation3d()))));
        }
    }

    private Gamepiece getClosest(Pose3d pose,Iterable<Gamepiece> others){
        Gamepiece closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (Gamepiece g:others) {
            Transform3d transform = g.getLocation().minus(pose);
            double distance = Math.hypot(Math.hypot(transform.getX(), transform.getY()),transform.getZ());
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = g;
            }
        }
        return closest;
    }

    public Gamepiece getClosestCoral(Pose3d pose){
        return getClosest(pose,coral);
    }

    public Gamepiece getClosestAlgae(Pose3d pose){
        return getClosest(pose,algae);
    }

    @Override
    public void periodic(){
        Pose3d[] coralPos=new Pose3d[coral.size()];
        for(int i=0;i<coral.size();i++){
            coralPos[i]=coral.get(i).getLocation();
        }
        coralPublisher.set(coralPos);
        Pose3d[] algaePos=new Pose3d[algae.size()];
        for(int i=0;i<algae.size();i++){
            algaePos[i]=algae.get(i).getLocation();
        }
        algaePublisher.set(algaePos);
        for(Gamepiece g:coral){
            g.periodic();
        }
        for(Gamepiece g:algae){
            g.periodic();
        }
    }
}
