package frc.robot.elastic;

import org.json.simple.JSONObject;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;

public class Reef implements NTSendable{

    // coral placed is L2, L3, L4 then 12 branches
    private boolean[][] coralPlaced;

    // algae placed is AB, CD, ..., KL
    private boolean[] algaePlaced;

    public Reef(){
        coralPlaced=new boolean[3][12];
        algaePlaced=new boolean[6];
        for(int i=0; i<algaePlaced.length; i++){
            algaePlaced[i]=true;
        }
    }

    // levels are 0 = L2, 1 = L3, 2 = L4
    public boolean isCoralPlaced(int level,int branch){
        return coralPlaced[level][branch];
    }

    public void setCoralPlaced(int level,int branch,boolean set){
        coralPlaced[level][branch]=set;
    }

    public boolean isAlgaePlaced(int side){
        return algaePlaced[side];
    }

    public void setAlgaePlaced(int side,boolean set){
        algaePlaced[side]=set;
    }

    public boolean isCoralBlocked(int level,int branch){
        int side=branch/2;
        if(algaePlaced[side]){
            int lowAlgae=side%2;
            if(lowAlgae==0){
                return level==1;
            }else{
                return (level==0 || level==1);
            }
        }else{
            return false;
        }
    }

    @SuppressWarnings("unchecked")
    public String jsonify(){
        JSONObject obj = new JSONObject();
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                String propName="B"+j+"L"+(i+2);
                if(isCoralBlocked(i,j)){
                    obj.put(propName, "Blocked");
                }else if(isCoralPlaced(i,j)){
                    obj.put(propName, "Placed");
                }else{
                    obj.put(propName, "Open");
                }
            }
        }
        return obj.toJSONString();
    }
    
    @Override
    public void initSendable(NTSendableBuilder builder) {
        for (int i=0; i<coralPlaced.length; i++){
            for (int j=0; j<coralPlaced[i].length; j++){
                int icopy=i;
                int jcopy=j;
                builder.addBooleanProperty("B"+j+"L"+(i+2), ()->isCoralPlaced(icopy, jcopy), (value)->setCoralPlaced(icopy, jcopy, value));
            }
        }
        for(int i=0; i<algaePlaced.length; i++){
            int icopy=i;
            builder.addBooleanProperty("Side "+i, ()->isAlgaePlaced(icopy), (value)->setAlgaePlaced(icopy, value));
        }
        builder.addStringProperty("ReefJson", ()->jsonify(), null);
        builder.setSmartDashboardType("Reef");
    }
}
