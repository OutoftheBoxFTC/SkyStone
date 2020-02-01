package Debug;

import java.util.ArrayList;
import java.util.HashMap;

import math.Vector3;

public class Registers {
    HashMap<String, String> registers, turns;
    public Registers(HashMap<String, String> defaults, HashMap<String, String> turns){
        this.registers = defaults;
        this.turns = turns;
    }

    public HashMap<String, String> get(){
        return registers;
    }

    public void setPoints (HashMap<String, String> toSet){
        for(String s : toSet.keySet()){
            registers.put(s, toSet.get(s));
        }
    }

    public void setPoints(ArrayList<String> map) {
        for(String s : map) {
            String[] sa = s.replaceAll("\\s", "").split(":");
            registers.put(sa[0], sa[1]);
        }
    }

    public void setTurns(HashMap<String, String> map){
        for(String s : map.keySet()){
            turns.put(s, map.get(s));
        }
    }

    public void setTurns(ArrayList<String> map){
        for(String s : map) {
            String[] sa = s.replaceAll("\\s", "").split(":");
            turns.put(sa[0], sa[1]);
        }
    }

    public Vector3 getPoint(String key){
        String[] s = registers.get(key).replaceAll("\\s","").split(",");
        return new Vector3(Double.valueOf(s[0]), Double.valueOf(s[1]), Double.valueOf(s[2]));
    }

    public Vector3 getTurn(String key){
        String[] s = turns.get(key).replaceAll("\\s","").split(",");
        return new Vector3(Double.valueOf(s[0]), Double.valueOf(s[1]), Double.valueOf(s[2]));
    }
}
