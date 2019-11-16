package Debug;

import java.util.HashMap;

import math.Vector3;

public class Registers {
    HashMap<String, String> registers;
    public Registers(HashMap<String, String> defaults){
        this.registers = defaults;
    }

    public HashMap<String, String> get(){
        return registers;
    }

    public void set (HashMap<String, String> toSet){
        for(String s : toSet.keySet()){
            registers.put(s, toSet.get(s));
        }
    }

    public Vector3 getVector(String key){
        String[] s = registers.get(key).replaceAll("\\s","").split(",");
        return new Vector3(Integer.valueOf(s[0]), Integer.valueOf(s[1]), Integer.valueOf(s[2]));
    }
}
