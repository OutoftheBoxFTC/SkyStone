package Debug.EditableValue;

import org.json.JSONException;

public abstract class EditableValue {
    String name;
    boolean isUpdated;
    public EditableValue(String name){
        this.name = name;
        isUpdated = false;
    }

    public boolean isUpdated(){
        return isUpdated;
    }

    public enum TYPE{
        DOUBLE,
        VECTOR3
    }

    public abstract String getJson() throws JSONException;
}
