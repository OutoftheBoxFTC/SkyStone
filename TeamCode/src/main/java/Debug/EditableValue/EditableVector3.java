package Debug.EditableValue;

import org.json.JSONException;
import org.json.JSONObject;

import math.Vector3;

public class EditableVector3 extends EditableValue {
    Vector3 value;
    public EditableVector3(String name, Vector3 value) {
        super(name);
        this.value = value;
    }

    public void setValue(Vector3 value) {
        isUpdated = true;
        this.value = value;
    }

    public Vector3 getValue() {
        isUpdated = false;
        return value;
    }

    public TYPE getType(){
        return TYPE.VECTOR3;
    }

    @Override
    public String getJson() throws JSONException {
        JSONObject object = new JSONObject();
        object.put("name", name);
        object.put("type", "Vector3");
        object.put("value", value.toString());
        return object.toString();
    }
}
