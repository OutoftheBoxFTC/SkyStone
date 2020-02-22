package Debug.EditableValue;

import org.json.JSONException;
import org.json.JSONObject;

public class EditableDouble extends EditableValue {
    double value;
    public EditableDouble(String name, double value) {
        super(name);
        this.value = value;
    }

    public void setValue(double value){
        this.value = value;
        this.isUpdated = true;
    }

    public double getValue() {
        this.isUpdated = false;
        return value;
    }

    public TYPE getType(){
        return TYPE.DOUBLE;
    }

    @Override
    public String getJson() throws JSONException {
        JSONObject object = new JSONObject();
        object.put("name", name);
        object.put("type", "double");
        object.put("value", value);
        return object.toString();
    }
}
