package hardware.controller;

public class Button {
    private boolean active, updated;
    public Button(){
        active = false;
        updated = false;
    }

    public void update(boolean active){
        updated = false;
        if(this.active != active){
            updated = true;
            this.active = active;
        }
    }

    public boolean isActive() {
        return active;
    }

    public boolean isUpdated() {
        return updated;
    }
}
