package HardwareSystems;

import android.content.Context;
import android.media.MediaPlayer;
import org.firstinspires.ftc.teamcode.R;

public class SoundMixer {
    MediaPlayer horn, reverse, shiftgears;
    public SoundMixer(Context context){
        horn = MediaPlayer.create(context, R.raw.chimeconnect);
        reverse = MediaPlayer.create(context, R.raw.chimeconnect);
        shiftgears = MediaPlayer.create(context, R.raw.chimeconnect);
    }

    public void playHorn(){
        if(!horn.isPlaying()) {
            stopAll();
            horn.start();
        }
    }

    public void playReverse(){
        if(!reverse.isPlaying()) {
            stopAll();
            reverse.start();
        }
    }

    public void playShiftGears(){
        if(!reverse.isPlaying()) {
            stopAll();
            shiftgears.start();
        }
    }

    public void stopAll(){
        if(horn.isPlaying()){
            horn.pause();
            horn.seekTo(0);
        }
        if(reverse.isPlaying()){
            reverse.pause();
            reverse.seekTo(0);
        }
        if(shiftgears.isPlaying()){
            shiftgears.pause();
            shiftgears.seekTo(0);
        }
    }

}
