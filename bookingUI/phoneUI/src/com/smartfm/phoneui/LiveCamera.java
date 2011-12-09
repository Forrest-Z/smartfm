package com.smartfm.phoneui;

import android.app.Activity;
import android.net.Uri;
import android.os.Bundle;
import android.widget.MediaController;
import android.widget.VideoView;

public class LiveCamera extends Activity {

    /**
     * TODO: Set the path variable to a streaming video URL or a local media
     * file path.
     */
//    private String path = "rtsp://v4.cache7.c.youtube.com/CjgLENy73wIaLwnX5Ct5sk5dtRMYJCAkFEIJbXYtZ29vZ2xlSARSB3Jlc3VsdHNgoMm5n8Pji8xODA==/0/0/0/video.3gp";
//	private String path = "http://daily3gp.com/vids/dragracer-blows-up.3gp";
//	private String path = "rtsp://184.72.239.149/vod/mp4:BigBuckBunny_175k.mov";
	private String path = "rtsp://192.168.1.136:5544/stream.sdp";

//
    private VideoView videoView;

    @Override
    public void onCreate(Bundle icicle) {
        super.onCreate(icicle);
        setContentView(R.layout.livecamera);
        videoView =  (VideoView) findViewById(R.id.videoview);

        /*
         * Alternatively,for streaming media you can use
         * mVideoView.setVideoURI(Uri.parse(URLstring));
         */
        videoView.setVideoURI(Uri.parse(path));
        videoView.setMediaController(new MediaController(this));
        videoView.requestFocus();
        videoView.start();
        }
    }

