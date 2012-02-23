package com.smartfm.phoneui;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class TaskDetailActivity extends Activity implements OnClickListener {

	Task task;
	Handler handler = new Handler();

	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.taskdetail);

		String s = getIntent().getStringExtra("com.smartfm.phoneui.taskstring");
		task = Task.fromString(s);
		displayTask();

		((Button) findViewById(R.id.canceltaskbutton)).setOnClickListener(this);
		((Button) findViewById(R.id.viewtaskonmapbutton)).setOnClickListener(this);
		
		Runnable periodicUpdateProcess = new Runnable(){
			@Override
			public void run() {
				updateTask();
				handler.postDelayed(this, 3000);
			}
		};
		handler.removeCallbacks(periodicUpdateProcess);
		handler.postDelayed(periodicUpdateProcess, 3000);
	}

	void displayTask() {		
		((TextView) findViewById(R.id.detail_pickup)).setText(task.pickup);
		((TextView) findViewById(R.id.detail_dropoff)).setText(task.dropoff);
		((TextView) findViewById(R.id.detail_status)).setText(task.status);
		if( task.status.compareToIgnoreCase("Acknowledged") == 0 ||
				task.status.compareToIgnoreCase("Confirmed") == 0 ||
				task.status.compareToIgnoreCase("Processing") == 0 )
			((TextView) findViewById(R.id.detail_eta)).setText(Integer.toString(task.vehicle.eta));
		else
			((TextView) findViewById(R.id.detail_eta)).setText("");
	}
	
	void updateTask() {
		try {
			task = DBInterface.getTask(task.requestID);
			displayTask();
		} catch (Exception e) {
			ErrDialog.show(this, "An error occured while retrieving the task info.\n"+e);
		}
	}

	@Override
	public void onClick(View v) {
		switch(v.getId()) {
		case R.id.canceltaskbutton:
			try {
				DBInterface.cancelTask(task.requestID);
				finish();
			} catch (Exception e) {
				ErrDialog.show(this, "Error while creating the task (RPC call failed).\n" + e);
			}
			break;

		case R.id.viewtaskonmapbutton:
			startActivity(new Intent(this,MapLocations.class));
			break;
		}
	}
	
}
