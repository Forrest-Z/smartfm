package com.smartfm.phoneui;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class TaskDetailActivity extends Activity implements OnClickListener {

	Task task;

	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.taskdetail);

		String s = getIntent().getStringExtra("com.smartfm.phoneui.taskstring");
		task = Task.fromString(s);

		displayTask();
		// TODO: Create a thread to periodically update the data.

		((Button) findViewById(R.id.canceltaskbutton)).setOnClickListener(this);
	}

	void displayTask() {
		((TextView) findViewById(R.id.detail_pickup)).setText(task.pickup);
		((TextView) findViewById(R.id.detail_dropoff)).setText(task.dropoff);
		((TextView) findViewById(R.id.detail_status)).setText(task.status);
		((TextView) findViewById(R.id.detail_eta)).setText(Integer.toString(task.eta));
	}

	@Override
	public void onClick(View v) {
		try {
			DBInterface.cancelTask(task.requestID);
			finish();
		} catch (Exception e) {
			ErrDialog.show(this, "Error while creating the task (RPC call failed).\n" + e);
		}
	}

}
