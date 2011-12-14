package com.smartfm.phoneui;

import java.util.List;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class TaskList extends Activity implements OnClickListener {

	TextView useridText, taskidText, pickupText, dropoffText, etaText,
			caridText;
	Button bookbutton, cancelbutton;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.info);
		
		bookbutton = (Button) findViewById(R.id.bookbutton);
		bookbutton.setOnClickListener(this);
		cancelbutton = (Button) findViewById(R.id.cancelbutton);
		cancelbutton.setOnClickListener(this);

		useridText = (TextView) findViewById(R.id.useridtext);
		useridText.setText("");
		taskidText = (TextView) findViewById(R.id.taskidtext);
		taskidText.setText("");
		pickupText = (TextView) findViewById(R.id.pickupinfotext);
		pickupText.setText("");
		dropoffText = (TextView) findViewById(R.id.dropoffinfotext);
		dropoffText.setText("");
		etaText = (TextView) findViewById(R.id.etatext);
		etaText.setText("");
		caridText = (TextView) findViewById(R.id.caridtext);
		caridText.setText("");
	}
	
	void update() {
		try {
			List<Task> tasks = DBInterface.listTasks();
			//TODO find current task and fill in the text fields
		} catch (Exception e) {
			Context context = getApplicationContext();
			CharSequence text = "Error while retrieving your bookings (RPC call failed).";
			Toast toast = Toast.makeText(context, text, Toast.LENGTH_LONG);
			toast.show();
		}
	}

	@Override
	public void onClick(View v) {
		if (v == bookbutton) {
			startActivity(new Intent(TaskList.this, TaskBooking.class));
		} else if (v == cancelbutton) {
			startActivity(new Intent(TaskList.this, TaskCancel.class));
		}
	}
}
