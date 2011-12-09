package com.smartfm.phoneui;

import java.util.ArrayList;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;

public class TaskList extends Activity implements OnClickListener{

	TextView useridText, taskidText, pickupText, dropoffText, etaText, caridText;
	Button bookbutton, cancelbutton;
	private double dropoffLatitude = 0.0;
    private double dropoffLongitude = 0.0;
    private double pickupLatitude = 0.0;
    private double pickupLongitude = 0.0;
    private String pickupLocation, dropoffLocation;
    ArrayList<String> taskListInStringToCancel = null;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		setContentView(R.layout.info);

		bookbutton = (Button) findViewById(R.id.bookbutton);
		bookbutton.setOnClickListener(this);
		cancelbutton = (Button) findViewById(R.id.cancelbutton);
		cancelbutton.setOnClickListener(this);

		this.pickupLatitude = getIntent().getDoubleExtra("pickup_lat", 0.0);
		this.pickupLongitude = getIntent().getDoubleExtra("pickup_longi", 0.0);
		this.dropoffLatitude = getIntent().getDoubleExtra("dest_lat", 0.0);
		this.dropoffLongitude = getIntent().getDoubleExtra("dest_longi", 0.0);
		this.pickupLocation = getIntent().getStringExtra("pickup_location");
		this.dropoffLocation = getIntent().getStringExtra("dest_location");
		this.taskListInStringToCancel = getIntent().getStringArrayListExtra("tmp");

		useridText = (TextView) findViewById(R.id.useridtext);
		useridText.setText("");
		taskidText = (TextView) findViewById(R.id.taskidtext);
		taskidText.setText("");
		pickupText = (TextView) findViewById(R.id.pickupinfotext);
		pickupText.setText(pickupLocation);
		dropoffText = (TextView) findViewById(R.id.dropoffinfotext);
		dropoffText.setText(dropoffLocation);
		etaText = (TextView) findViewById(R.id.etatext);
		etaText.setText("");
		caridText = (TextView) findViewById(R.id.caridtext);
		caridText.setText("");
	}

	@Override
	protected void onPause() {
		// TODO Auto-generated method stub
		super.onPause();
	}

	@Override
	protected void onResume() {
		// TODO Auto-generated method stub
		super.onResume();
	}

	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub
		if(v==bookbutton){
			Intent intent = new Intent(TaskList.this, TaskBooking.class);
			startActivity(intent);
		}else if(v==cancelbutton){
			Intent intent = new Intent(TaskList.this, TaskCancel.class);
			intent.putStringArrayListExtra("tmp", taskListInStringToCancel);
			startActivity(intent);
		}
	}
}
