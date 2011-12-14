package com.smartfm.phoneui;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Spinner;
import android.widget.Toast;
import android.widget.AdapterView.OnItemSelectedListener;

public class TaskBooking extends Activity implements OnClickListener {

	StationList stations = new StationList();
	String pickupStation, dropoffStation;

	static final int STATION_TYPE_PICKUP = 1;
	static final int STATION_TYPE_DROPOFF = 2;

	// initialize variables
	ArrayAdapter<String> aspnPickups;
	ArrayAdapter<String> aspnDests;
	Spinner pickupSp, destSp;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.booking);

		findViewById(R.id.pickupmapbutton).setOnClickListener(this);
		findViewById(R.id.destinationmapbutton).setOnClickListener(this);
		findViewById(R.id.confirmbutton).setOnClickListener(this);
		findViewById(R.id.cancelbutton).setOnClickListener(this);

		// pickup spinner
		pickupSp = (Spinner) findViewById(R.id.spinner1);
		pickupSp.setOnItemSelectedListener(new PickupSelectedListener());
		aspnPickups = new ArrayAdapter<String>(this,
				android.R.layout.simple_spinner_item, stations.allNames());
		aspnPickups
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		pickupSp.setAdapter(aspnPickups);

		// destination spinner
		destSp = (Spinner) findViewById(R.id.spinner2);
		destSp.setOnItemSelectedListener(new DestSelectedListener());
		aspnDests = new ArrayAdapter<String>(this,
				android.R.layout.simple_spinner_item, stations.allNames());
		aspnDests
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		destSp.setAdapter(aspnDests);
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		super.onActivityResult(requestCode, resultCode, data);
		if (resultCode == RESULT_OK) {
			if (requestCode == STATION_TYPE_PICKUP) {
				pickupStation = data
						.getStringExtra("com.smartfm.phoneui.stationName");
				pickupSp.setSelection(stations.getStationNumber(pickupStation));
			} else if (requestCode == STATION_TYPE_DROPOFF) {
				dropoffStation = data
						.getStringExtra("com.smartfm.phoneui.stationName");
				destSp.setSelection(stations.getStationNumber(dropoffStation));
			}
		}
	}

	@Override
	public void onClick(View v) {
		switch(v.getId()) {
		
		case R.id.pickupmapbutton:
			startActivityForResult(
					new Intent(this,TaskBookingMap.class), 
					STATION_TYPE_PICKUP);
			break;
			
		case R.id.destinationmapbutton:
			startActivityForResult(
					new Intent(this,TaskBookingMap.class), 
					STATION_TYPE_DROPOFF);
		
		case R.id.confirmbutton:
			if ( pickupStation.compareTo(dropoffStation)==0 ) {
				Context context = getApplicationContext();
				CharSequence text = "Service cannot be delivered. Pick-up location and Drop-off location cannot be the same.";
				Toast toast = Toast.makeText(context, text,
						Toast.LENGTH_LONG);
				toast.show();
			} else {
				try {
					DBInterface.addTask(pickupStation, dropoffStation);
				} catch (Exception e) {
					Context context = getApplicationContext();
					CharSequence text = "Error while creating the task (RPC call failed).";
					Toast toast = Toast.makeText(context, text, Toast.LENGTH_LONG);
					toast.show();
				}
				finish();
			}
			break;
			
		case R.id.cancelbutton:
			finish();
		}
	}

	public class PickupSelectedListener implements OnItemSelectedListener {
		public void onItemSelected(AdapterView<?> parent, View view, int pos,
				long id) {
			pickupStation = parent.getItemAtPosition(pos).toString();
		}

		public void onNothingSelected(AdapterView<?> parent) {
			// Do nothing.
		}
	}

	public class DestSelectedListener implements OnItemSelectedListener {
		public void onItemSelected(AdapterView<?> parent, View view, int pos,
				long id) {
			dropoffStation = parent.getItemAtPosition(pos).toString();
		}

		public void onNothingSelected(AdapterView<?> parent) {
			// Do nothing.
		}
	}
}
