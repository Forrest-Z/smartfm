package com.smartfm.phoneui;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.Toast;
import android.widget.AdapterView.OnItemSelectedListener;

public class TaskBooking extends Activity implements OnClickListener {

	private StationList stations = new StationList();
	private Station pickupStation, dropoffStation;

	private static final int STATION_TYPE_PICKUP = 1;
	private static final int STATION_TYPE_DROPOFF = 2;

	// initialize variables
	Button pickupButton, destinationButton, confirmButton, cancelButton;
	Spinner pickupSp, destSp;
	private ArrayAdapter<String> aspnPickups;
	private ArrayAdapter<String> aspnDests;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.booking);

		pickupButton = (Button) findViewById(R.id.pickupmapbutton);
		destinationButton = (Button) findViewById(R.id.destinationmapbutton);
		confirmButton = (Button) findViewById(R.id.confirmbutton);
		cancelButton = (Button) findViewById(R.id.cancelbutton);
		pickupButton.setOnClickListener(this);
		destinationButton.setOnClickListener(this);
		confirmButton.setOnClickListener(this);
		cancelButton.setOnClickListener(this);

		// choose pickup
		pickupSp = (Spinner) findViewById(R.id.spinner1);
		pickupSp.setOnItemSelectedListener(new PickupSelectedListener());
		aspnPickups = new ArrayAdapter<String>(this,
				android.R.layout.simple_spinner_item, stations.allNames());
		aspnPickups
				.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		pickupSp.setAdapter(aspnPickups);

		// choose destination
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
				String stationName = data
						.getStringExtra("com.smartfm.phoneui.stationName");
				pickupSp.setSelection(stations.getStationNumber(stationName));
				pickupStation = stations.getStation(stationName);
			} else if (requestCode == STATION_TYPE_DROPOFF) {
				String stationName = data
						.getStringExtra("com.smartfm.phoneui.stationName");
				destSp.setSelection(stations.getStationNumber(stationName));
				dropoffStation = stations.getStation(stationName);
			}
		}
	}

	@Override
	public void onClick(View v) {
		if (v == pickupButton) {
			startActivityForResult(new Intent(TaskBooking.this,
					TaskBookingMap.class), STATION_TYPE_PICKUP);
		} else if (v == destinationButton) {
			startActivityForResult(new Intent(TaskBooking.this,
					TaskBookingMap.class), STATION_TYPE_DROPOFF);
		} else if (v == confirmButton) {
			if (pickupStation.name == dropoffStation.name) {
				Context context = getApplicationContext();
				CharSequence text = "Service cannot be delivered. Pick-up location and Drop-off location cannot be the same.";
				Toast toast = Toast.makeText(context, text,
						Toast.LENGTH_LONG);
				toast.show();
			} else {
				try {
					DBInterface.addTask(pickupStation.name, dropoffStation.name);
				} catch (Exception e) {
					Context context = getApplicationContext();
					CharSequence text = "Error while creating the task (RPC call failed).";
					Toast toast = Toast.makeText(context, text, Toast.LENGTH_LONG);
					toast.show();
				}
				startActivity(new Intent(this, MainActivity.class));
				finish();
			}
		} else if (v == cancelButton) {
			startActivity(new Intent(this,
					MainActivity.class));
		}
	}

	public class PickupSelectedListener implements OnItemSelectedListener {
		public void onItemSelected(AdapterView<?> parent, View view, int pos,
				long id) {
			String op = parent.getItemAtPosition(pos).toString();
			pickupStation = stations.getStation(op);
		}

		public void onNothingSelected(AdapterView<?> parent) {
			// Do nothing.
		}
	}

	public class DestSelectedListener implements OnItemSelectedListener {
		public void onItemSelected(AdapterView<?> parent, View view, int pos,
				long id) {
			String op = parent.getItemAtPosition(pos).toString();
			dropoffStation = stations.getStation(op);
		}

		public void onNothingSelected(AdapterView<?> parent) {
			// Do nothing.
		}
	}
}
