package com.smartfm.phoneui;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.AdapterView.OnItemSelectedListener;

public class TaskBooking extends Activity implements OnClickListener {

	StationList stations;
	String pickupStation=null, dropoffStation=null;

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

		findViewById(R.id.pickupbutton).setOnClickListener(this);
		findViewById(R.id.pickupmapbutton).setOnClickListener(this);
		findViewById(R.id.dropoffbutton).setOnClickListener(this);
		findViewById(R.id.destinationmapbutton).setOnClickListener(this);
		findViewById(R.id.confirmbutton).setOnClickListener(this);
		findViewById(R.id.cancelbutton).setOnClickListener(this);

		stations = new StationList();
		aspnPickups = new ArrayAdapter<String>(this,
				android.R.layout.simple_spinner_dropdown_item, stations.allNames());
		aspnDests = new ArrayAdapter<String>(this,
				android.R.layout.simple_spinner_dropdown_item, stations.allNames());
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		super.onActivityResult(requestCode, resultCode, data);
		if (resultCode == RESULT_OK) {
			if (requestCode == STATION_TYPE_PICKUP) {
				pickupStation = data
						.getStringExtra("com.smartfm.phoneui.stationName");
				((Button)findViewById(R.id.pickupbutton)).setText(pickupStation);
			} else if (requestCode == STATION_TYPE_DROPOFF) {
				dropoffStation = data
						.getStringExtra("com.smartfm.phoneui.stationName");
				((Button)findViewById(R.id.dropoffbutton)).setText(dropoffStation);
			}
		}
	}

	@Override
	public void onClick(View v) {
		switch(v.getId()) {
		
		case R.id.pickupbutton:
			new AlertDialog.Builder(this)
  				.setTitle("Pick a station")
  				.setAdapter(aspnPickups, new DialogInterface.OnClickListener() {

				@Override
				public void onClick(DialogInterface dialog, int which) {
			    	pickupStation = aspnPickups.getItem(which);
			    	((Button)findViewById(R.id.pickupbutton)).setText(pickupStation);
					dialog.dismiss();
			    }
			}).create().show();
			break;

		case R.id.dropoffbutton:
			new AlertDialog.Builder(this)
  				.setTitle("Pick a station")
  				.setAdapter(aspnDests, new DialogInterface.OnClickListener() {

				@Override
				public void onClick(DialogInterface dialog, int which) {
			    	dropoffStation = aspnDests.getItem(which);
			    	((Button)findViewById(R.id.dropoffbutton)).setText(dropoffStation);
					dialog.dismiss();
			    }
			}).create().show();
			break;
		
		case R.id.pickupmapbutton:
			startActivityForResult(
					new Intent(TaskBooking.this,TaskBookingMap.class), 
					STATION_TYPE_PICKUP);
			break;
			
		case R.id.destinationmapbutton:
			startActivityForResult(
					new Intent(TaskBooking.this,TaskBookingMap.class), 
					STATION_TYPE_DROPOFF);
			break;
		
		case R.id.confirmbutton:
			ProgressDialog dialog = ProgressDialog.show(TaskBooking.this, "", 
                    "Loading. Please wait...", true);
			dialog.show();
			if( pickupStation==null ){
				dialog.cancel();
				ErrDialog.show(this, "Choose a pickup station first!");}
			else if( dropoffStation==null ){
				dialog.cancel();
				ErrDialog.show(this, "Choose a dropoff station first!");}
			else if ( pickupStation.compareTo(dropoffStation)==0 ){
				dialog.cancel();
				ErrDialog.show(this, "Pick-up location and Drop-off location cannot be the same!");}
			else {
				try {
					DBInterface.addTask(pickupStation, dropoffStation);
				} catch (Exception e) {
					ErrDialog.show(this, "Error while creating the task (RPC call failed).\n" + e);
				}
				finish();
			}
			break;
			
		case R.id.cancelbutton:
			finish();
			break;
		}
	}

	public class PickupSelectedListener implements OnItemSelectedListener {
		public void onItemSelected(AdapterView<?> parent, View view, int pos,
				long id) {
			pickupStation = parent.getItemAtPosition(pos).toString();
			((Button)findViewById(R.id.pickupbutton)).setText(pickupStation);
		}

		public void onNothingSelected(AdapterView<?> parent) {
			// Do nothing.
		}
	}

	public class DestSelectedListener implements OnItemSelectedListener {
		public void onItemSelected(AdapterView<?> parent, View view, int pos,
				long id) {
			dropoffStation = parent.getItemAtPosition(pos).toString();
			((Button)findViewById(R.id.dropoffbutton)).setText(dropoffStation);
		}

		public void onNothingSelected(AdapterView<?> parent) {
			// Do nothing.
		}
	}
}
