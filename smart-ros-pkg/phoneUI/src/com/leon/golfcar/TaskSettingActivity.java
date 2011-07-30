package com.leon.golfcar;

import java.util.ArrayList;
import java.util.List;


import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;

public class TaskSettingActivity extends Activity {
	Spinner pickupSp;
	Spinner destSp;
	private ArrayAdapter<String> aspnPickups;
	private ArrayAdapter<String> aspnDests;
	private List<String> allPickups;
	private List<String> allDests;

	private Double pickupLatitude = 0.0, pickupLongitude = 0.0 , destLatitude = 0.0, destLongitude = 0.0;
	int pickupOption = 0;
	int destOption = 0;
	String pickupLocation = "0";
	String destLocation = "0";
	
	Button confirmButton;
	OnClickListener dataTransferListener = null;
	private int actionType = 0;
	
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setTitle("BookCarActivity");
		setContentView(R.layout.spinner);
		choosePickupAndDest();
		transferData();
	}

	private static final String[] pickupLocations = { "Workshop","Mcdonalds","EA","E3A"};
	
	private static final String[] destLocations = { "Workshop","Mcdonalds","EA","E3A"};
	
	
	private void choosePickupAndDest() {
		// choose pickup
		pickupSp = (Spinner) findViewById(R.id.spinner_1);
		pickupSp.setOnItemSelectedListener(new PickupSelectedListener());
		allPickups = new ArrayList<String>();
		for (int i = 0; i < pickupLocations.length; i++) {
			allPickups.add(pickupLocations[i]);
		}
		aspnPickups = new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item, allPickups);
		aspnPickups.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		pickupSp.setAdapter(aspnPickups);

		// choose destination
		destSp = (Spinner) findViewById(R.id.spinner_2);
		destSp.setOnItemSelectedListener(new DestSelectedListener());
		allDests = new ArrayList<String>();
		for (int i = 0; i < destLocations.length; i++) {
			allDests.add(destLocations[i]);
		}
		aspnDests = new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item, allDests);
		aspnDests.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		destSp.setAdapter(aspnDests);		
	}

	private void transferData() {
		// version 2, transfer data
        dataTransferListener = new OnClickListener() {
			public void onClick(View v) {
				// Intent: change from one Activity to another. Activity->ActivityFrameLayout
				Bundle bundle = new Bundle();
				bundle.putInt("actionType", 2);
				bundle.putInt("pickup_op", pickupOption);
				bundle.putInt("dest_op", destOption);
				bundle.putString("pickup_location", pickupLocation);
				bundle.putString("dest_location", destLocation);
				bundle.putDouble("pickup_lat", pickupLatitude);
				bundle.putDouble("pickup_longi", pickupLongitude);
				bundle.putDouble("dest_lat", destLatitude);
				bundle.putDouble("dest_longi", destLongitude);
				
				Intent intent = new Intent(TaskSettingActivity.this, MainActivity.class);
				intent.putExtras(bundle);
				setResult(RESULT_OK,intent);
				finish();
			}
		};  
		
		confirmButton = (Button) findViewById(R.id.Confirm);
		confirmButton.setOnClickListener(dataTransferListener);
	}
	
	public class PickupSelectedListener implements OnItemSelectedListener {

	    public void onItemSelected(AdapterView<?> parent,
	        View view, int pos, long id) {

	    String op = parent.getItemAtPosition(pos).toString();
	    
		    if (op=="Workshop")
		    {
		    	pickupOption = 1;
				pickupLatitude = 1.299292;
				pickupLongitude = 103.770596;
				pickupLocation = "Workshop";
		    } else if(op=="Mcdonalds")
		    {
		    	pickupOption = 2;
				pickupLatitude = 1.298247;
				pickupLongitude = 103.771121;
				pickupLocation = "Mcdonalds";
		    } else if(op=="EA")
		    {
		    	pickupOption = 3;
				pickupLatitude = 1.300997;
				pickupLongitude = 103.770694;
				pickupLocation = "EA";
		    } else if(op=="E3A")
		    {
		    	pickupOption = 4;
				pickupLatitude = 1.300775;
				pickupLongitude = 103.771247;
				pickupLocation = "E3A";
		    }
	    }

	    public void onNothingSelected(AdapterView<?> parent) {
	      // Do nothing.
	    }
	}
	
	public class DestSelectedListener implements OnItemSelectedListener {

	    public void onItemSelected(AdapterView<?> parent,
	        View view, int pos, long id) {
	    String op = parent.getItemAtPosition(pos).toString();
	    
	    if (op=="Workshop")
	    {
	    	destOption = 1;
	    	destLatitude = 1.299292;
	    	destLongitude = 103.770596;
	    	destLocation = "Workshop";
	    } else if(op=="Mcdonalds")
	    {
	    	destOption = 2;
	    	destLatitude = 1.298247;
	    	destLongitude = 103.771121;
	    	destLocation = "Mcdonalds";
	    } else if(op=="EA")
	    {
	    	destOption = 3;
	    	destLatitude = 1.300997;
	    	destLongitude = 103.770694;
	    	destLocation = "EA";
	    } else if(op=="E3A")
	    {
	    	destOption = 4;
	    	destLatitude = 1.300775;
	    	destLongitude = 103.771247;
	    	destLocation = "E3A";
	    }
	    }

	    public void onNothingSelected(AdapterView<?> parent) {
	      // Do nothing.
	    }
	}
	
}