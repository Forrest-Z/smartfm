package com.steve.smartfm;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;
import android.widget.AdapterView.OnItemSelectedListener;

public class TaskBooking extends Activity implements OnClickListener{

	//initialize variables
	Button pickupButton, destinationButton, confirmButton, cancelButton;
	Spinner pickupSp, destSp;
	private ArrayAdapter<String> aspnPickups;
	private ArrayAdapter<String> aspnDests;
	private List<String> allPickups;
	private List<String> allDests;
	
	private static final String[] pickupLocations = { "Workshop","Mcdonalds","EA","E3A"};
	private static final String[] destLocations = { "Workshop","Mcdonalds","EA","E3A"};
	
	private Double pickupLatitude = 0.0, pickupLongitude = 0.0 , destLatitude = 0.0, destLongitude = 0.0;
	int pickupOption = 0;
	int destOption = 0;
	String pickupLocation = "0";
	String destLocation = "0";
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
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
		allPickups = new ArrayList<String>();
		for (int i = 0; i < pickupLocations.length; i++) {
			allPickups.add(pickupLocations[i]);
		}
		aspnPickups = new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item, allPickups);
		aspnPickups.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		pickupSp.setAdapter(aspnPickups);

		// choose destination
		destSp = (Spinner) findViewById(R.id.spinner2);
		destSp.setOnItemSelectedListener(new DestSelectedListener());
		allDests = new ArrayList<String>();
		for (int i = 0; i < destLocations.length; i++) {
			allDests.add(destLocations[i]);
		}
		aspnDests = new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item, allDests);
		aspnDests.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		destSp.setAdapter(aspnDests);	
	}

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		// TODO Auto-generated method stub
		super.onActivityResult(requestCode, resultCode, data);
		
		//pickup result
		if(requestCode ==1 & resultCode == RESULT_OK){
			//CHANGE SPINNER
			pickupSp.setSelection(data.getIntExtra("location", 0));
			if(data.getStringExtra("location")=="MacDonald's"){
				pickupOption = 2;
				pickupLatitude = 1.298132;
				pickupLongitude = 103.771083;
				pickupLocation = "Mcdonalds";
			}else if(data.getStringExtra("location")=="Workshop"){
				pickupOption = 1;
				pickupLatitude = 1.299076;
				pickupLongitude = 103.770815;
				pickupLocation = "Workshop";
			}else if(data.getStringExtra("location")=="Block EA"){
				pickupOption = 3;
				pickupLatitude = 1.300792;
				pickupLongitude = 103.77067;
				pickupLocation = "EA";
			}else if(data.getStringExtra("location")=="Block E3A"){
				pickupOption = 4;
				pickupLatitude = 1.300524;
				pickupLongitude = 103.771459;
				pickupLocation = "E3A";
			}
		}
		
		//destination result
		if(requestCode == 2 & resultCode == RESULT_OK){
			//CHANGE SPINNER
			destSp.setSelection(data.getIntExtra("location", 0));
			if(data.getStringExtra("location")=="MacDonald's"){
				destOption = 2;
				destLatitude = 1.298132;
				destLongitude = 103.771083;
				destLocation = "Mcdonalds";
			}else if(data.getStringExtra("location")=="Workshop"){
				destOption = 1;
				destLatitude = 1.299076;
				destLongitude = 103.770815;
				destLocation = "Workshop";
			}else if(data.getStringExtra("location")=="Block EA"){
				destOption = 3;
				destLatitude = 1.300792;
				destLongitude = 103.77067;
				destLocation = "EA";
			}else if(data.getStringExtra("location")=="Block E3A"){
				destOption = 4;
				destLatitude = 1.300524;
				destLongitude = 103.771459;
				destLocation = "E3A";
			}
		}
	}

	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub
		if(v==pickupButton){
			Intent pickupIntent = new Intent(TaskBooking.this, TaskBookingMap.class);
			startActivityForResult(pickupIntent, 1);
		}else if(v==destinationButton){
			Intent destinationIntent = new Intent(TaskBooking.this, TaskBookingMap.class);
			startActivityForResult(destinationIntent, 2);
		}else if(v==confirmButton){
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
			
			Intent intent = new Intent(TaskBooking.this, MainActivity.class);
			intent.putExtras(bundle);
			startActivity(intent);
			finish();
		}else if(v==cancelButton){
			startActivity(new Intent(getApplicationContext(), MainActivity.class));
		}
	}
	
	public class PickupSelectedListener implements OnItemSelectedListener {

	    public void onItemSelected(AdapterView<?> parent,
	        View view, int pos, long id) {

	    String op = parent.getItemAtPosition(pos).toString();
	    
		    if (op=="Workshop")
		    {
		    	pickupOption = 1;
				pickupLatitude = 1.299076;
				pickupLongitude = 103.770815;
				pickupLocation = "Workshop";
		    } else if(op=="Mcdonalds")
		    {
		    	pickupOption = 2;
				pickupLatitude = 1.298132;
				pickupLongitude = 103.771083;
				pickupLocation = "Mcdonalds";
		    } else if(op=="EA")
		    {
		    	pickupOption = 3;
				pickupLatitude = 1.300792;
				pickupLongitude = 103.77067;
				pickupLocation = "EA";
		    } else if(op=="E3A")
		    {
		    	pickupOption = 4;
				pickupLatitude = 1.300524;
				pickupLongitude = 103.771459;
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
	    	destLatitude = 1.299076;
	    	destLongitude = 103.770815;
	    	destLocation = "Workshop";
	    } else if(op=="Mcdonalds")
	    {
	    	destOption = 2;
	    	destLatitude = 1.298132;
	    	destLongitude = 103.771083;
	    	destLocation = "Mcdonalds";
	    } else if(op=="EA")
	    {
	    	destOption = 3;
	    	destLatitude = 1.300792;
	    	destLongitude = 103.77067;
	    	destLocation = "EA";
	    } else if(op=="E3A")
	    {
	    	destOption = 4;
	    	destLatitude = 1.300524;
	    	destLongitude = 103.771459;
	    	destLocation = "E3A";
	    }
	    }

	    public void onNothingSelected(AdapterView<?> parent) {
	      // Do nothing.
	    }
	}
}
