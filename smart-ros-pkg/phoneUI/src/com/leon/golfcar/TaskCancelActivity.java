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

public class TaskCancelActivity extends Activity {
	Button confirmButton;
	OnClickListener dataTransferListener = null;
	Spinner cancelSp;
	private List<String> allTasks;
	private ArrayAdapter<String> aspnTasks;
	private int cancelTaskID = 0;
	
	
	private List<Task> userOrders = new ArrayList<Task>();
	private ArrayList<String> tmp = new ArrayList<String>();
	
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setTitle("CancelTaskActivity");
		setContentView(R.layout.spinnercanceltask);
		
		
		Bundle extras = getIntent().getExtras();

        if (extras != null)
        {
        	//totalTaskNumber = extras.getInt("totalTaskNumber");
        	
        	tmp = extras.getStringArrayList("tmp");
        	
            for (int i=0;i<tmp.size();i++)
            {
            	Task s = new  Task();
            	
            	int[] count;
            	count = new int[10];
            	count[0] = 0;
            	for (int j=1;j<8;j++)
            		count[j] = tmp.get(i).indexOf(":",count[j-1]+1);

            	s.ID = tmp.get(i).substring(count[0], count[1]);
            	s.taskID = tmp.get(i).substring(count[1]+1,count[2]);
            	s.pickupOption = tmp.get(i).substring(count[2]+1, count[3]);
            	s.dropoffOption = tmp.get(i).substring(count[3]+1, count[4]);
            	s.waitTime = tmp.get(i).substring(count[4]+1, count[5]);
            	s.carID = tmp.get(i).substring(count[5]+1, count[6]);
            	s.pickupLocation = tmp.get(i).substring(count[6]+1, count[7]);
            	s.dropoffLocation = tmp.get(i).substring(count[7]+1);
            	
            	userOrders.add(s);
            }
        	
        }
		cancelTask();
		transferData();	
	}
	
	private void cancelTask() {
	
		cancelSp = (Spinner) findViewById(R.id.spinnerCancel);
		cancelSp.setOnItemSelectedListener(new CancelListener());
		allTasks = new ArrayList<String>();
		
		if (userOrders.size()==0)
			allTasks.add("No task to cancel");
		else
			for (int i=0;i<userOrders.size();i++) {
				if (userOrders.get(i).pickupOption.compareTo("-1")!=0)
					//allTasks.add("Task ID: " + userOrders.get(i).taskID + "Pick-up: " 
						//	+ userOrders.get(i).pickup_location + "Drop-off: " + userOrders.get(i).dest_location);
					allTasks.add("Task ID: " + userOrders.get(i).taskID);
					
			}
	
	
		
		aspnTasks = new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item, allTasks);
		aspnTasks.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		cancelSp.setAdapter(aspnTasks);
		
	}

	private void transferData() {
		// version 2, transfer data
        dataTransferListener = new OnClickListener() {
			public void onClick(View v) {
				// Intent: change from one Activity to another. Activity->ActivityFrameLayout
				Bundle bundle = new Bundle();				
				bundle.putInt("actionType", 1);
				bundle.putInt("cancelTaskID", cancelTaskID);
				Intent intent = new Intent(TaskCancelActivity.this, MainActivity.class);
				intent.putExtras(bundle);
				setResult(RESULT_OK,intent);
				finish();
			}
		};  
		
		confirmButton = (Button) findViewById(R.id.ConfirmCancel);
		confirmButton.setOnClickListener(dataTransferListener);
	}
	
	
	public class CancelListener implements OnItemSelectedListener {

	    public void onItemSelected(AdapterView<?> parent,
	        View view, int pos, long id) {
	    String op = parent.getItemAtPosition(pos).toString();

	    
	    for (int i=0;i<userOrders.size();i++){
	    	if (op.compareTo("Task ID: "+userOrders.get(i).taskID)==0)
	    		cancelTaskID = i+1;
	    }
	    
	    
	    
	    }

	    public void onNothingSelected(AdapterView<?> parent) {
	      // Do nothing.
	    }
	}
}