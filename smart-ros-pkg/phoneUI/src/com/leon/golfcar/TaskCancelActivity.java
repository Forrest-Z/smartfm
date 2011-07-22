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
	private Button confirmButton;
	private OnClickListener dataTransferListener = null;
	private Spinner cancelSp;
	private List<String> allTasks;
	private ArrayAdapter<String> aspnTasks;
	private int cancelTaskID = 0;
	private List<Task> taskList = new ArrayList<Task>();
	private ArrayList<String> tasklistInfoString = new ArrayList<String>();
	
	
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setTitle("CancelTaskActivity");
		setContentView(R.layout.spinnercanceltask);
		
		cancelTask();
	}
	
	private void cancelTask(){

		Bundle extras = getIntent().getExtras();

        if (extras != null)
        	receiveDataFromMain(extras);
        
		displayTaskListSpinner();
		sendCancelTaskToMain();	
	}
	
	private void receiveDataFromMain(Bundle extras){
    	tasklistInfoString = extras.getStringArrayList("tmp");
    	
    	convertToTasklist(tasklistInfoString);
	}
	
	private void convertToTasklist(ArrayList<String> tasklistInfoString){
		for (int i=0;i<tasklistInfoString.size();i++)
        {
        	Task task = new Task();
        	
        	int[] count = new int[10];
        	//count = new int[10];
        	count[0] = 0;
        	for (int j=1;j<8;j++)
        		count[j] = tasklistInfoString.get(i).indexOf(":",count[j-1]+1);

        	task.ID = tasklistInfoString.get(i).substring(count[0], count[1]);
        	task.taskID = tasklistInfoString.get(i).substring(count[1]+1,count[2]);
        	task.pickUpOption = tasklistInfoString.get(i).substring(count[2]+1, count[3]);
        	task.dropOffOption = tasklistInfoString.get(i).substring(count[3]+1, count[4]);
        	task.waitTime = tasklistInfoString.get(i).substring(count[4]+1, count[5]);
        	task.carID = tasklistInfoString.get(i).substring(count[5]+1, count[6]);
        	task.pickUpLocation = tasklistInfoString.get(i).substring(count[6]+1, count[7]);
        	task.dropOffLocation = tasklistInfoString.get(i).substring(count[7]+1);
        	
        	taskList.add(task);
        }
	}
	
	private void displayTaskListSpinner() {
		cancelSp = (Spinner) findViewById(R.id.spinnerCancel);
		cancelSp.setOnItemSelectedListener(new CancelListener());
		allTasks = new ArrayList<String>();
		
		if (taskList.size()==0)
			allTasks.add("No task to cancel");
		else
			for (int i=0;i<taskList.size();i++) {
				if (taskList.get(i).pickUpOption.compareTo("-1")!=0)
					allTasks.add("Task ID: " + taskList.get(i).taskID);
			}
	
		aspnTasks = new ArrayAdapter<String>(this,android.R.layout.simple_spinner_item, allTasks);
		aspnTasks.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
		cancelSp.setAdapter(aspnTasks);
	}

	private void sendCancelTaskToMain() {
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
		    
		    for (int i=0;i<taskList.size();i++){
		    	if (op.compareTo("Task ID: "+taskList.get(i).taskID)==0)
		    		cancelTaskID = i+1;
		    }
	    }

	    public void onNothingSelected(AdapterView<?> parent) {
	      // Do nothing.
	    }
	}
}