package com.smartfm.phoneui;

import java.util.ArrayList;
import java.util.List;

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

public class TaskCancel extends Activity {
	
	//TODO: find another display than a spinner, for instance a 
	// list with a summary / cancel / info button for each item.

	Button confirmButton;
	Spinner cancelSp;
	
	List<Task> tasks = new ArrayList<Task>();
	int cancelTaskID = -1;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setTitle("CancelTaskActivity");
		setContentView(R.layout.spinnercanceltask);
		createTaskSpinner();
		createConfirmButton();
		updateTaskList();		
	}
	
	void createTaskSpinner() {
		cancelSp = (Spinner) findViewById(R.id.spinnerCancel);
		cancelSp.setOnItemSelectedListener( new OnItemSelectedListener() {
			public void onItemSelected(AdapterView<?> parent, View view, int pos,
					long id) {
				String op = parent.getItemAtPosition(pos).toString();
				for (Task task: tasks)
					if (op.compareTo("" + task.requestID) == 0)
						cancelTaskID = task.requestID;
			}

			public void onNothingSelected(AdapterView<?> parent) {
				// Do nothing.
			}
		});
	}
	
	void createConfirmButton() {
		confirmButton = (Button) findViewById(R.id.ConfirmCancel);
		confirmButton.setOnClickListener( new OnClickListener() {
			public void onClick(View v) {
				if( cancelTaskID>=0 ) {
					try {
						DBInterface.cancelTask(cancelTaskID);
					} catch (Exception e) {
						Context context = getApplicationContext();
						CharSequence text = "Error while cancelling the task (RPC call failed).";
						Toast toast = Toast.makeText(context, text, Toast.LENGTH_LONG);
						toast.show();
					}
				}
				startActivity(new Intent(TaskCancel.this, MainActivity.class));
			}
		});
	}
	
	void updateTaskList() {
		try {
			tasks = DBInterface.listTasks();
		} catch (Exception e) {
			Context context = getApplicationContext();
			CharSequence text = "Error while retrieving your bookings (RPC call failed).";
			Toast toast = Toast.makeText(context, text, Toast.LENGTH_LONG);
			toast.show();
		}
		
		List<String> allTasks = new ArrayList<String>();
		if (tasks.isEmpty()) {
			allTasks.add("No task to cancel");
		} else {
			for (Task task: tasks) {
				// TODO: display more that just the ID.
				allTasks.add("" + task.requestID);
			}
		}
		ArrayAdapter<String> aspnTasks = new ArrayAdapter<String>(this,
				android.R.layout.simple_spinner_item, allTasks);
		aspnTasks.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);		
		cancelSp.setAdapter(aspnTasks);
	}
}
