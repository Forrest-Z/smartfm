package com.smartfm.phoneui;

import java.util.List;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.AdapterView.OnItemClickListener;

public class MainActivity extends Activity  implements OnClickListener, OnItemClickListener {
	
	ArrayAdapter<String> tasksDescriptions;
	List<Task> tasks;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.main);
		
		findViewById(R.id.makebookingbutton).setOnClickListener(this);
		findViewById(R.id.refreshbookingsbutton).setOnClickListener(this);
		
		ListView lv = (ListView) findViewById(R.id.bookingslistview);
		tasksDescriptions = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1);
		lv.setAdapter(tasksDescriptions);
		lv.setOnItemClickListener(this);
		
		updateBookings();
	}
	
	void updateBookings() {
		try {
			tasks = DBInterface.listTasks();
			tasksDescriptions.clear();
			for(Task task: tasks) {
				String description = "";
				description += "From " + task.pickup + " To " + task.dropoff + "\n";
				description += "Status: " + task.status + ", ETA=" + task.eta;
				tasksDescriptions.add(description);
			}
		} catch (Exception e) {
			// TODO report the error
		}
	}

	@Override
	public void onClick(View v) {
		switch(v.getId()) {
		
		case R.id.makebookingbutton:
			// Ask for a result, so that we can call updateBookings on returns.
			// (see onActivityResult). Actually the activity returns nothing.
			startActivityForResult(new Intent(this, TaskBooking.class), 1);
			break;
			
		case R.id.refreshbookingsbutton:
			updateBookings();
			break;

		}
	}
	
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		updateBookings();
	}
	
	@Override
    public void onItemClick(AdapterView<?> parent, View view,
	        int position, long id) {
		// A list item has been clicked: show the details of this task in
		// the TaskDetailActivity activity.
		Task task = tasks.get(position);
		Intent intent = new Intent(this, TaskDetailActivity.class);
		intent.putExtra("com.smartfm.phoneui.taskstring", task.toString());
		
		// Ask for a result, so that we can call updateBookings on returns.
		// (see onActivityResult). Actually the activity returns nothing.
		startActivityForResult(intent, 1);
    }
}