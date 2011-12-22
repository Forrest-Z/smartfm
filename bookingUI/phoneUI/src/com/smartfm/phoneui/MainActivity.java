package com.smartfm.phoneui;

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
import android.widget.ListView;
import android.widget.TextView;
import android.widget.AdapterView.OnItemClickListener;

public class MainActivity extends Activity implements OnClickListener,
		OnItemClickListener {

	ArrayAdapter<String> tasksDescriptions = null;
	List<Task> tasks = null;

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.main);

		((Button) findViewById(R.id.makebookingbutton))
				.setOnClickListener(this);
		((Button) findViewById(R.id.refreshbookingsbutton))
				.setOnClickListener(this);

		tasks = new ArrayList<Task>();
		tasksDescriptions = new ArrayAdapter<String>(this,
				android.R.layout.simple_list_item_1);
		updateBookings();
		
		ListView lv = (ListView) findViewById(R.id.bookingslistview);		
		lv.setAdapter(tasksDescriptions);
		lv.setOnItemClickListener(this);
	}

	void updateBookings() {
		try {
			List<Task> tasks_ = DBInterface.listTasks();
			tasks.clear();
			tasksDescriptions.clear();
			for (Task task : tasks_) {
				if (task.status.compareToIgnoreCase("Cancelled") != 0
						&& task.status.compareToIgnoreCase("Completed") != 0) {
					tasks.add(task);
					String description = "";
					description += "From " + task.pickup + " To "
							+ task.dropoff + "\n";
					description += "Status: " + task.status;
					if( task.status.compareToIgnoreCase("Acknowledged") == 0 ||
							task.status.compareToIgnoreCase("Confirmed") == 0 ||
							task.status.compareToIgnoreCase("Processing") == 0 )
						description += ", ETA=" + task.eta;
					tasksDescriptions.add(description);
				}
			}

			findViewById(R.id.nocurrentbookings).setVisibility(
					tasks.isEmpty() ? View.VISIBLE : View.GONE);

		} catch (Exception e) {
			ErrDialog.show(this, "An error occured while retrieving the current bookings.\n"+e);
		}
	}

	@Override
	public void onClick(View v) {
		switch (v.getId()) {

		case R.id.makebookingbutton:
			// Ask for a result, so that we can call updateBookings on returns.
			// (see onActivityResult). Actually the activity returns nothing.
			startActivityForResult(new Intent(MainActivity.this, TaskBooking.class), 1);
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
	public void onItemClick(AdapterView<?> parent, View view, int position,
			long id) {
		// A list item has been clicked: show the details of this task in
		// the TaskDetailActivity activity.
		Task task = tasks.get(position);
		Intent intent = new Intent(MainActivity.this, TaskDetailActivity.class);
		intent.putExtra("com.smartfm.phoneui.taskstring", task.toString());

		// Ask for a result, so that we can call updateBookings on returns.
		// (see onActivityResult). Actually the activity returns nothing.
		startActivityForResult(intent, 1);
	}
}