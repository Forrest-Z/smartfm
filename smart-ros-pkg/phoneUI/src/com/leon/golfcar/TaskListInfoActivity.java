package com.leon.golfcar;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public class TaskListInfoActivity extends Activity{
	private String orderInfo = "Task Information:\n";
    private List<Task> taskList = new ArrayList<Task>();
	private ArrayList<String> tasklistInfoString = new ArrayList<String>();
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		setContentView(R.layout.serviceinfo);
		setTitle("Service Information:");
		
		getTaskListInfo();
		displayTaskListinfo();
	}
	
	private void getTaskListInfo(){
		Bundle extras = getIntent().getExtras();
		
		if (extras != null)
        {	        	
        	tasklistInfoString = extras.getStringArrayList("tasklistInfoString");
        	
        	convertToTasklist(tasklistInfoString);
        //displayOrderInfo();        
        } 
	}
	
	private void convertToTasklist(ArrayList<String> tasklistInfoString){
		for (int i=0;i<tasklistInfoString.size();i++)
        {
        	Task task = new Task();
        	
        	int[] count = new int[10];
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
	
	
	private void displayTaskListinfo() {
		TextView service_status = (TextView) findViewById(R.id.order);
		
		if (taskList.size()!=0)
			for (int i=0;i<taskList.size();i++)
			{
				if (taskList.get(i).pickUpOption.compareTo("-1")!=0)
				{
					orderInfo +="+++++++++++++++++++++++++\n";
					orderInfo +="User ID: "+taskList.get(i).ID+" Task ID: "+taskList.get(i).taskID;
					orderInfo +="\nPick-up location: "+taskList.get(i).pickUpLocation
							+"\nDest location: "+taskList.get(i).dropOffLocation
							+"\nWait time: "+taskList.get(i).waitTime + " mins"
							+"\nCar ID: "+taskList.get(i).carID;
					orderInfo +="\n";
				}
			}
		else 
			orderInfo = "No Service. Please order.";
		service_status.setText(orderInfo);
	}
}
