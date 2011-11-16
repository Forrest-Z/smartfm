package com.leon.golfcar;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public class TaskListInfoActivity extends Activity{
	private String orderInfo = "Task Information:\n";
    private List<Task> userOrders = new ArrayList<Task>();
	private ArrayList<String> tmp = new ArrayList<String>();
	
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// TODO Auto-generated method stub
		super.onCreate(savedInstanceState);
		setContentView(R.layout.serviceinfo);
		setTitle("Service Information:");
		Bundle extras = getIntent().getExtras();

        if (extras != null)
        {	        	
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

        displayOrderInfo();        
        } 
	}
	
	private void displayOrderInfo() {
		TextView service_status = (TextView) findViewById(R.id.order);
		
		if (userOrders.size()!=0)
			for (int i=0;i<userOrders.size();i++)
			{
				if (userOrders.get(i).pickupOption.compareTo("-1")!=0)
				{
					orderInfo +="+++++++++++++++++++++++++\n";
					orderInfo +="User ID: "+userOrders.get(i).ID+" Task ID: "+userOrders.get(i).taskID;
					orderInfo +="\nPick-up location: "+userOrders.get(i).pickupLocation
						 +"\nDest location: "+userOrders.get(i).dropoffLocation
						 +"\nWait time: "+userOrders.get(i).waitTime + " mins"
						 +"\nCar ID: "+userOrders.get(i).carID;
					orderInfo +="\n";
				}
			}
		else 
			orderInfo = "No Service. Please order.";
		service_status.setText(orderInfo);
	}
}
