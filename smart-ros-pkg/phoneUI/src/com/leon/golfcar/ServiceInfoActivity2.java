package com.leon.golfcar;

import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;

public class ServiceInfoActivity2 extends Activity{
	private String orderInfo = "Task Information:\n";
    private List<ServiceMsg> userOrders = new ArrayList<ServiceMsg>();
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
        	ServiceMsg s = new  ServiceMsg();
        	
        	int[] count;
        	count = new int[10];
        	count[0] = 0;
        	for (int j=1;j<8;j++)
        		count[j] = tmp.get(i).indexOf(":",count[j-1]+1);

        	s.ID = tmp.get(i).substring(count[0], count[1]);
        	s.taskID = tmp.get(i).substring(count[1]+1,count[2]);
        	s.pickup_op = tmp.get(i).substring(count[2]+1, count[3]);
        	s.dest_op = tmp.get(i).substring(count[3]+1, count[4]);
        	s.wait = tmp.get(i).substring(count[4]+1, count[5]);
        	s.carID = tmp.get(i).substring(count[5]+1, count[6]);
        	s.pickup_location = tmp.get(i).substring(count[6]+1, count[7]);
        	s.dest_location = tmp.get(i).substring(count[7]+1);
        	
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
				orderInfo +="+++++++++++++++++++++++++\n";
				orderInfo +="User ID: "+userOrders.get(i).ID+" Task ID: "+userOrders.get(i).taskID;
				orderInfo +="\nPick-up location: "+userOrders.get(i).pickup_location
					 +"\nDest location: "+userOrders.get(i).dest_location
					 +"\nWait time: "+userOrders.get(i).wait + "mins"
					 +"\nCar ID: "+userOrders.get(i).carID;
				orderInfo +="\n";
			}
		else 
			orderInfo = "No Service. Please order.";
		service_status.setText(orderInfo);
	}
}
