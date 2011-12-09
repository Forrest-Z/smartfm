package com.smartfm.phoneui;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.lang.reflect.Method;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import android.app.TabActivity;
import android.content.Context;
import android.content.Intent;
import android.content.res.Resources;
import android.os.Bundle;
import android.widget.TabHost;
import android.widget.Toast;

public class MainActivity extends TabActivity {

    private double dropoffLatitude = 0.0;
    private double dropoffLongitude = 0.0;
    private double pickupLatitude = 0.0;
    private double pickupLongitude = 0.0;
    private String pickupLocation, dropoffLocation;
    private boolean isSetTask = false;
    private boolean isInitNetwork = false;
    private int userID;
    private int taskID = 1;

    private int cancelTaskID = 0;
    private boolean isCancelTask = false;

//    private boolean isNetworkConnectted = false;

//    final private int REQUEST_CODE = 1;

    private static final String REMOTE_HOSTNAME = "172.17.38.204";

    private static String LOCAL_HOSTNAME = "172.17.184.92";
    private static final int LOCAL_SERVERPORT = 4440;
    private static final String TASK_CANCEL_CONFIRM = "-6";
    private static final String TASK_CANCEL_INVALID = "-5";
    private static final String PICKUP_CANCEL_CONFIRM = "-1";
    private static final String DROPOFF_CANCEL_CONFIRM = "-1";

    private ServerSocket serverSocket = null;

    private List<Task> taskList = new ArrayList<Task>();


    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle bundle) {
        super.onCreate(bundle);
        setContentView(R.layout.main);

        getBookingData();
        setUpTabs();
        initializeUserIp();
        initializeUserID();
        runNetworkListener();
    }

    private void getBookingData() {
		// TODO Auto-generated method stub
    	Bundle extras = getIntent().getExtras();

    	if (extras != null)
		{
			if (extras.getInt("actionType")==1)
			{
				cancelTaskID = extras.getInt("cancelTaskID");

				if(cancelTaskID!=0)
				{
					isCancelTask = true;
				}
			}else {

		//ServiceMsg
		//get data from BookCarActivity.java
		pickupLatitude = extras.getDouble("pickup_lat");
		pickupLongitude = extras.getDouble("pickup_longi");
		dropoffLatitude = extras.getDouble("dest_lat");
		dropoffLongitude = extras.getDouble("dest_longi");
		pickupLocation = extras.getString("pickup_location");
		dropoffLocation = extras.getString("dest_location");

		if (extras.getInt("pickup_op")!=extras.getInt("dest_op"))
		{
			// add new task to services
			Task s = new Task(Integer.toString(userID),Integer.toString(taskID++),Integer.toString(extras.getInt("pickup_op")),
							  Integer.toString(extras.getInt("dest_op")),"--","--",extras.getString("pickup_location"),extras.getString("dest_location"));
			taskList.add(s);

			isSetTask = true;
		} else {
			Context context = getApplicationContext();
        	CharSequence text = "Service cannot be deliveried. Pick-up location and Drop-off location cannot be the same.";
        	int duration = Toast.LENGTH_LONG;

        	Toast toast = Toast.makeText(context, text, duration);
        	toast.show();
			}
			}
		}
    }

	@Override
	protected void onStop() {
		// TODO Auto-generated method stub
		super.onStop();
		try {
			// make sure you close the socket upon exiting
			serverSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private void runNetworkListener() {
		// TODO Auto-generated method stub
    	if (LOCAL_HOSTNAME!= null) {
			Thread fst = new Thread(new NetworkListener());
			fst.start();
		} else
			displayNetworkError();
	}

	private void displayNetworkError() {
		// TODO Auto-generated method stub
		String errMsg = "Attention\n"
			+ "A network error has occured. Please exit the app, and try again.";

		for (int i=0;i<1;i++)
			Toast.makeText(MainActivity.this, errMsg, Toast.LENGTH_LONG).show();
		}

	private void initializeUserID() {
		// TODO Auto-generated method stub
		String serialNumberOfPhone = null;

		try {
		    Class<?> c = Class.forName("android.os.SystemProperties");
		    Method get = c.getMethod("get", String.class);
		    serialNumberOfPhone = (String) get.invoke(c, "ro.serialno");
		} catch (Exception ignored) {
		}

		userID = generateUserID(serialNumberOfPhone);
	}

	private int generateUserID(String serialNumberOfPhone) {
		// TODO Auto-generated method stub
		return serialNumberOfPhone.hashCode() > 0 ? serialNumberOfPhone.hashCode() : -serialNumberOfPhone.hashCode();
	}

	private void setUpTabs(){
    	 Resources res = getResources(); // Resource object to get Drawables
         TabHost tabHost = getTabHost();  // The activity TabHost
         TabHost.TabSpec spec;  // Resusable TabSpec for each tab
         Intent intent;  // Reusable Intent for each tab

         // Create an Intent to launch an Activity for the tab (to be reused)
         Bundle information = new Bundle();
         information.putDouble("pickup_lat", pickupLatitude);
         information.putDouble("pickup_longi", pickupLongitude);
         information.putDouble("dest_lat", dropoffLatitude);
         information.putDouble("dest_longi", dropoffLongitude);
         information.putString("pickup_location", pickupLocation);
         information.putString("dest_location", dropoffLocation);
         ArrayList<String> taskListInStringToCancel = null;
         taskListInStringToCancel = convertTaskListToStringFormat(taskList);
         information.putStringArrayList("tmp", taskListInStringToCancel);

         intent = new Intent().setClass(this, TaskList.class);
         intent.replaceExtras(information);

         // Initialize a TabSpec for each tab and add it to the TabHost
         spec = tabHost.newTabSpec("info").setIndicator("Booking Information",
                           res.getDrawable(R.drawable.ic_tab_info))
                       .setContent(intent);
         tabHost.addTab(spec);

         // Do the same for the other tabs
         intent = new Intent().setClass(this, MapLocations.class);
         spec = tabHost.newTabSpec("mapview").setIndicator("Map View",
                           res.getDrawable(R.drawable.ic_tab_mapview))
                       .setContent(intent);
         tabHost.addTab(spec);

         intent = new Intent().setClass(this, LiveCamera.class);
         spec = tabHost.newTabSpec("camera").setIndicator("Car Camera",
                           res.getDrawable(R.drawable.ic_tab_camera))
                       .setContent(intent);
         tabHost.addTab(spec);


         tabHost.setCurrentTab(0);
    }

    private void initializeUserIp(){
		LOCAL_HOSTNAME = getLocalIpAddress();
	}

	private String getLocalIpAddress() {
		try {
            for (Enumeration<NetworkInterface> en = NetworkInterface.getNetworkInterfaces(); en.hasMoreElements();) {
                NetworkInterface intf = en.nextElement();
                for (Enumeration<InetAddress> enumIpAddr = intf.getInetAddresses(); enumIpAddr.hasMoreElements();) {
                    InetAddress inetAddress = enumIpAddr.nextElement();
                    if (!inetAddress.isLoopbackAddress()) { return inetAddress.getHostAddress().toString(); }
                }
            }
        } catch (SocketException ex) {

        }
        return null;
	}

	public class NetworkListener implements Runnable {

        public void run() {
				try {
                	int[] ports = {4440, 4441, 1024, 8080};
                	PrintWriter out = null;
                    BufferedReader in = null;

                    serverSocket = new ServerSocket(LOCAL_SERVERPORT);
                    Socket socket = null;

                    for( int p: ports ) {
        				socket = new Socket(REMOTE_HOSTNAME,p);
        				out = new PrintWriter(socket.getOutputStream(), true);
        			    in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        			    if (socket!=null) {
        			    	isInitNetwork = true;
        			    	break;
        	            }
                    }

//                    isNetworkConnectted = true;

                	while(true){
                		if (isInitNetwork) {
                			String data = ";" + userID + ":0:0:0";
                    		out.println(data);

                    		isInitNetwork = false;
                		} else if (isSetTask) {
                			String setTaskData = ";" + taskList.get(taskList.size()-1).ID + ":"
							+ taskList.get(taskList.size()-1).taskID + ":"
							+ taskList.get(taskList.size()-1).pickupOption + ":"
							+ taskList.get(taskList.size()-1).dropoffOption;

                			out.println(setTaskData);

                			isSetTask = false;
                		} else if (isCancelTask) {
                			String cancelTaskData = ";" + Integer.toString(userID) + ":"
							+ cancelTaskID + ":"
							+ "-1:-1";

                			out.println(cancelTaskData);
                			isCancelTask = false;
                		} else {
	                		try {
	                            String msg = null;

	                            //;task_id:wait:carID
	                            while (in.ready()&&(msg = in.readLine()) != null) {
	                                if (msg.startsWith(";")) {
	                                	int f = msg.indexOf(":");
	                                	int s = msg.indexOf(":",f+1);
	                                	String temp_taskID = msg.substring(1,f);

	                                	// check taskID
	                                	int sidx = -1;
	                                	for (int i=0;i<taskList.size();i++)
	                                	{
	                                		if(taskList.get(i).taskID.compareTo(temp_taskID)==0){
	                                			sidx = i;
	                                			break;
	                                		}
	                                	}

	                                	if(sidx==-1){
	                                		System.out.println("No TaskID in TaskList: "+temp_taskID);
	                                	} else {
	                                		if (msg.substring(f+1, s).compareTo(PICKUP_CANCEL_CONFIRM)==0 &&
												msg.substring(s+1).compareTo(TASK_CANCEL_CONFIRM)==0)
	                                		{
	                                			taskList.get(sidx).pickupOption = PICKUP_CANCEL_CONFIRM;
	                							taskList.get(sidx).dropoffOption = DROPOFF_CANCEL_CONFIRM;
	                                		} else if (msg.substring(f+1, s).compareTo(PICKUP_CANCEL_CONFIRM)==0 &&
													   msg.substring(s+1).compareTo(TASK_CANCEL_INVALID)==0)
	                                		{

	                                		} else {
	                                			taskList.get(sidx).waitTime = msg.substring(f+1, s);
	                                			taskList.get(sidx).carID = msg.substring(s+1);
	                                		}
	                                	}
	                                }
	                            }
	                        } catch (Exception e) {
	                        	e.printStackTrace();
	                        }
                		}

                		try {
                            Thread.sleep(500);
                        } catch( InterruptedException e ) {
                        	e.printStackTrace();
                        }
                	}
				} catch (Exception e) {
					displayServerError();
				}
        }

        private void displayServerError(){
        	MainActivity.this.runOnUiThread(new Runnable() {
				public void run() {
					String errMsg = "Attention\n"
								+ "The app cannot connect to Server. Please exit the app, and try again.";
					for (int i=0;i<1;i++)
						Toast.makeText(MainActivity.this, errMsg, Toast.LENGTH_LONG).show();
				}
			});
        }
	}

	private ArrayList<String> convertTaskListToStringFormat(List<Task> taskList) {
		ArrayList<String> taskListInStringFormat = new ArrayList<String>();

		for (int i=0;i<taskList.size();i++)
		{
			taskListInStringFormat.add(taskList.get(i).ID+":"+taskList.get(i).taskID+":"+taskList.get(i).pickupOption+":"+taskList.get(i).dropoffOption
									   +":"+taskList.get(i).waitTime+":"+taskList.get(i).carID+":"+taskList.get(i).pickupLocation+":"
									   +taskList.get(i).dropoffLocation);
		}

		return taskListInStringFormat;
	}

}