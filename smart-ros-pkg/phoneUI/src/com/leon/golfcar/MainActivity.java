package com.leon.golfcar;

import java.util.Enumeration;
import java.util.List;
import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.Overlay;
import com.leon.golfcar.R;
import android.content.Context;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.Toast;

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

public class MainActivity extends MapActivity  {
    /** Called when the activity is first created. */
    private MapView mapView = null;
    private MapController mapController = null;
	private GeoPoint dropOffGeoPoint = null;
	private GeoPoint pickUpGeoPoint;
    private double dropOffLatitude = 0.0;
    private double dropOffLongitude = 0.0;
    private double pickUpLatitude = 0.0;
    private double pickUpLongitude = 0.0;
    private boolean orderFlag = false;
    private int userID;
    private int taskID = 1;
    
    private int cancelTaskID = 0;
    private boolean cancelFlag = false;
    
    final private int REQUEST_CODE = 1;
    
    private static final String REMOTE_HOSTNAME = "172.17.185.120";
    // Phone's IP and Port for sending and listening
    private static String LOCAL_HOSTNAME = "172.17.184.92"; //default
    private static final int LOCAL_SERVERPORT = 4440;
    private static final String CANCEL_CONFIRM = "-6";
    private static final String CANCEL_INVALID = "-5";
    private static final String CANCEL_PICKUP = "-1";
    private static final String CANCEL_DEST = "-1";
    private static final int SET_TASK = 2;
    private static final int CANCEL_TASK = 1;

    private List<Task> taskList = new ArrayList<Task>();

	@Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
		setTitle("Google Map");
		
		initializeApp();
    }
	
	private void initializeApp(){
		initializeGoogleMap();
		initializeUserIp();
		initializeUserID();
		initializeUserID();
		runNetworkListener();
	}
	
	private void initializeGoogleMap(){
		mapView = (MapView) findViewById(R.id.mapView);
        mapView.setSatellite(true);
        mapView.setEnabled(true);
        mapView.setClickable(true);
        mapView.setBuiltInZoomControls(true);
        mapView.displayZoomControls(true);
        mapController = mapView.getController();
	}
	
	private void initializeUserIp()
	{
		LOCAL_HOSTNAME = getLocalIpAddress();
	}
	
	private void initializeUserID()
	{
		// serial number of the phone
		String serialNumber = null; 

		try {
		    Class<?> c = Class.forName("android.os.SystemProperties");
		    Method get = c.getMethod("get", String.class);
		    serialNumber = (String) get.invoke(c, "ro.serialno");
		} catch (Exception ignored) {
		}
		
		userID = generateUserID(serialNumber);
	}
	
	private void runNetworkListener()
	{
		Thread fst = new Thread(new NetworkListener());
       // fst.setName("Thread:onCreate");
        fst.start();
	}
	
	// Generate ID
	private int generateUserID(String serialNumber){
		return serialNumber.hashCode() > 0 ? serialNumber.hashCode() : -serialNumber.hashCode();
	}
	

	// check connection to Server
	private void checkConnectionWithServer(){
		 //if (!client.connection) {
	       if (true){
			Context context = getApplicationContext();
	        	CharSequence text = "Cannot connection to Server. Please check Internet settings";
	        	int duration = Toast.LENGTH_LONG;
	        	
	        	Toast toast = Toast.makeText(context, text, duration);
	        	toast.show();
	        }
	}

	
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		MenuInflater inflater = getMenuInflater();
		inflater.inflate(R.menu.service, menu);
		return true;
	}
	
	@Override
	public boolean onPrepareOptionsMenu(Menu menu) {
		MenuItem setting = menu.findItem(R.id.check_setting);
		CharSequence on = "Connection: ON";
		CharSequence off = "Connection: OFF";
		
		//if(client.connection)
			setting.setTitle(on);
		//else
		//	setting.setTitle(off);
		
		return super.onPrepareOptionsMenu(menu);
	}
	
	//define Option Menu, including set_pickup, set_destination, get_status, cancel task
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()){
		case R.id.book_car:
			setTask();
			return true;
		case R.id.service_info:
			checkTasklist();
			return true;
		case R.id.cancel_service:
			cancelTask();
			return true;	
		default:
			return super.onOptionsItemSelected(item);
		}
	}
	
	private void setTask(){
		Intent intentSetTaskActivity = new Intent(MainActivity.this,TaskSettingActivity.class);
		startActivityForResult(intentSetTaskActivity, REQUEST_CODE);
	}
	
	private void checkTasklist(){
		Intent intentTasklistInfoActivity = new Intent(MainActivity.this,TaskListInfoActivity.class);		
		ArrayList<String> tasklistInfoString = new ArrayList<String>();

		for (int i=0;i<taskList.size();i++)
		{
			tasklistInfoString.add(taskList.get(i).convertTaskInfoToString());
		}
		
		intentTasklistInfoActivity.putStringArrayListExtra("tasklistInfoString", tasklistInfoString);
		startActivity(intentTasklistInfoActivity);
	}

	private void cancelTask() {
		Intent intentCancelTask = new Intent(MainActivity.this, TaskCancelActivity.class);
		
		ArrayList<String> tasklistInfoString = new ArrayList<String>();
		// store tasks info into tmp
		for (int i=0;i<taskList.size();i++)
		{
			tasklistInfoString.add(taskList.get(i).convertTaskInfoToString());
		}
		
		intentCancelTask.putStringArrayListExtra("tmp", tasklistInfoString);			
		startActivityForResult(intentCancelTask, REQUEST_CODE);
	}
	
	
	//handle return value from RadioGroupActivity and RadioGroupActivity
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {		
		if (requestCode == REQUEST_CODE && resultCode == RESULT_OK) {
			setTaskAndCancelTask(data);
		}
	}
	
	private void setTaskAndCancelTask(Intent data){
		Bundle extras = data.getExtras();
		if (extras != null) {
			receiveDataFromSetTaskActivityAndCancelTaskActivity(extras);
		}
	}
	
	private void receiveDataFromSetTaskActivityAndCancelTaskActivity(Bundle extras){
		int actionType = 0;
		
		actionType = extras.getInt("actionType");
		
		switch(actionType) {
			case CANCEL_TASK:
				cancelTask(extras);
				break;
			case SET_TASK:
				setTask(extras);
				break;
			default:
				break;
		}
	}
	
	private void cancelTask(Bundle extras) {
		cancelTaskID = extras.getInt("cancelTaskID");
		
		if(cancelTaskID!=0) {
			cancelFlag = true;
		}
	}
	
	private void setTask(Bundle extras) {
			if (extras.getInt("pickup_op")!=extras.getInt("dest_op"))
			{
				drawTaskOnMapAndAddTaskToTasklist(extras);
				orderFlag = true;
			} else {
				displaySetTaskErr();
			}
	}
	
	private void drawTaskOnMapAndAddTaskToTasklist(Bundle extras) {
		drawTaskDetailOnMap(extras);
		addTaskToTasklist(extras);
	}
	
	private void drawTaskDetailOnMap(Bundle extras) {
		getLocationInformation(extras);
		drawDropOffLocation();
		drawPickUpLocation();
	}
	
	private void addTaskToTasklist(Bundle extras) {
		Task s = new Task(Integer.toString(userID),Integer.toString(taskID++),
						Integer.toString(extras.getInt("pickup_op")),Integer.toString(extras.getInt("dest_op")),
						"--","--",extras.getString("pickup_location"),extras.getString("dest_location"));
		taskList.add(s);
	}
	
	private void displaySetTaskErr() {
		Context context = getApplicationContext();
    	CharSequence text = "Service cannot be deliveried. Pick-up location and Drop-off location cannot be the same.";
    	int duration = Toast.LENGTH_LONG;
    	
    	Toast toast = Toast.makeText(context, text, duration);
    	toast.show();
	}
	
	private void getLocationInformation(Bundle extras){
		//ServiceMsg 
		//get data from BookCarActivity.java
		pickUpLatitude = extras.getDouble("pickup_lat");
		pickUpLongitude = extras.getDouble("pickup_longi");
		dropOffLatitude = extras.getDouble("dest_lat");
		dropOffLongitude = extras.getDouble("dest_longi");	
	}
	
	private void drawDropOffLocation(){
		// draw destination pin on Map
		dropOffGeoPoint = new GeoPoint ((int)(dropOffLatitude * 1000000), (int)(dropOffLongitude * 1000000));        
		mapController.animateTo(dropOffGeoPoint);
		mapController.setZoom(17);	
		
		DropOffLocationOverlay dropOffOverlay = new DropOffLocationOverlay();
		List<Overlay> dest_list = mapView.getOverlays();
		dest_list.add(dropOffOverlay);
	}
	
	private void drawPickUpLocation(){
		// draw pickup location pin on Map
		pickUpGeoPoint = new GeoPoint ((int)(pickUpLatitude * 1000000), (int)(pickUpLongitude * 1000000));        
		mapController.animateTo(pickUpGeoPoint);
		mapController.setZoom(17);	
		
		PickUpLocationOverlay pickUpOverlay = new PickUpLocationOverlay();
		List<Overlay> pickup_list = mapView.getOverlays();
		pickup_list.add(pickUpOverlay);
	}
	
	//draw pickup location overlay
	public class PickUpLocationOverlay extends Overlay
	{
		public boolean draw(Canvas canvas, MapView mapView, boolean shadow, long when)
		{
			super.draw(canvas, mapView, shadow);
			Paint paint = new Paint();
			Point myScreenCoords = new Point();
			Bitmap bmp;
			
			//convert
			mapView.getProjection().toPixels(pickUpGeoPoint, myScreenCoords);
			paint.setStrokeWidth(1);
			paint.setARGB(255, 255, 0, 0);
			paint.setStyle(Paint.Style.STROKE);
			bmp = BitmapFactory.decodeResource(getResources(),R.drawable.car);
			canvas.drawBitmap(bmp, myScreenCoords.x, myScreenCoords.y,paint);
			
			return true;	
		}
		
		public void disableCompass() {
			// TODO Auto-generated method stub
		}
		
		public void enableCompass() {
			// TODO Auto-generated method stub
		}
	}
	
	//draw destination location overlay
	public class DropOffLocationOverlay extends Overlay
	{
		public boolean draw(Canvas canvas, MapView mapView, boolean shadow, long when)
		{
			super.draw(canvas, mapView, shadow);
			Paint paint = new Paint();
			Point myScreenCoords = new Point();
			Bitmap bmp;
			//convert
			
			mapView.getProjection().toPixels(dropOffGeoPoint, myScreenCoords);
			paint.setStrokeWidth(1);
			paint.setARGB(255, 255, 0, 0);
			paint.setStyle(Paint.Style.STROKE);
			
			bmp = BitmapFactory.decodeResource(getResources(),R.drawable.cabin);			
			canvas.drawBitmap(bmp, myScreenCoords.x, myScreenCoords.y,paint);
			
			return true;
		}
		
		public void disableCompass() {
			// TODO Auto-generated method stub
		}
		
		public void enableCompass() {
			// TODO Auto-generated method stub
		}
	}
	
    public class NetworkListener implements Runnable {
    	private final int[] ports = {4440, 4441, 1024, 8080};
    	private PrintWriter out = null;
        private BufferedReader in = null;
        private ServerSocket serverSocket = null;
    	private String feedbackTaskID = "0";
    	private String feedbackTaskWaitTime = "0";
    	private String feedbackTaskCarID = "0";
    	
    	public void run() {
            if (LOCAL_HOSTNAME != null) {
            	runNetworkListener();
            } else {	//ServiceIP = null
            }
        }
    
        private void runNetworkListener(){
	    	try{
	    		initializeNetWorkListener();
	            transferData();
	    	} catch (Exception e) {
	    		e.printStackTrace();
	    	}
	    }
    
    	private void initializeNetWorkListener() throws IOException {
    		serverSocket = new ServerSocket(LOCAL_SERVERPORT);
            Socket socket = null;
            
            for( int p: ports ) {
				socket = new Socket(REMOTE_HOSTNAME,p);
				out = new PrintWriter(socket.getOutputStream(), true);
			    in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
			    if (socket!=null) {
	                break;
	            }
            }
    	}
    
    	private void transferData() throws IOException {
        	while(true){
        		if (orderFlag) {
        			sendSetTaskData();
        		}
        		if (cancelFlag) {
        			sendCancelTaskData();
        		}
        		
        		getFeedbackFromScheduler();     
	        	
	        	try {
	        		Thread.sleep(500);
	        	} catch( InterruptedException e ) {
	        		e.printStackTrace();
	        	}
	        }
    	}
    	
    	private void sendSetTaskData(){
    		String data = ";" + taskList.get(taskList.size()-1).ID + ":" 
    				+ taskList.get(taskList.size()-1).taskID + ":" 
    				+ taskList.get(taskList.size()-1).pickUpOption + ":" 
    				+ taskList.get(taskList.size()-1).dropOffOption;
    		
    		out.println(data);
    		
    		orderFlag = false;
    	}
    
    	private void sendCancelTaskData(){
    		String data =";" + Integer.toString(userID) + ":"
					+ cancelTaskID + ":" + "-1:-1"; 
    		
    		out.println(data);
			
    		cancelFlag = false;
    	}
    	
    	private void getFeedbackFromScheduler() throws IOException {
            String msg = null;
            
            //;task_id:wait:carID
            while (in.ready()&&(msg = in.readLine()) != null) {                                
                updateTasklist(msg);
            }
    	}
    	
    	private void updateTasklist(String msg){
    		if (msg.startsWith(";")) {
    			readFeedback(msg);
            	
            	int taskIDInTasklist = -1;
            	taskIDInTasklist = checkTaskValid();
            	
            	if(taskIDInTasklist==-1){
            		displayTaskInvalid(); //no such Task in Tasklist
            	} 
            	else{
            		updateTaskInTasklist(taskIDInTasklist);
            	}         
            }
    	}
    	
    	private void readFeedback(String msg){
    		int f = msg.indexOf(":");
        	int s = msg.indexOf(":",f+1);
        	
    		feedbackTaskID = msg.substring(1,f);
    		feedbackTaskWaitTime = msg.substring(f+1, s);
    		feedbackTaskCarID = msg.substring(s+1);
    	}
    	
    	private int checkTaskValid(){
    		int sidx = -1;
    		for (int i=0;i<taskList.size();i++)
        	{
        		if(taskList.get(i).taskID.compareTo(feedbackTaskID)==0){
        			sidx = i;
        			break;
        		}		
        	}
    		
    		return sidx;
    	}
    	
    	private void displayTaskInvalid(){
    		System.out.println("No TaskID: " + feedbackTaskID);
    	}
    	
    	private void updateTaskInTasklist(int taskIDInTasklist){
    		if (feedbackTaskWaitTime.compareTo(CANCEL_PICKUP)==0 && feedbackTaskCarID.compareTo(CANCEL_CONFIRM)==0)
    		{
    			taskList.get(taskIDInTasklist).pickUpOption = CANCEL_PICKUP;
				taskList.get(taskIDInTasklist).dropOffOption = CANCEL_DEST;
    		} else if (feedbackTaskWaitTime.compareTo(CANCEL_PICKUP)==0 
    			&& feedbackTaskCarID.compareTo(CANCEL_INVALID)==0)
    		{
    			
    		} else {
    			taskList.get(taskIDInTasklist).waitTime = feedbackTaskWaitTime;
    			taskList.get(taskIDInTasklist).carID = feedbackTaskCarID;
    		}
    	}
   }
	
    // gets the ip address of your phone's network
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
            Log.e("ServerActivity", ex.toString());
        }
        return null;
    }
	
    /*
    @Override
    protected void onStop() {
        super.onStop();
        try {
			// make sure you close the socket upon exiting
			serverSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
    }*/

	@Override
	protected boolean isRouteDisplayed() {
		// TODO Auto-generated method stub
		return false;
	}
}


