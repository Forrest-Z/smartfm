package com.leon.golfcar;

import java.util.Enumeration;
import java.util.List;
import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.Overlay;
import com.leon.golfcar.R;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Point;
import android.os.Bundle;
import android.view.KeyEvent;
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
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.logging.*;

public class MainActivity extends MapActivity  {
    /** Called when the activity is first created. */
    private static final String REMOTE_HOSTNAME = "172.17.38.204";
    private static final String TASK_CANCEL_CONFIRM = "-6";
    private static final String TASK_CANCEL_INVALID = "-5";
    private static final String PICKUP_CANCEL_CONFIRM = "-1";
    private static final String DROPOFF_CANCEL_CONFIRM = "-1";
    private static final int LOCAL_SERVERPORT = 4440;
    private static String LOCAL_HOSTNAME = "172.17.184.92"; 
    public static Logger logger;
    private final int REQUEST_CODE = 1;
    private double dropoffLatitude = 0.0;
    private double dropoffLongitude = 0.0;
    private double pickupLatitude = 0.0;
    private double pickupLongitude = 0.0;
    private boolean isSetTask = false;
    private boolean isInitNetwork = false;
    private boolean isCancelTask = false;
    private boolean isNetworkConnectted = false;
    private int userID;
    private int taskID = 1;
    private int cancelTaskID = 0;    
	private MapView mapView = null;
    private MapController mapController = null;
    private GeoPoint initGeoPoint = null;
	private GeoPoint dropoffGeoPoint = null;
	private GeoPoint pickupGeoPoint = null;
	private GeoPoint carGeoPoint = null;
    private ServerSocket serverSocket = null;
    private List<Task> taskList = new ArrayList<Task>();
    private List<Overlay> overlayList = null;
 //   private List<Overlay> carOverlay = null;
	
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
		setTitle("Google Map");
		
		initializeGoogleMap();
        initializeUserIp();
        initializeUserID();
        runNetworkListener();
    }
	
	private void initializeGoogleMap(){
		initGeoPoint = new GeoPoint ((int)(1.299292 * 1000000), (int)(103.770596 * 1000000));
		
		mapView = (MapView) findViewById(R.id.mapView);
        mapView.setSatellite(true);
        mapView.setEnabled(true);
        mapView.setClickable(true);
        mapView.setBuiltInZoomControls(true);
        mapView.displayZoomControls(true);
        mapController = mapView.getController();
        mapController.animateTo(initGeoPoint);
		mapController.setZoom(19);
        overlayList = mapView.getOverlays();
        //carOverlay = mapView.getOverlays();
	}
	
	private void initializeUserIp()
	{
		LOCAL_HOSTNAME = getLocalIpAddress();
	}
	
	private void initializeUserID()
	{
		String serialNumberOfPhone = null; 
		
		try {
		    Class<?> c = Class.forName("android.os.SystemProperties");
		    Method get = c.getMethod("get", String.class);
		    serialNumberOfPhone = (String) get.invoke(c, "ro.serialno");
		} catch (Exception ignored) {
		}
		
		userID = generateUserID(serialNumberOfPhone);
	}
	
	private int generateUserID(String serialNumberOfPhone){
		return serialNumberOfPhone.hashCode() > 0 ? serialNumberOfPhone.hashCode() : -serialNumberOfPhone.hashCode();
	}
	
	private void runNetworkListener()
	{
		if (LOCAL_HOSTNAME!= null) {
			Thread fst = new Thread(new NetworkListener());
			fst.start();
		} else
			displayNetworkError();
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
		
		if(isNetworkConnectted)
			setting.setTitle(on);
		else
			setting.setTitle(off);
		
		return super.onPrepareOptionsMenu(menu);
	}
	
	//define Option Menu, including set_pickup, set_destination, get_status, cancel task
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()){
			case R.id.book_car:
				Intent intent_bookCar = new Intent(MainActivity.this,TaskSettingActivity.class);
				startActivityForResult(intent_bookCar, REQUEST_CODE);
				return true;
			case R.id.service_info:
				Intent intent_serviceInfo = new Intent(MainActivity.this,TaskListInfoActivity.class);		
				ArrayList<String> taskListInString = null;
				
				taskListInString = convertTaskListToStringFormat(taskList);
				intent_serviceInfo.putStringArrayListExtra("tmp", taskListInString);
				startActivity(intent_serviceInfo);
				return true;
			case R.id.cancel_service:
				
				Intent intent_cancelOrder = new Intent(MainActivity.this, TaskCancelActivity.class);
				ArrayList<String> taskListInStringToCancel = null;
				
				taskListInStringToCancel = convertTaskListToStringFormat(taskList);
				intent_cancelOrder.putStringArrayListExtra("tmp", taskListInStringToCancel);
				startActivityForResult(intent_cancelOrder, REQUEST_CODE);
				return true;	
			default:
				return super.onOptionsItemSelected(item);
		}
	}
	
	@Override
	public boolean onKeyDown(int keyCode, KeyEvent event) {
	    //Handle the back button
	    if(keyCode == KeyEvent.KEYCODE_BACK) {
	        //Ask the user if they want to quit
	        new AlertDialog.Builder(this)
	        .setIcon(android.R.drawable.ic_dialog_alert)
	        .setTitle(R.string.quit)
	        .setMessage(R.string.really_quit)
	        .setPositiveButton(R.string.yes, new DialogInterface.OnClickListener() {
				
	            public void onClick(DialogInterface dialog, int which) {
					
	            	onStop();
	                //Stop the activity
	                MainActivity.this.finish();    
	            }
				
	        })
	        .setNegativeButton(R.string.no, null)
	        .show();
			
	        return true;
	    }
	    else {
	        return super.onKeyDown(keyCode, event);
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
	
	//handle return value from RadioGroupActivity and RadioGroupActivity
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {		
		if (requestCode == REQUEST_CODE) {
			if (resultCode == RESULT_OK){
	    		Bundle extras = data.getExtras();
				if (extras != null)
				{
					if (extras.getInt("actionType")==1)
					{
						cancelTaskID = extras.getInt("cancelTaskID");
						
						if(cancelTaskID!=0)
						{
							isCancelTask = true;
						}
					}
					else {
						//ServiceMsg 
						//get data from BookCarActivity.java
						pickupLatitude = extras.getDouble("pickup_lat");
						pickupLongitude = extras.getDouble("pickup_longi");
						dropoffLatitude = extras.getDouble("dest_lat");
						dropoffLongitude = extras.getDouble("dest_longi");	
						
						if (extras.getInt("pickup_op")!=extras.getInt("dest_op"))
						{
							/*
							// draw destination pin on Map
							dropoffGeoPoint = new GeoPoint ((int)(dropoffLatitude * 1000000), (int)(dropoffLongitude * 1000000));        
							mapController.animateTo(dropoffGeoPoint);
							drawDestinationLocationOverlay dest_overlay = new drawDestinationLocationOverlay();
							overlayList.add(dest_overlay);
							mapView.invalidate();
							
							// draw pickup location pin on Map
							pickupGeoPoint = new GeoPoint ((int)(pickupLatitude * 1000000), (int)(pickupLongitude * 1000000));        
							drawPickupLocationOverlay pickup_overlay = new drawPickupLocationOverlay();
							overlayList.add(pickup_overlay);
							 */
							
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
		}
	}
	
	class drawPickupLocationOverlay extends Overlay
	{	
		public boolean draw(Canvas canvas, MapView mapView, boolean shadow, long when)
		{
			super.draw(canvas, mapView, shadow);
			Paint paint = new Paint();
			Point myScreenCoords = new Point();
			Bitmap bmp;
			
			//convert
			mapView.getProjection().toPixels(pickupGeoPoint, myScreenCoords);
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
	
	class drawDestinationLocationOverlay extends Overlay
	{		
		public boolean draw(Canvas canvas, MapView mapView, boolean shadow, long when)
		{
			super.draw(canvas, mapView, shadow);
			Paint paint = new Paint();
			Point myScreenCoords = new Point();
			Bitmap bmp;
			//convert
			
			mapView.getProjection().toPixels(dropoffGeoPoint, myScreenCoords);
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
	
    @Override
    protected void onStop() {
        super.onStop();
        try {
			// make sure you close the socket upon exiting
			serverSocket.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
	
	@Override
	protected boolean isRouteDisplayed() {
		// TODO Auto-generated method stub
		return false;
	}
	
	private void displayNetworkError(){
		String errMsg = "Attention\n" 
		+ "A network error has occured. Please exit the app, and try again.";
		
		for (int i=0;i<1;i++)
			Toast.makeText(MainActivity.this, errMsg, Toast.LENGTH_LONG).show();  
    }
	
	static {
	    try {
	      boolean append = true;
	      FileHandler fh = new FileHandler("./NetworkListener.log", append);
	      //fh.setFormatter(new XMLFormatter());
	      fh.setFormatter(new SimpleFormatter());
	      logger = Logger.getLogger("TestLog");
	      logger.addHandler(fh);
	    }
	    catch (IOException e) {
	      e.printStackTrace();
	    }
	}
    class NetworkListener implements Runnable {
    	private final int[] ports = {4440, 4441, 1024, 8080};
    	//private String forcheck = null;
    	//private String forcheck2 = null;
		private PrintWriter out = null;
		private BufferedReader in = null;
		private Socket socket = null;

		public void run() {
			try {
				serverSocket = new ServerSocket(LOCAL_SERVERPORT);

				initNetwork();				
				if (socket != null) {
                    isNetworkConnectted = true;                    
                	while(true){
                		dataTransfer();
                		sleep();
                	}
				} else {
					close();
				}
			} catch (UnknownHostException e) {
				displayServerError();
			} catch (IOException e) {
				displayServerError();
			}
        }
        
        private void initNetwork() throws IOException{
        	for( int p: ports ) {
				socket = new Socket(REMOTE_HOSTNAME,p);
				out = new PrintWriter(socket.getOutputStream(), true);
				in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
				if (socket!=null) {
					isInitNetwork = true;
					break;
				}
			}
        }
        
        private void dataTransfer(){
        	if (isInitNetwork) {
    			sendInitInfo();
    		} else if (isSetTask) {
    			sendSettingTaskInfo();
    		} else if (isCancelTask) {
    			sendCancellingTaskInfo();
    		} else {
    			getDataFromServer();	        
    		}
        }
        
        private void sendInitInfo(){
        	String data = ";" + userID + ":0:0:0";
    		
        	out.println(data);
    		isInitNetwork = false;
        }
        
        private void sendSettingTaskInfo(){
        	String setTaskData = ";" + taskList.get(taskList.size()-1).ID + ":" 
					+ taskList.get(taskList.size()-1).taskID + ":" 
					+ taskList.get(taskList.size()-1).pickupOption + ":" 
					+ taskList.get(taskList.size()-1).dropoffOption; 
        			
        	out.println(setTaskData);
        	isSetTask = false;
        }
        
        private void sendCancellingTaskInfo(){
        	String cancelTaskData = ";" + Integer.toString(userID) + ":"
					+ cancelTaskID + ":"
					+ "-1:-1"; 
        			
        	out.println(cancelTaskData);
        	isCancelTask = false;
        }
        
        private void getDataFromServer(){
        	try {
                String msg = null;
                //;task_id:wait:carID
                while (in.ready()&&(msg = in.readLine()) != null) {        
                    if (msg.startsWith(";")) {
                    	readDataFromScheduler(msg);
                    	
                    } else if (msg.startsWith("/")){ //data format: "/latitude:longitude"
                    	readDataFromCar(msg);
                    }
                }
            } catch (Exception e) {
            	e.printStackTrace();
            }            
        }
        
        private void readDataFromScheduler(String msg) {
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
        		if (msg.substring(f+1, s).compareTo(PICKUP_CANCEL_CONFIRM)==0 && msg.substring(s+1).compareTo(TASK_CANCEL_CONFIRM)==0)
        		{
        			taskList.get(sidx).pickupOption = PICKUP_CANCEL_CONFIRM;
					taskList.get(sidx).dropoffOption = DROPOFF_CANCEL_CONFIRM;
        		} else if (msg.substring(f+1, s).compareTo(PICKUP_CANCEL_CONFIRM)==0 && msg.substring(s+1).compareTo(TASK_CANCEL_INVALID)==0)
        		{
        		} else {
        			taskList.get(sidx).waitTime = msg.substring(f+1, s);
        			taskList.get(sidx).carID = msg.substring(s+1);
        		}
        	}         
        }
        
        private void readDataFromCar(String msg){
        	//test
        	//forcheck = msg;
        	//checkData();
        	
        	int f = msg.indexOf(":");
        	double latitude = 0.0;
        	double longitude = 0.0;
        	
        	latitude = Double.parseDouble(msg.substring(1,f));
        	longitude = Double.parseDouble(msg.substring(f+1));
        	if (latitude != 0.0 && longitude != 0.0) {
      			// draw car marker on Map  
        		overlayList.clear();
        		carGeoPoint = new GeoPoint ((int)(latitude* 1000000), (int)(longitude* 1000000));  
      			drawCarLocationOverlay car_overlay = new drawCarLocationOverlay();
      			//bug		                          			
      			try {
      			mapController.animateTo(carGeoPoint);
      			overlayList.add(car_overlay);
      			mapView.postInvalidate();
      			} catch (Exception e) {
      				logger.info(e.getMessage());
      			}
        	}
        }
        
        private void sleep(){
        	try {
                Thread.sleep(500);
            } catch( InterruptedException e ) { 
            	e.printStackTrace();
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
        
//        private void checkData(){
//        	MainActivity.this.runOnUiThread(new Runnable() {
//				public void run() {
//					String errMsg = "checkData: " + forcheck;
//					for (int i=0;i<1;i++)
//						Toast.makeText(MainActivity.this, errMsg, Toast.LENGTH_LONG).show();   
//				}
//			});;
//        }
//        
//        private void checkError(){
//        	MainActivity.this.runOnUiThread(new Runnable() {
//				public void run() {
//					String errMsg = "checkData: " + forcheck2;
//					for (int i=0;i<1;i++)
//						Toast.makeText(MainActivity.this, errMsg, Toast.LENGTH_LONG).show();   
//				}
//			});
//        }
//        
        
      //draw destination location overlay
    	public class drawCarLocationOverlay extends Overlay
    	{		
    		public boolean draw(Canvas canvas, MapView mapView, boolean shadow, long when)
    		{
    			super.draw(canvas, mapView, shadow);
    			Paint paint = new Paint();
    			Point myScreenCoords = new Point();
    			Bitmap bmp;
    			//convert
    			mapView.getProjection().toPixels(carGeoPoint, myScreenCoords);
    			paint.setStrokeWidth(1);
    			paint.setARGB(255, 255, 0, 0);
    			paint.setStyle(Paint.Style.STROKE);
    			bmp = BitmapFactory.decodeResource(getResources(),R.drawable.cab);			
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
    	
    	private void close() throws IOException{
    		in.close();
    		out.close();
    		socket.close();
    	}
    }
}


