package fmui;

import java.net.*;
import java.io.*;
import java.lang.Runnable;

public class Server implements Runnable {
	/** A server instance.
	 Listens for incoming connections on a given port. Once a client connects,
	 update the central database.
	 */
    private static class theLock extends Object{ }
    static public theLock lockObject = new theLock();
    
    private ServerSocket serverSocket = null;
    private CentralDataBase db = null;
    private int port = -1;
    private int clientType = 0;
    
    private final int USER = 1;
    private final int SCHEDULER = 2;
    private final int CAR = 3;
    private final int WEBPAGE = 4;
    private final int ERR = 5;
    
    
    private int dataFromServerToSchedulerID = -1;
    private int dataFromServerToUserID = -1;
    private boolean isDataFromSchedulerToServer = false;
    private boolean isDataFromCarToWeb = false;
    private boolean isDataFromCarToUser = false;
    
    
    public Server(int port, CentralDataBase db) throws IOException {
        this.port = port;
        serverSocket = new ServerSocket(port);
        this.db = db;
    }
	
    public void run() {
		while (true) {
            ClientData c = null;
            try {
                c = new ClientData(serverSocket.accept());
            } catch (IOException e) {
                System.err.println("IOException occured: " + e.getMessage());
                System.exit(1);
            }
            
			try{
				System.out.println("Got a new connection from " + c.getSocket().getInetAddress().getHostAddress() + " on port " + port);
				clientType = c.initCleint();
				
				switch(clientType) {
				case USER:
					System.out.println("Connection establish: User");
					UserData userData = new UserData(c.getSocket());
					new UserDataHandler(userData).start();
					this.updateDatabase(userData);
					break;
				case SCHEDULER:
					System.out.println("Connection establish: Scheduler");
					SchedulerData schedulerData = new SchedulerData(c.getSocket());
					new SchedulerDataHandler(schedulerData).start();
					break;
				case CAR:
					System.out.println("Connection establish: Car");
                    CarData carData = new CarData(c.getSocket());
					new CarDataHandler(carData).start();
                    break;
				case WEBPAGE:
					System.out.println("Connection establish: Webpage");
                    WebpageData webpageData = new WebpageData(c.getSocket());
					new WebpageDataHandler(webpageData).start();
                    break;
				case ERR:
                	System.out.println("Client ERR");
                default:
                	break;
				}
			} catch (IOException e) {
                System.err.println("IOException occured: " + e.getMessage());
                continue;
            }	
		}
	}
    
    private void updateDatabase(UserData userData){

		// Search the CentralDataBase list. if the client is already there, do nothing. 
		// if not, add the new one that is to add the ClientData into CentralDataBase
		
    	synchronized(db.clients) {
			if (userData instanceof UserData) {
			
				int cidx = -1;
				for( int i=0; i<db.clients.size(); i++ ) {
					if( db.clients.get(i).getID() == userData.getID() ) {
						cidx = i;
						break;
					}
				}
				if( cidx==-1 ) {
					// add new client
					System.out.println("This is a new client");
					db.clients.add(userData);
				}
				else {
					// updata the existing client
					System.out.println("This is an existing client");
					try {
						db.clients.get(cidx).reconnect(userData);
					} catch (IOException e) {
						System.err.println("IOException occured: " + e.getMessage());
						System.exit(1); //what to do here?
					}
				}	
				System.out.println("Number of clients: " + db.clients.size() );
			}
		}
    }
    
    
    private class UserDataHandler extends Thread{
    	private UserData userData = null;
    	
    	public UserDataHandler(UserData userData) throws IOException {
    		this.userData = userData;
    	}
    	
    	public void run() {
    		while(true){
    			try {
    				synchronized(lockObject){
    					//test
    					//System.out.println("in userhandler");
    					if (userData.readUserData()) {
    						System.out.format("User->Server: ;%d:%d:%d:%d\n", userData.getID(), userData.getTaskID(),
									userData.getPickupOption(), userData.getDropoffOption());
		    				if (userData.isTaskRequestValid()) {
		    					dataFromServerToSchedulerID = userData.getID();
		    					db.updatePickupAndDropoff(dataFromServerToSchedulerID, userData.getDataToServer());
		    				}
    					} 
    					if (dataFromServerToUserID != -1 && dataFromServerToUserID == userData.getID()) {
    						int cidx = -1;
							
	    					for( int i=0; i<db.clients.size(); i++ ) {
	    						if( db.clients.get(i).getID() ==  dataFromServerToUserID) {
	    							cidx = i;
	    							break;
	    						}
	    					}
	    					if (cidx != -1) {
	    						String dataSentToUser;
	    						UserData userData = db.clients.get(cidx);
	    						
	    						dataSentToUser = ";" + userData.getTaskID() + ":" + userData.getWaitTime() + ":" 
	    							+ userData.getCarID();
	    						System.out.println("Scheduler->Server->User: " + dataSentToUser);
	    						userData.sendDataToUser(dataSentToUser);	
	    					} else
	    						System.out.println("UserData not exist in Database!");
	    					dataFromServerToUserID = -1;
    					} 
    					
    					try {
							lockObject.wait();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
    					
    					if (isDataFromCarToUser) {
    						System.out.println("Car->Server->User: "+db.getDataFromCarToUser());
    						userData.sendDataToUser(db.getDataFromCarToUser());
    						isDataFromCarToUser = false;
    					}
    				}
    			} catch (IOException e) {
					System.err.println("IOException occured: " + e.getMessage());
					continue;
				}
    		}
    	}
    }
    
    
    private class SchedulerDataHandler extends Thread{
    	private SchedulerData schedulerData = null;
    	
    	public SchedulerDataHandler(SchedulerData schedulerData) throws IOException {
    		this.schedulerData = schedulerData;
    	}
    	
    	public void run() {
    		while(true){
				try {
	    			synchronized(lockObject){
						if (dataFromServerToSchedulerID != -1) {
							int cidx = -1;
							
	    					for( int i=0; i<db.clients.size(); i++ ) {
	    						if( db.clients.get(i).getID() ==  dataFromServerToSchedulerID) {
	    							cidx = i;
	    							break;
	    						}
	    					}
	    					if (cidx != -1) {
	    						String dataSentFromServerToScheduler;
	    						UserData userData = db.clients.get(cidx);
	    						
	    						dataSentFromServerToScheduler = ";" + userData.getID() + ":" + userData.getTaskID() + ":"
	    								+ userData.getPickupOption() + ":" + userData.getDropoffOption();
	    						System.out.println("Server->Scheduler: " + dataSentFromServerToScheduler);
	    						schedulerData.sendDataToScheduler(dataSentFromServerToScheduler);	
	    					} else
	    						System.out.println("UserData not exist in Database!");
	    					
	    					dataFromServerToSchedulerID = -1;
	    					isDataFromSchedulerToServer = true;
	    				} 
						if (isDataFromSchedulerToServer) {
	    					if (schedulerData.readSchedulerData()) {
	    						System.out.format("Scheduler->Server: ;%d:%d:%d:%d\n", schedulerData.getUserID(), schedulerData.getTaskID(),
	    								schedulerData.getWaitTime(), schedulerData.getCarID());
	    						int dataFromSchedulerToServerID = schedulerData.getUserID();
	    						
	    						db.updateWaitTimeAndCarID(dataFromSchedulerToServerID, schedulerData.getDataToServer());
	    						dataFromServerToUserID = dataFromSchedulerToServerID;
	    					}
						}
						lockObject.notify();
					}
				} catch (IOException e) {
					System.err.println("IOException occured: " + e.getMessage());
					continue;
				}
    		}
    	}
    }
    
    private class CarDataHandler extends Thread{
    	private CarData carData = null;
    	
    	public CarDataHandler(CarData carData) throws IOException {
    		this.carData= carData;
    	}
    	
    	public void run() {
    		while(true){
				try {
	    			synchronized(lockObject){						
	    					if (carData.readSchedulerData()) {
	    						System.out.format("Car->Server: ;%d:%d:%f:%f\n", carData.getCarID(), carData.getTaskID(),
	    								carData.getLatitude(), carData.getLongitude());
	    						String dataFromCarToWeb = carData.getLatitude() + " " + carData.getLongitude(); 
	    						String dataFromCarToUser = "/"+ carData.getLatitude() + ":" + carData.getLongitude() ; 
	    						
	    						db.setDataFromCarToWeb(dataFromCarToWeb);
	    						db.setDataFromCarToUser(dataFromCarToUser);
	    						isDataFromCarToWeb = true;
	    						isDataFromCarToUser = true;
	    					}
	    					
	    					/*
	    					try {
	    						Thread.sleep(500);
	    					} catch (Exception e) {
	    						e.getStackTrace();
	    					}
	    					*/
					}
				} catch (IOException e) {
					System.err.println("IOException occured: " + e.getMessage());
					continue;
				}
    		}
    	}
    }
    
    private class WebpageDataHandler extends Thread{
    	private WebpageData webpageData = null;
    	
    	public WebpageDataHandler(WebpageData webpageData) throws IOException {
    		this.webpageData = webpageData;
    	}
    	
    	public void run() {
    		while(true) {
	    		try {
	    			synchronized(lockObject){						
	    					if (isDataFromCarToWeb) {
	    						System.out.println("Car->Web" +db.getDataFromCarToWeb());
	    						webpageData.sendDataToWebpage(db.getDataFromCarToWeb());
	    						isDataFromCarToWeb = false;
	    					}
					}
				} catch (Exception e) {
					System.err.println("Exception occured: " + e.getMessage());
					continue;
				}
    		}
    	}
    }
    
}
