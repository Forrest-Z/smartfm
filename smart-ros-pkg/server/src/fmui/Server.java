package fmui;

import java.net.*;
import java.io.*;
import java.util.*;
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
    private boolean isServerToScheduler = false;
    private boolean isServerToUser = false;
	private boolean isServerToWeb = false;
    private boolean dataSend = true;
    private String pickUpOption = "0";
    private String dropOffOption = "0";
    private String userID = "0";
    private String waitTime = "0";
    private String carID = "0";
    private String taskID = "0";
    private String latitude = "0";
    private String longitude = "0";

    
    private int client = -1;
    
    private final int USER = 1;
    private final int SCHEDULER = 2;
    private final int CAR = 3;
    private final int WEBPAGE = 4;
    private final int ERR = 5;
    
    private String toUser = "-1";
    
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
                // Print the error and exit
                // A better mechanism should be implemented to continue accepting connections...
                System.err.println("IOException occured: " + e.getMessage());
                System.exit(1);
            }
            
			try{	
				client = c.checkClientType();
				
				if (client==SCHEDULER) {
					System.out.println("Connection establish: Scheduler");
					new Scheduler(c).start();
				} else if (client==WEBPAGE){
                    System.out.println("Connection establish: Webpage");
                    new Webpage(c).start();
                } else if (client==CAR){
                    System.out.println("Connection establish: Car");
                    new Car(c).start();
                } else if (client==USER){
					System.out.println("Connection establish: User");
					new User(c).start();
			    } else
			    	System.out.println("Unknown Client. Please check again." );
			} catch (IOException e) {
                System.err.println("IOException occured: " + e.getMessage());
                continue;
            }
			
			System.out.println("Got a new connection from " + c.socket.getInetAddress().getHostAddress() + " on port " + port);
			
			// Search the CentralDataBase list. if the client is already there, do nothing. 
			// if not, add the new one that is to add the ClientData into CentralDataBase
			synchronized(db.clients) {
				int cidx = -1;
				for( int i=0; i<db.clients.size(); i++ ) {
					if( db.clients.get(i).ID.compareTo(c.ID)==0 ) {
						cidx = i;
						break;
					}
				}
				if( cidx==-1 ) {
					// add new client
					System.out.println("This is a new client");
					db.clients.add(c);
				}
				else {
					// updata the existing client
					System.out.println("This is an existing client");
					try {
						db.clients.get(cidx).reconnect(c);
					} catch (IOException e) {
						System.err.println("IOException occured: " + e.getMessage());
						System.exit(1); //what to do here?
					}
				}	
				System.out.println("Number of clients: " + db.clients.size() +"\n");
			}
    	}
	}
	
	// Server Thread
	private class Scheduler extends Thread {
    	private ClientData cd = null;
    	
    	public Scheduler(ClientData cd) {
    		this.cd = cd;
    	}
    	
    	public void run() {
			while(true){
				synchronized(lockObject){
					try {			
						// if User sends order to Server, Server forwards the msgs to Scheduler
						if (isServerToScheduler)
						{
							System.out.println("Server->Scheduler: "+";"+Integer.parseInt(userID)+":"+taskID+":"+pickUpOption+":"+dropOffOption);
							
							cd.out.println(";"+Integer.parseInt(userID)+":"+taskID+":"+pickUpOption+":"+dropOffOption);
							
							// clean received msgs
							isServerToScheduler = false;
							taskID = "0";
							userID = "0";
							pickUpOption = "0";
							dropOffOption = "0";
							
							try {
								Thread.sleep(500);
							} catch( InterruptedException e ) {
								e.getStackTrace();
            				}
						} else if (dataSend){ // if not order, keep reading from Scheduler
							// read msgs from Scheduler
							
							if (cd.readMsg(SCHEDULER)){
								
								// Scheduler sends feedback to none-zero User ID
								if (cd.user_id.compareTo("0")!=0)
								{
									System.out.println("Scheduler->Server: "+";"+cd.ID+":"+cd.user_id+":"
													+cd.taskID+":"+cd.wait+":"+cd.carID);
									// set transfer msgs
									//isServerToUser = true;
									
									userID = cd.user_id;
									taskID = cd.taskID;
									waitTime = cd.wait;
									carID = cd.carID;
									
									//int uidx = -1;
				
				for( int i=0; i<db.clients.size(); i++ ) {
					if( db.clients.get(i).ID.compareTo(userID)==0 ) {
						//uidx = i;
						toUser = db.clients.get(i).ID;
						break;
					}
				}
									
									dataSend = false;
									
									
									
									// clean sending msgs
									cd.taskID = "0";
									cd.user_id = "0";
									cd.wait="0";
									cd.carID="0";
								}
							}
						}
					} catch (IOException e) {
						System.err.println("IOException occured: " + e.getMessage());
						continue;
					}
				}
    		}
    	}	
    }
    
    private class User extends Thread {
    	private ClientData cd = null;
    	
    	public User(ClientData cd) {
    		this.cd = cd;
    	}
		
    	public void run() {
			while(true){
				try{
					synchronized(lockObject){
						// if Server receives from Scheduler, then forwards to User
						//if (isServerToUser)
						if (toUser.compareTo(cd.ID)==0)
						{	
							System.out.println("Server->User: "+"UserID: "+cd.ID+";"+taskID+":"+waitTime+":"+carID);
							cd.out.println(";"+taskID+":"+waitTime+":"+carID);
							taskID = "0";
							waitTime = "0";
							carID = "0";
							//isServerToUser = false;
							toUser = "-1";
							dataSend = true;
							
							try {
								Thread.sleep(500);
							} catch( InterruptedException e ) {
            				}
						} else {
							// read msg from User
							if (cd.readMsg(USER)) {
								// if User sends order, receive it
								if (cd.pickup.compareTo("0")!=0&&cd.dest.compareTo("0")!=0)
								{
									System.out.println("User->Server: "+";"+cd.ID+":"+cd.taskID+":"+cd.pickup+":"+cd.dest);
									isServerToScheduler = true;
									userID = cd.ID;
									taskID = cd.taskID;
									pickUpOption = cd.pickup;
									dropOffOption = cd.dest;
									
									cd.taskID = "0";
									cd.pickup = "0";
									cd.dest = "0";
									cd.user_id = "0";
								}	
							}
						}
					}		
				}catch (IOException e) {
					System.err.println("IOException occured: " + e.getMessage());
					continue;
				}
			}
    	} 
    }
	
    private class Car extends Thread{
        private ClientData cd = null;
		
        public Car(ClientData cd){
            this.cd = cd;
        }
		
        public void run(){
            while(true){
                try{  
                    synchronized(lockObject){
						
                        if (cd.readMsg(CAR)){
							
							if(cd.taskID.compareTo("1")==0){
								System.out.println("Car->Server: "+";"+cd.ID+":"+cd.taskID+":"+cd.latitude+":"+cd.longitude);
								latitude = cd.latitude;
								longitude = cd.longitude;
								
								//System.out.println("lat: "+latitude+" long: "+longitude);                            
								
								isServerToWeb = true;
							} 
						}
						
                    }
                } catch (IOException e) {
                	System.err.println("IOException occured: " + e.getMessage());
					continue;
                }
            }
        }
		
    }
    
    private class Webpage extends Thread {
    	
		public int count =0;
		private ClientData cd = null;
		String coordinates = 	"1.298529" +" "+ "103.770853" +"\n"+
		"1.298718" +" "+ "103.770778" +"\n"+
		"1.299048" +" "+ "103.770617" +"\n"+
		"1.299070" +" "+ "103.770118" +"\n"+
		"1.299352" +" "+ "103.769957" +"\n"+
		"1.300279" +" "+ "103.770150" +"\n"+
		"1.300775" +" "+ "103.770702" +"\n"+
		"1.300579" +" "+ "103.771263";
		
		public Webpage(ClientData cd) {
    		this.cd = cd;
    	}
		
    	public void run() {
			while(true){
				try{
					synchronized(lockObject){
						if (isServerToWeb) {
							System.out.println("Server->Web: "+";"+cd.ID+":"+cd.latitude+":"+cd.longitude);
							cd.out.println(latitude+" "+longitude);
							isServerToWeb = false;
							
							try {
								Thread.sleep(500);
					        } catch( InterruptedException e ) {
								System.out.println("Interrupted");
            				}
            			}
					}
				} catch (Exception e) {
					System.err.println("IOException occured: " + e.getMessage());
					continue;
				}
			}
    	} 
    }
    
    
    
}
