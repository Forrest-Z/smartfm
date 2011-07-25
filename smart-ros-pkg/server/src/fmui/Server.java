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
    static class theLock extends Object{ }
    static public theLock lockObject = new theLock();
    
    ServerSocket serverSocket = null;
    CentralDataBase db = null;
    int port = -1;
    boolean toScheduler = false;
    boolean toUser = false;
    //boolean toWeb = false;
    boolean toWeb = true;
    String pickup = "0";
    String dest = "0";
    String user_id = "0";
    String wait = "0";
    String carID = "0";
    String taskID = "0";
    String latitude = "0";
    String longitude = "0";
    
    
     private int clientType = -1;
    
    private final int USER = 1;
    private final int SCHEDULER = 2;
    private final int CAR = 3;
    private final int DESKTOP = 4;
    private final int ERR = 5;
    
    
    public Server(int port, CentralDataBase db) throws IOException {
        this.port = port;
        serverSocket = new ServerSocket(port);
        this.db = db;
    }
	
    public void run() {
		while (true) {

            ClientData c = null;
            try {
            	System.out.println(serverSocket.toString());
                c = new ClientData(serverSocket.accept());
                System.out.println("after"+serverSocket.toString());
            } catch (IOException e) {
                // Print the error and exit
                // A better mechanism should be implemented to continue accepting connections...
                System.err.println("IOException occured: " + e.getMessage());
                System.exit(1);
            }
			try{

				
				clientType = c.initConnection();
				
				
				if (clientType==SCHEDULER) {
					System.out.println("Scheduler");
					new ServerThread(c).start();
				} else if (clientType==DESKTOP){
                    System.out.println("Desktop");
                    new DesktopThread(c).start();
                } else if (clientType==CAR){
                    System.out.println("Car");
                    new CarThread(c).start();
                } else if (clientType==USER){
					System.out.println("User");
					new UserThread(c).start();
			    } else
			    	System.out.println("Unknown type." );
	               // System.exit(1);
	                
	                
                
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
				System.out.println("Number of clients: " + db.clients.size() +"\n\n");
			}
			
			
			/*
			// Sleep 2s
            try {
                Thread.sleep(2000);
            } catch( InterruptedException e ) {
            }
			*/
			
    	}
	}
	
	// Server Thread
	private class ServerThread extends Thread {
    	private ClientData cd = null;
    	
    	public ServerThread(ClientData cd) {
    		this.cd = cd;
    	}
    	
    	public void run() {
			
			while(true){
				synchronized(lockObject){
					try {			
						// if User sends order to Server, Server forwards the msgs to Scheduler
						if (toScheduler)
						{
							System.out.println("Server-->Scheduler");
							// send to Scheduler
							cd.out.println(";"+Integer.parseInt(user_id)+":"+taskID+":"+pickup+":"+dest);
							// clean received msgs
							toScheduler = false;
							taskID = "0";
							user_id = "0";
							pickup = "0";
							dest = "0";
							
							try {
             				   Thread.sleep(500);
					            } catch( InterruptedException e ) {
            				}
						} else{ // if not order, keep reading from Scheduler
							// read msgs from Scheduler
							
							if (cd.readMsg("0")){
							
							// Scheduler sends feedback to none-zero User ID
							if (cd.user_id.compareTo("0")!=0)
							{
								System.out.println("Scheduler-->Server");
								// set transfer msgs
								toUser = true;
								taskID = cd.taskID;
								wait = cd.wait;
								carID = cd.carID;
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
    
    private class UserThread extends Thread {
    	private ClientData cd = null;
    	
    	public UserThread(ClientData cd) {
    		this.cd = cd;
    	}
		
    	public void run() {
			while(true){
				try{
					synchronized(lockObject){
						// if Server receives from Scheduler, then forwards to User
						if (toUser)
						{	
							System.out.println("Server-->User");
							cd.out.println(";"+taskID+":"+wait+":"+carID);
							taskID = "0";
							wait = "0";
							carID = "0";
							toUser = false;
							
							try {
             				   Thread.sleep(500);
					            } catch( InterruptedException e ) {
            				}
						} else {
							// read msg from User
							if (cd.readMsg("user")) {
							// if User sends order, receive it
							if (cd.pickup.compareTo("0")!=0&&cd.dest.compareTo("0")!=0)
							{
								System.out.println("User-->Server");
								toScheduler = true;
								user_id = cd.ID;
								taskID = cd.taskID;
								pickup = cd.pickup;
								dest = cd.dest;
								
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

    private class CarThread extends Thread{
        private ClientData cd = null;

        public CarThread(ClientData cd){
            this.cd = cd;
        }

        public void run(){
            while(true){
                try{  
                    synchronized(lockObject){

                        if (cd.readMsg("car")){
                        
                        if(cd.taskID.compareTo("1")==0){
                            System.out.println("Car-->Server");
                            latitude = cd.latitude;
                            longitude = cd.longitude;

                            System.out.println("lat: "+latitude+" long: "+longitude);                            
                            
                            
                            toWeb = true;
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
    
    private class DesktopThread extends Thread {
    	
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
	
	public DesktopThread(ClientData cd) {
    		this.cd = cd;
    	}
		
    	public void run() {
			while(true){
				try{
					synchronized(lockObject){
						if (toWeb) {
						//Put code here
						
						/*
						ArrayList arr = new ArrayList(); //size determined from arr.size();
						arr.add(coordinates);
						if (!arr.isEmpty()) {
						  Object temp = arr.get(arr.size()-1);
						  cd.out.println(temp);		
							toWeb = false;
                        }
                        */
                        
                        cd.out.println(latitude+" "+longitude);
                        toWeb = false;
                        
                        
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
