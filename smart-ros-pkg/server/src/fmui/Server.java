package fmui;

import java.net.*;
import java.io.*;

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
    String pickup = "0";
    String dest = "0";
    String user_id = "0";
    String wait = "0";
    String carID = "0";
    String taskID = "0";
    
    
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
				c.readMsg();
				
				if (c.ID.compareTo("0")==0) {
					System.out.println("Server");
					new ServerThread(c).start();
				} else {
					System.out.println("User");
					new UserThread(c).start();
				}
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
			// Sleep 2s
            try {
                Thread.sleep(2000);
            } catch( InterruptedException e ) {
            }
			
			
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
             				   Thread.sleep(1000);
					            } catch( InterruptedException e ) {
            				}
						} else{ // if not order, keep reading from Scheduler
							// read msgs from Scheduler
							cd.readMsg();
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
             				   Thread.sleep(1000);
					            } catch( InterruptedException e ) {
            				}
						} else {
							// read msg from User
							cd.readMsg();
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
				}catch (IOException e) {
					System.err.println("IOException occured: " + e.getMessage());
					continue;
				}
			}
    	} 
    }
}