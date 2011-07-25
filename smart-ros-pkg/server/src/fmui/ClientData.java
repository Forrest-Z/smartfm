package fmui;

import java.net.*;
import java.io.*;
import java.util.*;

class ClientData {
    
    /** Information about known clients: ID, socket, etc. */
    
    public Socket socket = null;
    public String ID = "-1";
    public String user_id = "0";
    public String pickup = "0";
    public String dest = "0";
    public String wait = "0";
    public String carID = "0";
    public String taskID = "0";
    public String latitude = "0";
    public String longitude = "0";
    public BufferedReader in;
    public PrintWriter out;
    
       
    private final int USER = 1;
    private final int SCHEDULER = 2;
    private final int CAR = 3;
    private final int DESKTOP = 4;
    private final int ERR = 5;
    
	
    public ClientData(Socket socket) throws IOException {
        this.socket = socket;
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out = new PrintWriter(socket.getOutputStream(), true);
    }
	
    public boolean readMsg(String IDcheck) throws IOException {
        String msg = null; 
        while(in.ready()&&(msg = in.readLine())!=null) {    	
            if (msg.startsWith(";")) {
            	int f = msg.indexOf(":");
            	ID = msg.substring(1,f);
				
				System.out.println("Check msg to Server: " + msg);
				
				 // store Scheduler data. --- ;ID:user_id:taskID:wait:carID
				if(IDcheck.compareTo("0")==0&&ID.compareTo("0")==0) {
	            	int s = msg.indexOf(":", f+1);
	            	int t = msg.indexOf(":", s+1);
	            	int i = msg.indexOf(":", t+1);
	            	
	            	user_id = msg.substring(f+1,s);
	            	taskID = msg.substring(s+1,t);
	                wait = msg.substring(t+1,i);
	                carID = msg.substring(i+1);
	                //break;
	                return true;
            	} else if(IDcheck.compareTo("car")==0&&ID.compareTo("car")==0){
                                     
	            	int s = msg.indexOf(":", f+1);
	            	int t = msg.indexOf(":", s+1);

                    taskID = msg.substring(f+1,s);
                    latitude = msg.substring(s+1,t);
                    longitude = msg.substring(t+1);
                    //break;
                    return true;
                } else if(IDcheck.compareTo("Desktop")==0&&ID.compareTo("Desktop")==0) {
                	//break;
                	return true;
                } else  {	// store User data. --- ;ID:taskID:pickup:dest
	            	int s = msg.indexOf(":", f+1);
	            	int t = msg.indexOf(":", s+1);
	            	
	            	taskID = msg.substring(f+1,s);
	                pickup = msg.substring(s+1,t);
	                dest = msg.substring(t+1);
	                //break;
	                return true;
                }  
            }
        }
        return false;
    }
    
    public String checkID() throws IOException{
    	String msg = in.readLine();
    	String ID = null;
    	int f = msg.indexOf(":");
    	
    	if (msg.startsWith(";")){
    		ID = msg.substring(1,f);
    		return ID;
    	} 
    	
    	return ID;
    	
    }
    
    
    
    
     public int initConnection() throws IOException{
		String msg = null;
		

		
    	while((msg = in.readLine())!=null) {   
    	
    	System.out.println("Check msg to Server: " + msg);
    	
    	if (msg.startsWith(";")){
    		int f = msg.indexOf(":");
    		ID = msg.substring(1,f);
    	}
    	
    	if (ID!=null) {
    	if (ID.compareTo("0")==0)
    		return SCHEDULER;
    	else if (ID.compareTo("car")==0)
    		return CAR;
    	else if (ID.compareTo("Desktop")==0)
    		return DESKTOP;
    	else
    		return USER;
    	} else 
    		return ERR;   	
    	}

    	return ERR;
    }
    
    
    
    
    
    public void reconnect(ClientData c) throws IOException {
        socket = c.socket;
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out = new PrintWriter(socket.getOutputStream(), true);
    }
    
    public void checkData(){
    	System.out.println("CHeck data in ClientData: "+ "ID: "+ ID + " user_id: "
						   + user_id + " pickup: " + pickup + " dest: " + dest + " wait: " 
						   + wait + " carID: " + carID);
    	return;
    }
    
    public void close() throws IOException {
        //out.println("Bye");
        out.close();
        in.close();
        socket.close();
    }
}
