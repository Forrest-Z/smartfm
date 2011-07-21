package fmui;

import java.net.*;
import java.io.*;

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
    public BufferedReader in;
    public PrintWriter out;
	
    public ClientData(Socket socket) throws IOException {
        this.socket = socket;
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out = new PrintWriter(socket.getOutputStream(), true);
    }
	
    public void readMsg() throws IOException {
        String msg = null; 
        while(in.ready() && (msg = in.readLine())!=null) {    	
            if (msg.startsWith(";")) {
            	int f = msg.indexOf(":");
            	ID = msg.substring(1,f);
				
				System.out.println("CHeck msg to Server: " + msg);
				
            	// store User data. --- ;ID:taskID:pickup:dest
            	if (ID.compareTo("0")!=0)
            	{
	            	int s = msg.indexOf(":", f+1);
	            	int t = msg.indexOf(":", s+1);
	            	
	            	taskID = msg.substring(f+1,s);
	                pickup = msg.substring(s+1,t);
	                dest = msg.substring(t+1);
	                break;
                }
            	else // store Scheduler data. --- ;ID:user_id:taskID:wait:carID
            	{
	            	int s = msg.indexOf(":", f+1);
	            	int t = msg.indexOf(":", s+1);
	            	int i = msg.indexOf(":", t+1);
	            	
	            	user_id = msg.substring(f+1,s);
	            	taskID = msg.substring(s+1,t);
	                wait = msg.substring(t+1,i);
	                carID = msg.substring(i+1);
	                break;
            	}
            }
        }
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