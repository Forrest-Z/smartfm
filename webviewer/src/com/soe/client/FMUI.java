package com.soe.client;

public class FMUI {
	
	public String hostname = null; 			//null --> localhost
    public int port = 0;
    public String ID = "0";
    public Client c = null;
    
    public void startClient() {
        hostname = "172.17.163.129";		//Hostname of server
        port = 4440;						//Port of server
        ID = "1";							//Default ID
        
        try {
        	c = new Client(hostname,port,ID);
        } catch (Exception e) {
        	System.err.println("Error connecting to " + hostname + ":" + port);
          }
        
        if (c!=null) {
        	System.out.println("Car " + ID + " Successfully Connected to " + hostname + ":" + port);
        }
        if (c==null) {
            System.err.println("Could not connect to " + hostname);
            //System.exit(1);
        }
    }
}
