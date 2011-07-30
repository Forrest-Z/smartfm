package fmui;

import java.net.*;
import java.io.*;

class ClientData {
    
    /** Information about known clients: ID, socket, etc. */
    private int clientID = -1;
    private Socket socket = null;
    private BufferedReader in;
    private PrintWriter out;
    
    private final int USER = 1; 
    private final int SCHEDULER = 2;
    private final int CAR = 3;
    private final int WEBPAGE = 4;
    private final int ERR = 5;
	
    private String test = null;
    
    public ClientData(Socket socket) throws IOException {
        this.socket = socket;
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out = new PrintWriter(socket.getOutputStream(), true);
    }    
    
    public int initCleint() throws IOException {
    	String data = null;
    	
    	while((data = in.readLine())!=null) {   
			System.out.println("Init client: " + data);
			
			if (data.startsWith(";")){
				int f = data.indexOf(":");
				/*
				clientID = Integer.parseInt(data.substring(1,f));
				
				if (clientID != -1) {
					if (clientID == 0)
						return SCHEDULER;
					else if (clientID >= 1 && clientID < 10)
						return CAR;
					else if (clientID >= 10 && clientID <20)
						return WEBPAGE;
					else if (clientID >=20)
						return USER;
				} else
					return ERR;
				*/
				test = data.substring(1,f);
				
				if (test!=null){
					if (test.compareTo("0")==0)
						return SCHEDULER;
					else if (test.compareTo("car")==0)
						return CAR;
					else if (test.compareTo("webpage")==0)
						return WEBPAGE;
					else 
						return USER;
				} else
					return ERR;
				
			} 
    	}
    	
    	return ERR;
    }
    
    public String readData() throws IOException {
    	String data = null;
		
		while(in.ready() && (data = in.readLine())!=null) {
		}
		return data;
    }
    
    public void sendData(String data) {
    	out.println(data);
    }
    
  
    public int getID() {
    	return clientID;
    }
    
    public Socket getSocket() {
    	return socket;
    }
    
    public boolean readyForRead() throws IOException{
		return in.ready();
	}
    
    public void reconnect(ClientData c) throws IOException {
        socket = c.socket;
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out = new PrintWriter(socket.getOutputStream(), true);
    }

    public void close() throws IOException {
        //out.println("Bye");
        out.close();
        in.close();
        socket.close();
    }
}
