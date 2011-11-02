package com.soe.client;

import java.io.*;
import java.net.*;

public class Client {
        
    public Socket socket = null;
    public PrintWriter out = null;
    public BufferedReader in = null;
    public String ID = null;
    public String host = null;
    public int port = -1;
    public String message = null;
    public String IDFormatted = null;
    
    public Client(String host, int port, String ID) throws UnknownHostException, IOException {
        this.ID = ID;
        this.host = host;
        this.port = port;
        
        socket = new Socket(host, port);
        out = new PrintWriter(socket.getOutputStream(), true);
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        
        IDFormatted = ";webpage:" + ID + ":0:0";
        out.println(IDFormatted);
    }
    
    public String readNewMessage(){
    	try {
			message = in.readLine();
		} catch (IOException e) {
			e.printStackTrace();
		  }
		return message;
    }
    
    public void close() throws IOException {
        out.println("Bye");
        out.close();
        in.close();
        socket.close();
    }
}
