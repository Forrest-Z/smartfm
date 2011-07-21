package fmui;

import java.io.*;
import java.net.*;

public class Client {
    
    /** The client (customer of vehicle side).
    Connects to the server and identifies itself (with ID).
    Does not do anything else for now!
    */
    
    public Socket socket = null;
    public PrintWriter out = null;
    public BufferedReader in = null;
    public String ID = "-1";
    public String user_id = "-1";
    public String host = "0";
    public String pickup = "0";
    public String dest = "0";
    public String wait = "0";
    public String carID = "0";
    public int port = -1;
    
    //Server --> Scheduler
    public Client(String host, int port, String ID, String pickup, String dest) throws UnknownHostException, IOException {
        this.user_id = ID;
        this.host = host;
        this.port = port;
        this.pickup = pickup;
        this.dest = dest;
        
        socket = new Socket(host, port);
        out = new PrintWriter(socket.getOutputStream(), true);
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out.println(";"+user_id+":"+pickup+":"+dest);
        //this.close();
    }
    
    //Server --> Android
    public Client(String host, int port, String wait, String carID) throws UnknownHostException, IOException {
        this.host = host;
        this.port = port;
        this.wait = wait;
        this.carID = carID;
        
        socket = new Socket(host, port);
        out = new PrintWriter(socket.getOutputStream(), true);
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out.println(";"+ wait + ":" + carID);
    }
    
    public void close() throws IOException {
        //out.println("Bye");
        out.close();
        in.close();
        socket.close();
    }
}
