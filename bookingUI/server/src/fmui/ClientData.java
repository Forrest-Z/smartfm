package fmui;

import java.net.*;
import java.io.*;

/** A base class for data types */
public class ClientData {

    public static final int USER = 1;
    public static final int SCHEDULER = 2;
    public static final int CAR = 3;
    public static final int WEBPAGE = 4;
    public static final int ERR = 5;

    private int clientID = -1;
    private Socket socket = null;
    private BufferedReader in;
    private PrintWriter out;

    public ClientData(Socket socket) throws IOException {
        this.socket = socket;
        in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
        out = new PrintWriter(socket.getOutputStream(), true);
    }

    /** Waits for incomming data to identify the client. */
    public int initClient() throws IOException {
        String data = null;

        while((data = in.readLine())!=null) {
            System.out.println("Init client: " + data);

            if (data.startsWith(";")) {
                String test = data.substring(1,data.indexOf(":"));

                if (test != null) {
                    if (test.compareTo("0")==0)
                        return SCHEDULER;
                    else if (test.compareTo("car")==0)
                        return CAR;
                    else if (test.compareTo("webpage")==0)
                        return WEBPAGE;
                    else
                        return USER;
                } else {
                    return ERR;
                }
            }
        }

        return ERR;
    }

    public String readData() throws IOException {
        String data = null;
        while(in.ready() && (data = in.readLine())!=null)
            break;
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
