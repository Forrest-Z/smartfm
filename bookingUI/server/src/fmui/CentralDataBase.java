package fmui;

import java.util.*;
import java.io.IOException;

public class CentralDataBase {

    /** Central data base on the server.
    Creates several server instances on various ports. Maintains
    a list of known clients.
    */

    public List<UserData> clients = Collections.synchronizedList(new ArrayList<UserData>());
    List<Thread> serverThreads = new ArrayList<Thread>();
    private String dataFromCarToWeb = null;
    private String dataFromCarToUser = null;

    public CentralDataBase(int[] ports) {
        for( int p:ports ) {
            Server s = null;
            try {
                s = new Server(p,this);
            } catch (IOException e) {
                System.err.println("Could not start a server on port " + p);
            }
            if (s!=null) {
                Thread t = new Thread(s);
                serverThreads.add(t);
                t.start();
            }
        }
    }

    public void updatePickupAndDropoff(int id, DataFromUserToServer d) {
        for( int i=0; i<clients.size(); i++ ) {
            if( clients.get(i).getID() == id) {
                clients.get(i).setPickupOption(d.pickupOption);
                clients.get(i).setDropoffOption(d.dropoffOption);
                break;
            }
        }
    }

    public void updateWaitTimeAndCarID(int id, DataFromSchedulerToServer d) {
        for( int i=0; i<clients.size(); i++ ) {
            if( clients.get(i).getID() == id ) {
                clients.get(i).setTaskIDAndWaitTimeAndCarID(d.taskID, d.waitTime, d.carID);
                break;
            }
        }
    }

    public void setDataFromCarToWeb(String s) {
        this.dataFromCarToWeb = s;
    }

    public void setDataFromCarToUser(String s) {
        this.dataFromCarToUser = s;
    }

    public String getDataFromCarToWeb(){
        return dataFromCarToWeb;
    }

    public String getDataFromCarToUser(){
        return dataFromCarToUser;
    }
}
