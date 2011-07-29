package fmui;

import java.util.*;
import java.io.IOException;

class CentralDataBase {
    
    /** Central data base on the server.
	 Creates several server instances on various ports. Maintains
	 a list of known clients.
	 */
	
    public List<UserData> clients = Collections.synchronizedList(new ArrayList<UserData>());
    List<Thread> serverThreads = new ArrayList<Thread>();
	
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
    
    public void updatePickupAndDropoff(int dataFromServerToSchedulerID, DataFromUserToServer dataFromUserToServer) {
    	for( int i=0; i<clients.size(); i++ ) {
			if( clients.get(i).getID() == dataFromServerToSchedulerID) {
				clients.get(i).setPickupOption(dataFromUserToServer.pickupOption);
				clients.get(i).setDropoffOption(dataFromUserToServer.dropoffOption);
				break;
			}
		}
    }
    
    public void updateWaitTimeAndCarID(int dataFromSchedulerToServerID, DataFromSchedulerToServer dataToServer) {
    	for( int i=0; i<clients.size(); i++ ) {
			if( clients.get(i).getID() == dataFromSchedulerToServerID) {
				clients.get(i).setWaitTime(dataToServer.waitTime);
				clients.get(i).setCarID(dataToServer.carID);
				break;
			}
		}
    }
}
