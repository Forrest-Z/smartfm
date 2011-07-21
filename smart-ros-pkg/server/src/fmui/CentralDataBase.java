package fmui;

import java.util.*;
import java.io.IOException;

public class CentralDataBase {
    
    /** Central data base on the server.
	 Creates several server instances on various ports. Maintains
	 a list of known clients.
	 */
	
    public List<ClientData> clients = Collections.synchronizedList(new ArrayList<ClientData>());
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
                //System.out.println("add server thread");
                //System.out.println(p);
                //System.out.println(serverThreads.size());
            }
        }
    }
}
