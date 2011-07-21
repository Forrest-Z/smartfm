package fmui;

import java.io.IOException;


public class FMUI {
    
    /** Main entry point.
	 Runs both the server side and the client side. Use a keyword on the
	 command line to use one or the other.
	 */
    
    static int[] parsePorts(String[] args) {
        for( int i=0; i<args.length; i++ ) {
            if( args[i].compareTo("-p")==0 ) {
                if (i+1>=args.length) {
                    System.err.println("Option -p must be followed by a colon separated list of ports");
                    System.exit(1);
                }
                String[] portlist = args[i+1].split(":");
                int[] ports = new int[portlist.length];
                for( int j=0; j<portlist.length; j++ )
                    ports[j] = Integer.parseInt(portlist[j]);
                return ports;
            }
        }
        
        int[] ports = {4440, 4441, 1024, 8080};
        return ports;
    }
    
    static String p2s(int[] ports) {
        /** For debugging purpose */
        StringBuilder sb = new StringBuilder();
        for( int i=0; i<ports.length; i++ ) {
            if( i<ports.length-1 )
                sb.append(""+ports[i]+":");
            else
                sb.append(ports[i]);
        }
        return sb.toString();
    }
    
    static void main_server(String[] args) {
        int[] ports = parsePorts(args);
        //System.out.println("ports: " + p2s(ports));
        
        CentralDataBase db = new CentralDataBase(ports);
        while(true) {
            try {
                Thread.sleep(1000);
            } catch( InterruptedException e ) {
            }
        }
    }
	
    static void main_client(String[] args) {
        String hostname = null; //null --> localhost
        String ID = null;
        String pickup = null;
        String dest = null;
        
        for( int i=0; i<args.length; ) {
            if( args[i].compareTo("-h")==0 ) {
                hostname = args[i+1];
                ID = args[i+2];
                i += 3;
            }
            else if( args[i].compareTo("-i")==0 ) {
                ID = args[i+1];
                pickup = args[i+2];
                dest = args[i+3];
                i+=4;
            }
            else
                i++;
        }
        
        int [] ports = parsePorts(args);
        //System.out.println("ports: " + p2s(ports));
        
        Client c = null;
        for( int p: ports ) {
            try {
                c = new Client(hostname,p,ID,pickup,dest);
				//    c.close();
            } catch (Exception e) {
                System.err.println("Error connecting to " + hostname + ":" + p);
            }
            if (c!=null) {
                System.out.println("Connected to " + hostname + ":" + p);
                break;
            }
        }
        if (c==null) {
            System.err.println("Could not connect to " + hostname);
            System.exit(1);
        }
        
        try {
            c.out.println("Hello");
            c.close();
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }
    
    public static void main(String[] args) {
        if( args.length==0 ) {
            System.err.println("You must indicate if you want to run as client or server");
            System.exit(1);
        }
        if( args[0].compareTo("server")==0 )
            main_server(args);
        else
            main_client(args);
    }
	
}
