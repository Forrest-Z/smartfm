package fmui;

import java.io.IOException;
import java.net.Socket;

public class SchedulerData extends ClientData {
    //private DataFromServerToScheduler dataToScheduler = new DataFromServerToScheduler();
    private DataFromSchedulerToServer dataToServer = new DataFromSchedulerToServer();

    private int schedulerID = -1;

    public SchedulerData(Socket socket) throws IOException {
        super(socket);
    }

    public void sendDataToScheduler(String dataSentToScheduler){
        super.sendData(dataSentToScheduler);
    }

    public boolean readSchedulerData() throws IOException {
        String data = null;
        boolean isReady = false;

        data = super.readData();
        if (data != null) {
            if (data.startsWith(";")) {
                int f = data.indexOf(":");
                int s = data.indexOf(":", f+1);
                int t = data.indexOf(":", s+1);
                int i = data.indexOf(":", t+1);

                //schedulerID = Integer.parseInt(data.substring(1,f));
                schedulerID = 0;
                dataToServer.userID = Integer.parseInt(data.substring(f+1,s));
                dataToServer.taskID = Integer.parseInt(data.substring(s+1,t));
                dataToServer.waitTime = Integer.parseInt(data.substring(t+1,i));
                dataToServer.carID = Integer.parseInt(data.substring(i+1));
            }
            isReady = true;
        }

        return isReady;
    }

    public int getID() {
        return schedulerID;
    }

    public int getUserID() {
        return dataToServer.userID;
    }

    public int getTaskID() {
        return dataToServer.taskID;
    }

    public int getWaitTime() {
        return dataToServer.waitTime;
    }

    public int getCarID() {
        return dataToServer.carID;
    }

    public void setDataToScheduler( ) {

    }

    public DataFromSchedulerToServer getDataToServer() {
        return dataToServer;
    }
}