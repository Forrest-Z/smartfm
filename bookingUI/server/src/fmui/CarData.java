package fmui;

import java.io.IOException;
import java.net.Socket;

public class CarData extends ClientData {
    private int carID = -1;
    private DataFromCarToServer dataToServer = new DataFromCarToServer();

    public CarData(Socket socket) throws IOException {
        super(socket);
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

                //carID = Integer.parseInt(data.substring(1,f));
                carID = 1;
                dataToServer.taskID = Integer.parseInt(data.substring(f+1,s));
                dataToServer.latitude = Double.parseDouble(data.substring(s+1,t));
                dataToServer.longitude = Double.parseDouble(data.substring(t+1));
            }
            isReady = true;
        }

        return isReady;
    }

    public int getCarID() {
        return carID;
    }

    public int getTaskID() {
        return dataToServer.taskID;
    }

    public double getLatitude() {
        return dataToServer.latitude;
    }

    public double getLongitude() {
        return dataToServer.longitude;
    }
}
