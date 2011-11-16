package fmui;

import java.io.IOException;
import java.net.Socket;

public class WebpageData extends ClientData{

    public WebpageData(Socket socket) throws IOException {
        super(socket);
    }

    public void sendDataToWebpage(String dataToWebpage) {
        super.sendData(dataToWebpage);
    }

}
