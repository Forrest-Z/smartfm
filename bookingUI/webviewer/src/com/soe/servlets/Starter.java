package com.soe.servlets;

import java.io.IOException;

import javax.servlet.ServletException;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;

import com.soe.client.FMUI;

/**
 * Servlet implementation class Starter
 */
public class Starter extends HttpServlet {
	private static final long serialVersionUID = 1L;
    
	public FMUI begin = new FMUI();
	
    public Starter() {
        super();
        begin.startClient();
    }
    
	protected void doGet(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException
	{
		performTask(request, response);
	}
	
	private void performTask(HttpServletRequest request, HttpServletResponse response) throws ServletException, IOException {
		
		String inputMessage = begin.c.readNewMessage();
		System.out.println(inputMessage);
		int temp1 = inputMessage.indexOf(" ");
		int temp2 = inputMessage.length();
		String lat = inputMessage.substring(0, temp1);
        String lng = inputMessage.substring(temp1+1, temp2);
        String ID = begin.c.ID;
        request.setAttribute("latitude", lat);
        request.setAttribute("longitude", lng);
        request.setAttribute("ID", ID);
        //String message = "12345";
        //request.setAttribute("message", message);
        response.addHeader("Refresh", "6");
        request.getRequestDispatcher("/ShowMap.jsp").forward(request, response);
	}
}
