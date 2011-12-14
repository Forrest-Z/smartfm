package com.smartfm.phoneui;


import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import org.apache.http.HttpResponse;
import org.apache.http.NameValuePair;
import org.apache.http.client.HttpClient;
import org.apache.http.client.entity.UrlEncodedFormEntity;
import org.apache.http.client.methods.HttpPost;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.message.BasicNameValuePair;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.InputSource;


/** A helper class to call the PHP scripts used to interface with the database. */
class RPC {
		
	String url;
	String scriptName;
	List<NameValuePair> nameValuePairs;
	
	public RPC(String phpscript) {
		this.scriptName = phpscript;
		this.url = DBInterface.BaseURI + phpscript;
		nameValuePairs = new ArrayList<NameValuePair>(1);
		addParameter("CustomerID", DBInterface.CustomerID);
	}
	
	public RPC addParameter(String name, String value) {
		nameValuePairs.add(new BasicNameValuePair(name,value));
		return this;
	}
	
	public Document call() throws Exception {
		InputStream in = null;
		Document dom = null;
		try {
			HttpClient client = new DefaultHttpClient();
			HttpPost request = new HttpPost(url);
			request.setEntity(new UrlEncodedFormEntity(nameValuePairs));
			HttpResponse response = client.execute(request);
			in = response.getEntity().getContent();
			
			BufferedReader rd = new BufferedReader(new InputStreamReader(in));
			String line = "", xml="";
			while ((line = rd.readLine()) != null)
				xml += line + "\n";
			
			DocumentBuilderFactory factory = DocumentBuilderFactory
					.newInstance();
			DocumentBuilder builder = factory.newDocumentBuilder();
			dom = builder.parse(new InputSource(new StringReader(xml)));
			
			NodeList items = dom.getDocumentElement().getElementsByTagName("status");
			for (int i = 0; i < items.getLength(); i++) {
				Element item = (Element) items.item(i);
				if( item.getAttribute("code").equalsIgnoreCase("err") ) {
					String msg = "RPC call to " + scriptName + " failed";
					if( item.hasAttribute("msg") )
						msg += " (" + item.getAttribute("msg") +").";
					else
						msg += ".";
					throw new RuntimeException(msg);
				}
			}
		} finally {
			if (in != null) {
				in.close();
			}
		}
		return dom;
	}
}

/**
 * A collection of static methods to interface with the database via the PHP
 * interface.
 */
public class DBInterface {
	
	public static final String BaseURI = "http://fmautonomy.no-ip.info/dbserver/";
	public static final String CustomerID = "cust1";


	public static List<Task> listTasks() throws Exception {
		List<Task> tasks = new ArrayList<Task>();
		Document dom = new RPC("list_requests.php").call();
		NodeList items = dom.getDocumentElement().getElementsByTagName("request");
		for (int i = 0; i < items.getLength(); i++) {
			Task task = new Task();
			Element item = (Element) items.item(i);

			task.customerID = item.getAttribute("customerID");
			task.requestID = Integer.parseInt(item.getAttribute("requestID"));
			task.status = item.getAttribute("status");
			task.pickup = item.getAttribute("pickup");
			task.dropoff = item.getAttribute("dropoff");

			if (item.hasAttribute("vehicleID")) {
				task.vehicleID = item.getAttribute("vehicleID");
				task.latitude = Double.parseDouble(item.getAttribute("latitude"));
				task.longitude = Double.parseDouble(item.getAttribute("longitude"));
				task.eta = Integer.parseInt(item.getAttribute("eta"));
			}

			tasks.add(task);
		}
		return tasks;
	}
	
	public static Task getTask(int taskID) throws Exception {
		Document dom = new RPC("list_requests.php")
						.addParameter("RequestID", ""+taskID)
						.call();
		NodeList items = dom.getDocumentElement().getElementsByTagName("request");
		
		if( items.getLength()==0 )
			throw new RuntimeException("Task " + taskID + " not found.");
		if( items.getLength()>1 )
			throw new RuntimeException("Too many tasks returned.");
		
		Task task = new Task();
		Element item = (Element) items.item(0);

		task.customerID = item.getAttribute("customerID");
		task.requestID = Integer.parseInt(item.getAttribute("requestID"));
		task.status = item.getAttribute("status");
		task.pickup = item.getAttribute("pickup");
		task.dropoff = item.getAttribute("dropoff");

		if (item.hasAttribute("vehicleID")) {
			task.vehicleID = item.getAttribute("vehicleID");
			task.latitude = Double.parseDouble(item.getAttribute("latitude"));
			task.longitude = Double.parseDouble(item.getAttribute("longitude"));
			task.eta = Integer.parseInt(item.getAttribute("eta"));
		}

		return task;
	}
	
	public static int addTask(String pickup, String dropoff) throws Exception {
		//check that the stations exist. If they don't an exception will
		//be thrown.
		StationList stations = new StationList();
		stations.getStation(pickup);
		stations.getStation(dropoff);
		
		//all right, stations are valid. Make the booking.
		Document dom = new RPC("new_request.php")
			.addParameter("PickUpLocation", pickup)
			.addParameter("DropOffLocation",dropoff)
			.call();
		
		//retrieve the task id and return it.
		NodeList items = dom.getDocumentElement().getElementsByTagName("taskadded");
		Element item = (Element) items.item(0);
		return Integer.parseInt(item.getAttribute("id"));
	}
	
	public static void cancelTask(int taskID) throws Exception {
		new RPC("cancel_request.php")
			.addParameter("RequestID", ""+taskID)
			.call();
	}
}
