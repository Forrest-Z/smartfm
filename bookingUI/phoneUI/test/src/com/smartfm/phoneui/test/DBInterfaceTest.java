package com.smartfm.phoneui.test;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import com.smartfm.phoneui.*;

import android.test.AndroidTestCase;

public class DBInterfaceTest extends AndroidTestCase {

	protected StationList stations;
	protected ArrayList<String> names;
	
	protected void setUp() {
		stations = new StationList();
		names = stations.allNames();
	}
	
	public void testTaskEqualHash() {
		Task t11 = new Task();
		t11.requestID = 1;
		t11.pickup = names.get(0);
		t11.dropoff = names.get(1);
		t11.customerID = DBInterface.CustomerID;
		
		Task t12 = new Task();
		t12.requestID = 1;
		t12.pickup = names.get(0);
		t12.dropoff = names.get(1);
		t12.customerID = DBInterface.CustomerID;
		
		// 2 different objects
		assertTrue(t11!=t12);
		
		// But all relevant fields are equals
		assertTrue(t11.equals(t12));
		assertEquals(t11, t12);
		assertTrue(t11.hashCode()==t12.hashCode());
		
		// When only irrelevant fields are different, 
		// equality should be maintained.
		t11.status = "a";
		t12.status = "b";
		assertTrue(t11.equals(t12));
		assertEquals(t11, t12);
		assertTrue(t11.hashCode()==t12.hashCode());
		
		// Changing a relevant field should brake equality.
		t11.dropoff = names.get(2);
		assertFalse(t11.equals(t12));
		assertFalse(t11.hashCode()==t12.hashCode());
	}
	
	public void testTaskSerialize() {
		Task t11 = new Task();
		t11.requestID = 1;
		t11.pickup = names.get(0);
		t11.dropoff = names.get(1);
		t11.customerID = DBInterface.CustomerID;
		
		String ser = t11.toString();
		Task t12 = Task.fromString(ser);
		
		assertEquals(t11, t12);
	}
	
	public void testTaskSets() {
		Task t11 = new Task();
		t11.requestID = 1;
		t11.pickup = names.get(0);
		t11.dropoff = names.get(1);
		t11.customerID = DBInterface.CustomerID;
		
		Task t12 = new Task();
		t12.requestID = 1;
		t12.pickup = names.get(0);
		t12.dropoff = names.get(1);
		t12.customerID = DBInterface.CustomerID;
		
		Set<Task> set = new HashSet<Task>();
		set.add(t11);
		set.add(t12);
		assertTrue(set.size()==1);
		
		set.remove(t12);
		assertTrue(set.isEmpty());
	}

	public void testAddTask() {
		// Adding wrong stations should throw a StationDoesNotExistException.
		try {
			DBInterface.addTask("dummypickup", "dummydropoff");
		} catch (StationDoesNotExistException e) {
			assertTrue(true);
		} catch (Exception e) {
			e.printStackTrace();
			assertTrue(false);
		}
		
		// But adding a real station should not fail
		try {
			DBInterface.addTask(names.get(0), names.get(1));
		} catch (Exception e) {
			e.printStackTrace();
			assertTrue(false);
		}
	}

	public void testAddListCancelTask() {
		try {
			// First get the current list of tasks.
			List<Task> tasks_pre = DBInterface.listTasks();
			
			// Then add a task.
			String pickup = names.get(0);
			String dropoff = names.get(1);
			int taskid = DBInterface.addTask(pickup, dropoff);
			
			// Then get the new list of task.
			List<Task> tasks_post = DBInterface.listTasks();
			
			// Substracts the two lists using a set.
			HashSet<Task> tasks_set = new HashSet<Task>();
			tasks_set.addAll(tasks_post);
			tasks_set.removeAll(tasks_pre);
			
			// There should be only one task in the difference between the 2 sets.
			assertTrue(tasks_post.size() == tasks_pre.size()+1);
			assertTrue(tasks_set.size()==1);

			// Retrieve the task we just added
			Task task = DBInterface.getTask(taskid);
			assertEquals(task.requestID, taskid);
			assertEquals(task.pickup, pickup);
			assertEquals(task.dropoff, dropoff);
			assertEquals(task.customerID, DBInterface.CustomerID);
			assertEquals(task, tasks_set.iterator().next());
		} catch (Exception e) {
			e.printStackTrace();
			assertTrue(false);
		}
	}

}
