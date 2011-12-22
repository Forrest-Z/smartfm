package com.smartfm.phoneui.test;

import java.util.ArrayList;

import com.smartfm.phoneui.*;

import android.test.AndroidTestCase;

public class StationListTest extends AndroidTestCase {

	protected StationList stations;
	protected ArrayList<String> names;
	
	protected void setUp() {
		stations = new StationList();
		names = stations.allNames();
	}
	
	public void testExist() {
		assertTrue( stations.exists(names.get(0)) );
		assertFalse( stations.exists("dummy") );
	}
}
