package com.smartfm.phoneui;


import android.app.TabActivity;
import android.content.Intent;
import android.content.res.Resources;
import android.os.Bundle;
import android.widget.TabHost;

public class MainActivity extends TabActivity {

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.main);
		setUpTabs();
	}

	private void setUpTabs() {
		Resources res = getResources(); // Resource object to get Drawables
		TabHost tabHost = getTabHost(); // The activity TabHost
		TabHost.TabSpec spec; // Resusable TabSpec for each tab
		Intent intent; // Reusable Intent for each tab

		intent = new Intent().setClass(this, TaskList.class);
		spec = tabHost.newTabSpec("info").setIndicator("Booking Information",
				res.getDrawable(R.drawable.ic_tab_info)).setContent(intent);
		tabHost.addTab(spec);

		// Do the same for the other tabs
		intent = new Intent().setClass(this, MapLocations.class);
		spec = tabHost.newTabSpec("mapview").setIndicator("Map View",
				res.getDrawable(R.drawable.ic_tab_mapview)).setContent(intent);
		tabHost.addTab(spec);

		intent = new Intent().setClass(this, LiveCamera.class);
		spec = tabHost.newTabSpec("camera").setIndicator("Car Camera",
				res.getDrawable(R.drawable.ic_tab_camera)).setContent(intent);
		tabHost.addTab(spec);

		tabHost.setCurrentTab(0);
	}
}