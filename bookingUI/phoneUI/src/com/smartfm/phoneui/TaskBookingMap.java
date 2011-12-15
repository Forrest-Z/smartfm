package com.smartfm.phoneui;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MapView;
import com.google.android.maps.Overlay;

import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Point;
import android.graphics.RectF;
import android.os.Bundle;

public class TaskBookingMap extends MapActivity {

	MapController mc;
	MapView mapView;
	StationList stations;
	Bitmap pin;
	
	class MapLocationOverlay extends Overlay {
		
		@Override
		public void draw(Canvas canvas, MapView mapView, boolean shadow) {
			super.draw(canvas, mapView, shadow);
			for( Station s: stations.getStations() ) {
				Point p = mapView.getProjection().toPixels(s.latlon, null);
				canvas.drawBitmap(pin, p.x - pin.getWidth() / 2,
						p.y - pin.getHeight(), null);
			}
		}

		@Override
		public boolean onTap(GeoPoint hit, MapView mapView) {	
			Point p_hit = mapView.getProjection().toPixels(hit, null);
			
			for( Station s: stations.getStations() ) {
				
				Point p_s = mapView.getProjection().toPixels(s.latlon, null);
				RectF hitTestRect = new RectF(p_s.x-pin.getWidth() / 2, 
						p_s.y-pin.getHeight(), p_s.y+pin.getWidth() / 2, p_s.y);
				
				if ( hitTestRect.contains(p_hit.x, p_hit.y) ) {
					setResult(RESULT_OK, new Intent().
							putExtra("com.smartfm.phoneui.stationName", s.name));
					finish();
					return true;
				}
			}
			return super.onTap(hit, mapView);
		}
	}

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle bundle) {
		super.onCreate(bundle);
		setContentView(R.layout.bookingmap);
		pin = BitmapFactory.decodeResource(getResources(), R.drawable.pin);
		
		stations = new StationList();

		mapView = (MapView) findViewById(R.id.mapView);
		mapView.setBuiltInZoomControls(true);
		mc = mapView.getController();
		mc.animateTo(new GeoPoint(1299631, 103771007));
		mc.setZoom(18);
		
		MapLocationOverlay mapLocationOverlay = new MapLocationOverlay();
		mapView.getOverlays().clear();
		mapView.getOverlays().add(mapLocationOverlay);
		mapView.invalidate();
	}

	@Override
	protected boolean isRouteDisplayed() {
		return false;
	}
}