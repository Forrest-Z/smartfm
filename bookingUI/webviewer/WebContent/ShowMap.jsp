<!DOCTYPE html "-//W3C//DTD XHTML 1.0 Strict//EN" 
  "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">
<html xmlns="http://www.w3.org/1999/xhtml">
  <head>
    <meta http-equiv="content-type" content="text/html; charset=utf-8"/>
    <title>Car on GoogleMap</title>
    <script src="http://maps.google.com/maps?file=api&amp;v=2&amp;key=ABQIAAAAu_goHPT-XC9AR_jpbSenbBSOj79YWDVmA-W-lPv1PjVCp6Bk_BS7DAXPZZEmjTNe0ruulP7GqNLT_Q&sensor=true"
            type="text/javascript"></script>
    <script type="text/javascript">

    function initialize() {
      if (GBrowserIsCompatible()) {
        var map = new GMap2(document.getElementById("map_canvas"));
        map.setMapType(G_SATELLITE_MAP);
        map.setCenter(new GLatLng("1.29947106","103.77083659"), 18);
        
        var lat = <%= Double.parseDouble((String)request.getAttribute("latitude")) %>;
        var lng = <%= Double.parseDouble((String)request.getAttribute("longitude")) %>;
        var ID = <%= Integer.parseInt((String)request.getAttribute("ID")) %>;
        var point = new GLatLng(lat, lng);
        
        var carIcon = new GIcon();
        carIcon.image = "Images/carIcon.png";
        carIcon.iconSize = new GSize(60, 60);
        carIcon.iconAnchor = new GPoint(30, 30);
        carIcon.infoWindowAnchor = new GPoint(30, 30);
        var marker = new GMarker(point, carIcon);
        var text = "Car " + ID;
        GEvent.addListener(marker, "mouseover", function() {
        	marker.openInfoWindowHtml(text);
        });
        GEvent.addListener(marker, "mouseout", function(){
        	marker.closeInfoWindow();
        });
        
        map.addOverlay(marker);
        
        map.setUIToDefault();
      }
    }

    </script>
  </head>
  <body onload="initialize()" onunload="GUnload()">
    <div id="map_canvas" style="width:100%; height:100%"></div>
  </body>
</html>