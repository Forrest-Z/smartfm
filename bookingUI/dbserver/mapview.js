

var car_icon = new google.maps.MarkerImage('http://labs.google.com/ridefinder/images/mm_20_purple.png');
car_icon.shadow = 'http://labs.google.com/ridefinder/images/mm_20_shadow.png';
car_icon.iconSize = new google.maps.Size(12, 20);
car_icon.shadowSize = new google.maps.Size(22, 20);
car_icon.iconAnchor = new google.maps.Point(6, 20);
car_icon.infoWindowAnchor = new google.maps.Point(5, 1);

var station_icon = new google.maps.MarkerImage('http://labs.google.com/ridefinder/images/mm_20_green.png');
station_icon.shadow = 'http://labs.google.com/ridefinder/images/mm_20_shadow.png';
station_icon.iconSize = new google.maps.Size(12, 20);
station_icon.shadowSize = new google.maps.Size(22, 20);
station_icon.iconAnchor = new google.maps.Point(6, 20);
station_icon.infoWindowAnchor = new google.maps.Point(5, 1);

var vehicleMarkers = {};

var map;



function initialize() {
    var myOptions = {
        zoom: 18,
        center: new google.maps.LatLng(1.2995, 103.77),
        mapTypeId: google.maps.MapTypeId.HYBRID
    };
    map = new google.maps.Map(document.getElementById("map_canvas"), myOptions);

    showStations();
    showCars();
}

function showStations() {
    downloadUrl("list_stations.php", function(xml) {
        var stationNodes = xml.documentElement.getElementsByTagName("station");
        for (var i = 0; i < stationNodes.length; i++) {
            var name = stationNodes[i].getAttribute("name");
            var point = new google.maps.LatLng(
                parseFloat(stationNodes[i].getAttribute("latitude")),
                parseFloat(stationNodes[i].getAttribute("longitude")) );
            var marker = new google.maps.Marker({position: point, map: map, title: name, icon: station_icon});
        }
    });
}

function showCars() {
    downloadUrl("list_vehicles.php", function(xml) {
        var vehicleNodes = xml.documentElement.getElementsByTagName("vehicle");
        var tmp = {};
        for (var i = 0; i < vehicleNodes.length; i++) {
            var vehicleID = vehicleNodes[i].getAttribute("vehicleID");
            var lat = vehicleNodes[i].getAttribute("latitude");
            var lng = vehicleNodes[i].getAttribute("longitude");
            if( lat=="" || lng=="" )
                continue;
            var point = new google.maps.LatLng(parseFloat(lat), parseFloat(lng));
            var tooltip = vehicleNodes[i].getAttribute("tooltip");
            var html = vehicleID + ": " + tooltip;

            if( vehicleID in vehicleMarkers ) {
                vehicleMarkers[vehicleID].setPosition(point);
                vehicleMarkers[vehicleID].setTitle(html);
            }
            else {
                var marker = new google.maps.Marker({position: point, map: map, title: html, icon: car_icon});
                vehicleMarkers[vehicleID] = marker;
            }

            tmp[vehicleID] = vehicleMarkers[vehicleID];
        }

        var tmp2 = {};
        for( var key in vehicleMarkers ) {
            if( key in tmp )
                tmp2[key] = vehicleMarkers[key];
            else
                vehicleMarkers[key].setVisible(false);
        }
        vehicleMarkers = tmp2;
    });

    setTimeout("showCars()", 2000);
}