drop table if exists stations;
create table stations (
    name varchar(50) not null,
    latitude float(10,6) not null,
    longitude float(10,6) not null
);

insert into stations values ('E3A', 1.30073, 103.77166);
insert into stations values ('EA', 1.300730, 103.770844);
insert into stations values ('DCC Workshop', 1.299121, 103.770788);
insert into stations values ('McDonald', 1.298116, 103.771209);

drop table if exists vehicles;
create table vehicles (
    vehicleID varchar(20) not null,
    status enum('WaitingForAMission', 'GoingToPickupLocation', 'GoingToDropoffLocation', 'AtPickupLocation', 'NotAvailable') not null,
    latitude float(10,6) default null,
    longitude float(10,6) default null,
    eta int(6) default null, /*estimate time of arrival to current dest (pick or drop)*/
    requestID int default null, /*requestID of the request being currently processed*/
    currentLocation varchar(50) default null /*where the vehicle is (station name)*/
);

drop table if exists requests;
create table requests (
    requestID int not null auto_increment,
    customerID varchar(20) not null,
    status enum('Requested', 'Acknowledged', 'Confirmed', 'Processing', 'Completed', 'Cancelled') not null,
    pickUpLocation varchar(50) not null,
    dropOffLocation varchar(50) not null,
    vehicleID varchar(10) default null, /*id of the vehicle assigned to the request*/
    eta int(6) default null, /*estimated time of arrival (either at pickup location or dropoff location)*/
    custCancelled boolean default false, /*whether or not customer cancelled (only customer can modify)*/
    vehicleAcknowledgedCancel enum('No', 'Acknowledged', 'Rejected') not null default 'No',
    primary key(requestID)
);
