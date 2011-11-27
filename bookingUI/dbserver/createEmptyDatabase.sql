drop table if exists stations;
create table stations (
    name char(50) not null,
    latitude float(10,6) not null,
    longitude float(10,6) not null
);

insert into stations values ('E3A', 1.30073, 103.77166);
insert into stations values ('E3', 1.300730, 103.770844);
insert into stations values ('DCC Workshop', 1.299121, 103.770788);
insert into stations values ('McDonald', 1.298116, 103.771209);

drop table if exists vehicles;
create table vehicles (
    vehicleID char(10) not null,
    status enum('WaitingForAMission', 'GoingToPickupLocation', 'GoingToDropOffLocation', 'AtPickupLocation', 'NotAvailable') not null,
    latitude float(10,6),
    longitude float(10,6),
    eta int(6),
    requestID int
);

drop table if exists requests;
create table requests (
    requestID int not null auto_increment,
    customerID char(10) not null,
    status enum('Requested', 'Acknowledged', 'Confirmed', 'Processing', 'Completed', 'Cancelled') not null,
    pickUpLocation char(50) not null,
    dropOffLocation char(50) not null,
    vehicleID char(10),
    primary key(requestID)
);