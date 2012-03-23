drop table if exists infrastructure;
create table infrastructure (
    id varchar(30) not null,
    status int not null default false,
    primary key(id)
);