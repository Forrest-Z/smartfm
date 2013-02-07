#include <mysql++/mysql++.h>
#include <iostream>
#include <iomanip>
#include <math.h>

using namespace std;

struct ScoreData
{
  int id, node_src, node_dst, time_taken;
  double score, score_ver, final_score, x, y, t;
};

class MySQLHelper
{
public:
	mysqlpp::Connection conn_;
	string table_name_;
  int table_size_;
	MySQLHelper(string database, string name): table_name_(name), conn_(false), table_size_(-1)
	{
		connect_db(database);
	}

	void createTable()
	{
		if(table_size_!=-1)
		{
			cout<<"Table found, not going to create score table"<<endl;
			return;
		}

		//conn_.query("drop table "+name).execute();

		//reminder: mysql has a hard limit on 4096 columns
		stringstream createtb_ss;
		createtb_ss << "create table "<<table_name_ <<" (id int not null AUTO_INCREMENT";
    createtb_ss << ", node_src int not null, node_dst int not null, score double, x double,"; 
    createtb_ss << " y double, t double, time int, PRIMARY Key(id), INDEX (id), UNIQUE (id))";
		mysqlpp::Query createtb_query = conn_.query(createtb_ss.str());
		mysqlpp::SimpleResult createtb_res = createtb_query.execute();
    cout<<createtb_ss.str()<<endl;
		if(createtb_res) cout<<"Table create successful"<<endl;
		else cout<<"Table create failed: "<<createtb_query.error()<<endl;

		//cout<<"Query used: "<<createtb_ss.str()<<endl;
	}

	void insertData(ScoreData &data)
	{
      stringstream insertdata_ss;
  		insertdata_ss<<"insert into "<<table_name_<<" (node_src, node_dst, score, x, y, t, time) values (";
      insertdata_ss<<data.node_src<<", ";
      insertdata_ss<<data.node_dst<<", ";
      insertdata_ss<<data.score<<", ";
      insertdata_ss<<data.x<<", ";
      insertdata_ss<<data.y<<", ";
      insertdata_ss<<data.t<<", ";
      insertdata_ss<<data.time_taken<<")";
      mysqlpp::Query insertdata_query = conn_.query(insertdata_ss.str());
      mysqlpp::SimpleResult insert_res = insertdata_query.execute();
		  if(insert_res) cout<<"Data "<<data.node_src<<" "<<data.node_dst<<" "<<data.score<<" "<<data.time_taken<<" inserted successful \xd"<<flush;
		  else cout<<"Data insert failed: "<<insertdata_query.error()<<endl;
	}

	void display_data(mysqlpp::Query query)
	{
		if (mysqlpp::StoreQueryResult res = query.store()) {
			cout << "We have:" << endl;
			for (size_t i = 0; i < res.num_rows(); ++i) {
				for(size_t j=0; j<res.num_fields(); j++)
					cout << res[i][j] <<" ";
			}
			cout<<endl;
		}
		else {
			cerr << "Failed to get item list: " << query.error() << endl;
		}
	}

  //write retrieve function
  vector<ScoreData> getListScoreConstraint(double score_lower, double score_upper, int seperation)
  {
    stringstream ss;
    ss<<"select * from "<<table_name_<<" where "<<table_name_<<".node_dst-"<<table_name_<<".node_src>"<<seperation;
    ss<<" and "<<table_name_<<".final_score between "<<score_lower<<" and "<<score_upper;
    mysqlpp::Query query = conn_.query(ss.str());
    vector<ScoreData> sds;
    if(mysqlpp::StoreQueryResult res = query.store())
    {
      cout<<"There are "<<res.num_rows()<<" with score between " <<score_lower<<" and "<<score_upper<<endl;
      for(int i=0; i<res.num_rows(); i++)
      {
        ScoreData sd;
        sd.id = res[i][0];
        sd.node_src = res[i][1];
        sd.node_dst = res[i][2];
        sd.score = res[i][3];
        sd.x = res[i][4];
        sd.y = res[i][5];
        sd.t = res[i][6];
        sd.time_taken = res[i][7];
        sd.score_ver = res[i][8];
        sd.final_score = res[i][9];
        sds.push_back(sd);
      }
    }
    return sds;
  }
  
  bool updateScoreVer(double score_ver, ScoreData data)
  {
    stringstream ss;
    ss<<"update frontEndData_1358896312661754646 set score_ver="<<score_ver<<", final_score="<<sqrt(score_ver*data.score)<<" where "<<table_name_<<".id="<<data.id;
    mysqlpp::Query updatedata_query = conn_.query(ss.str());
    mysqlpp::SimpleResult insert_res = updatedata_query.execute();
		if(insert_res) cout<<"Data "<<data.node_src<<" "<<data.node_dst<<" "<<data.score<<" "<<data.time_taken<<" updated successful \xd"<<flush;
		else cout<<"Update score failed: "<<updatedata_query.error()<<endl;
    return true;
  }

	bool getData(ScoreData &sd)
	{
		stringstream ss;
		double score = -1;
		ss<<"select * from "<<table_name_<<" where "<<table_name_<<".node_src="<<sd.node_src<<" and ";
    ss<<table_name_<<".node_dst="<<sd.node_dst;
		//cout<<ss.str()<<endl;
		mysqlpp::Query query = conn_.query(ss.str());
		if(mysqlpp::StoreQueryResult res = query.store())
		{
      assert(res.num_rows() <= 1);
      if(res.num_rows() == 0) return false;
      sd.id = res[0][0];
      sd.node_src = res[0][1];
      sd.node_dst = res[0][2];			
      sd.score = res[0][3];
      sd.x = res[0][4];
      sd.y = res[0][5];
      sd.t = res[0][6];
      sd.time_taken = res[0][7];
      sd.score_ver = res[0][8];
   //   sd.final_score = res[0][9];
      return true;
		}
		else
		{
			return false;
		}
	}
	

private:
	void connect_db(string database)
	{
		// Get database access parameters from command line
		const char* db = database.c_str(), *server = "localhost", *user = "root", *pass = "nus";

		// Connect to the sample database.

		if (conn_.connect(db, server, user, pass)) {
			cout <<"DB connected"<<endl;
		}
		else {
			cerr << "DB connection failed: " << conn_.error() << endl;
			exit(1);
		}

		mysqlpp::Query query = conn_.query("select * from "+table_name_+"_0");

		if(mysqlpp::StoreQueryResult res = query.store())
		{
			cout << "Database "<<database<<" have "<<res.num_fields()<<"x"<<res.num_rows()<<endl;

			if (res.num_fields() > 0)
				table_size_ = res.num_rows();
		}
	}
};
