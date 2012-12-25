#include <mysql++/mysql++.h>
#include <iostream>
#include <iomanip>

using namespace std;

class MySQLHelper
{
public:
	int skip_reading_;
	mysqlpp::Connection conn_;
	string table_name_;
	int table_size_;
	const int max_fields_per_table_;
	MySQLHelper(int skip_reading, string database, string name): skip_reading_(skip_reading), conn_(false), table_size_(-1), max_fields_per_table_(200)
	{
		size_t filename_length = name.find(".");
		if( filename_length != name.length())
			name.resize(filename_length);
		stringstream table_name;
		table_name<<name<<"_skip"<<skip_reading;
		table_name_ = table_name.str();
		connect_db(database);
	}


	void create_2dtable(int size)
	{
		if(table_size_!=-1)
		{
			cout<<"Table found, not going to create one"<<endl;
			return;
		}


		string name = table_name_;
		//conn_.query("drop table "+name).execute();

		//mysql has a hard limit on 4096 columns, got to get away around it
		//practically, it cannot contain more than 400 in this case

		table_size_  = (int)(size/skip_reading_) +1;
		int table_count = 0;
		int fields_now = 0;
		while(fields_now < table_size_)
		{

			stringstream createtb_ss;
			createtb_ss << "create table "<<name <<"_"<<table_count<<" (id int not null";
			int subtable_total = max_fields_per_table_;
			if(table_size_ - fields_now < max_fields_per_table_) subtable_total = table_size_ - fields_now;
			for(int i=0; i<subtable_total; i++)
				createtb_ss << ", node_"<<(i+fields_now)*skip_reading_<<" DOUBLE default -1";
			fields_now += subtable_total;
			cout<<"fields_now: "<<fields_now<<endl;
			createtb_ss<< ", PRIMARY KEY (id), INDEX (id), UNIQUE (id))";
			mysqlpp::Query createtb_query = conn_.query(createtb_ss.str());
			mysqlpp::SimpleResult createtb_res = createtb_query.execute();
			if(createtb_res) cout<<"Table create successful"<<endl;
			else cout<<"Table create failed: "<<createtb_query.error()<<endl;

			//cout<<"Query used: "<<createtb_ss.str()<<endl;

			for(int i=0; i<table_size_; i++)
			{
				stringstream data;
				data<<"insert "<<name<<"_"<<table_count<<" set "; //field1=value_1, field2=value_2;
				data <<"id="<<i*skip_reading_;
				conn_.query(data.str()).execute();
			}
			table_count++;
		}
		cout<<"Table "<<name<<" with fields "<<table_size_<<" created in mysql."<<endl;
	}

	void update_data(vector<double> data, int node_id)
	{


		if(data.size() != (size_t)table_size_)
		{
			cout<<"data.size() == expected_size failed! Given "<<data.size()<<" "<<table_size_<<endl;
			exit(0);
		}
		int table_count = 0;
		int fields_now = 0;
		while(fields_now < table_size_)
		{
			stringstream update_ss;
			update_ss<<"update "<<table_name_<<"_"<<table_count<<" set ";
			int subtable_total = max_fields_per_table_;
			if(table_size_ - fields_now < max_fields_per_table_) subtable_total = table_size_ - fields_now;
			for(int j=0; j<subtable_total; j++)
			{
				string first_word = ", node_";
				if(j==0) first_word = "node_";
				update_ss<<first_word<<(j+fields_now)*skip_reading_<<"="<<data[j+fields_now];
			}
			update_ss<< " where "<<table_name_<<"_"<<table_count<<".id ="<<node_id;
			mysqlpp::Query update_query = conn_.query(update_ss.str());
			if(!update_query.execute())
				cout<<"Update data failed at fields "<<fields_now<<": "<<update_query.error()<<endl;

			fields_now += subtable_total;
			table_count++;
		}
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

	double retrieve_first_score(int row)
	{
		stringstream ss;
		double score = -1;
		ss<<"select node_0 from "<<table_name_<<"_0 where "<<table_name_<<"_0.id="<<row;
		//cout<<ss.str()<<endl;
		mysqlpp::Query query = conn_.query(ss.str());
		if(mysqlpp::StoreQueryResult res = query.store())
		{
			score = res[0][0];
		}
		else
		{
			cout << "Failed to get item list: " << query.error() << endl;
		}
		return score;
	}
	vector<vector<double> > retrieve_score()
	{

		vector<vector<vector<double> > > scores;
		int table_count = int(table_size_/max_fields_per_table_)+1;
		for(int i=0; i<table_count; i++)
		{
			stringstream retrieve_query;
			retrieve_query<<"select * from "<<table_name_<<"_"<<i;
			mysqlpp::Query query = conn_.query(retrieve_query.str());
			vector<vector<double> > score_table;
			if(mysqlpp::StoreQueryResult res = query.store())
			{
				cout << "We have "<<res.num_fields()<<"x"<<res.num_rows()<<endl;

				for(size_t i=0; i<res.num_rows(); i++)
				{
					vector<double> score_row;
					for(size_t j=1; j<res.num_fields(); j++)
						score_row.push_back(res[i][j]);
					score_table.push_back(score_row);
				}
			}
			else
			{
				cerr << "Failed to get item list: " << query.error() << endl;
			}
			scores.push_back(score_table);
		}
		vector<vector<double> > reorg_scores;

		for(size_t j=0; j<scores[0].size(); j++)
		{
			vector<double> score_row;
			for(size_t i=0; i<scores.size(); i++)
			{
				for(size_t k=0; k<scores[i][j].size(); k++)
				{

					score_row.push_back(scores[i][j][k]);
				}
			}
			reorg_scores.push_back(score_row);
		}

		return reorg_scores;
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
