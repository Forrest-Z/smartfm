#include "mysql_helper.h"
#include "fmutil/fm_stopwatch.h"
int main(int argc, char *argv[])
{

	MySQLHelper sql(1, "scanmatch_result", argv[1]);

	cout<<sql.retrieve_first_score(0)<<" "<<sql.retrieve_first_score(100)<<endl;
	//sql.create_2dtable(2100);
	vector<vector<double> > scores = sql.retrieve_score();

	int count = 0;
	for(size_t i=0; i<scores.size(); i++)
	{
		if(scores[i][0] != -1) count++;
	}
	cout<<"Percent complete "<<count<<"/"<<scores.size()<<"="<<count/(double)scores.size()<<endl;
	return 0;



	// Retrieve a subset of the sample stock table set up by resetdb
	// and display it.

	sql.display_data(sql.conn_.query("select node_0 from utown"));
	//UPDATE table1 SET field1=new_value1 WHERE condition

	fmutil::Stopwatch sw("update data");
	for(int i=0; i<sql.table_size_; i+=sql.skip_reading_)
	{
		vector<double> data;
		for(int j=0; j<sql.table_size_; j+=sql.skip_reading_)
		{
			data.push_back(rand()%100000/1000.);
		}
		sql.update_data(data, i);
	}
	sw.end();
	sql.display_data(sql.conn_.query("select node_0 from utown"));


return 0;
}
