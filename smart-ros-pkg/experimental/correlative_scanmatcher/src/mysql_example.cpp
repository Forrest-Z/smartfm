#include "mysql_helper.h"
#include "fmutil/fm_stopwatch.h"
int main(int argc, char *argv[])
{

	MySQLHelper sql(5, "scanmatch_result", argv[1]);

	sql.create_2dtable(2100);


	// Retrieve a subset of the sample stock table set up by resetdb
	// and display it.

	//sql.display_data(sql.conn_.query("select node_0 from utown"));
	//UPDATE table1 SET field1=new_value1 WHERE condition

	fmutil::Stopwatch sw("update data");
	for(int i=0; i<sql.table_size_*sql.skip_reading_; i+=sql.skip_reading_)
	{
		vector<double> data;
		for(int j=0; j<sql.table_size_*sql.skip_reading_; j+=sql.skip_reading_)
		{
			data.push_back(rand()%100000/1000.);
		}
		sql.update_data(data, i);
	}
	sw.end();

	vector<vector<double> > scores = sql.retrieve_score();
	for(size_t i=0; i<scores[0].size(); i++)
		cout<< scores[0][i]<<" ";
	cout<<endl;
	for(size_t i=0; i<scores[scores.size()-1].size(); i++)
		cout<< scores[scores.size()-1][i]<<" ";
	cout<<endl;

	//sql.display_data(sql.conn_.query("select node_0 from utown"));


return 0;
}
