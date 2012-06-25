#include <iostream>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <vector>
#include <inttypes.h>
#include <math.h>
#include <sstream>
using namespace std;

uint64_t Duoas_dec2bin(uint64_t n, string& bin){
	uint64_t number = n;
	char result[(sizeof(uint64_t)*8)+1];
	uint64_t index=sizeof(uint64_t)*8;
	result[index]='\0';
	uint64_t count_one=0;
	do
	{
		result[--index]='0'+(n&1);
		if(result[index] == '1') count_one++;
	}
	while (n>>=1);
	//if(count_one>15) cout<<number<<" has more than 15 segments"<<endl;
	bin = std::string(result+index);
	return count_one;
}

int main(){

	uint64_t segments = 19;
	int no_one = 0;
	vector<string> possibilities;
	cout << "Enter number of segments to be turn on: "<<endl;
		cin >> no_one;
		double possible_combinations = pow(2.0, segments);
		uint64_t pc_int64 = possible_combinations;
		cout << pc_int64 << " possible combinations"<<endl;
		int combination_after_thres = 0;
		int progress_report = pc_int64/10;
		int progress_now = 0;

		#pragma omp parallel for shared(combination_after_thres)
		for(uint64_t i=0; i<pc_int64; i++)
		{
			string bin;
			uint64_t count_one = Duoas_dec2bin(i, bin);
			if(count_one == no_one)
			{
				//cout<<i<<": "<<count_one<<endl;
				#pragma omp critical
				possibilities.push_back(bin);
				combination_after_thres ++;
			}//cout<<" B"<<endl;
		}
			//if(i%progress_report==0) cout<<progress_now++<<" ";

		cout << "\nPercentage reduction: "<< 100 - combination_after_thres/(double)pc_int64*100<<endl;
		cout << combination_after_thres << "/" << pc_int64<<endl;
		cout << "\nNumber of vector pushed: "<<possibilities.size()<<endl;

		ofstream myfile;
		stringstream ss;
		ss<<no_one<<".txt";
		myfile.open (ss.str().c_str());
		for( vector<string>::iterator it = possibilities.begin(); it!=possibilities.end(); it++)
		{
			string binary = *it;
			for(size_t i = binary.size(); i < segments; i++)
				binary = "0"+binary;
			myfile << binary <<"\n";
		}
		myfile.close();
			//if minimum is set to by the quarter, number obtain for 28 segments: 266752238/268435456 with 0.627048 savings. Insignificant!!!
			//however if it is set to by half, number obtain is 114159428/268435456 with 57.4723 savings. Quite Good!
			//Should shoot for having only 20 segments. Only 431910/1048576 combinations need to be evaluated

			//compile with omp reduced the running time by slightly more than half with 28 segments. From 1m02s to 26s.


	return 0;
}
