#include <iostream> 

using namespace std;

class Echo 
{ 
public:
    int sec_a_func(int value)
    {
        cout<< "Sec A function called with value: "<<value<<endl;
    }

    int sec_b_func(int value)
    {
        cout<< "Sec B function called with value: "<<value<<endl;
    }

    void first_func(int (*f)(int), int value)
    {
        (*f)(value*5);
    }
    Echo(int arg)
    {
        cout<<"Int received"<<endl;
        first_func(sec_a_func, 1);
    }
    
    Echo(double arg)
    {
        cout<<"Double received"<<endl;
        first_func(sec_b_func, 2);
    }
}; 



int main() 
{ 
    int i_no = 1;
    double d_no=2.0;
    Echo echo(i_no);
    Echo echo2(d_no);
    return 0;
}  
