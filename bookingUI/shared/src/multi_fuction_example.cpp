#include <iostream>
using namespace std;

double F(double (*f)(), int N,double);
double f(double,double);

double F(double (*f)(double,double), int N,double y)
{
    double sum = 0;
    for(int i = 1; i <= N; i++)
        sum += (*f)(1.0*i,y);
    return sum;
}

double f(double x,double y)
{
    return x*y;
}


int main(int argc,char* argv)
{
    cout << F(f,4,6);
    return 0;
}
