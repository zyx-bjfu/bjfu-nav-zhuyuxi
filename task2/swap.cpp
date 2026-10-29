#include <iostream>
#include <stdio.h>
using namespace std;

void swap_by_value(int a,int b)
{
    int temp=a;
    a=b;
    b=temp;
}

void swap_by_pointer(int* a,int* b)
{
    int t=*a;
    *a=*b;
    *b=t;
}

void swap_by_ref(int& a,int& b)
{
    int t=a;
    a=b;
    b=t;

}

int main()
{
    int x=3,y=7;
    cout<<"Before exchange:"<<x<<" "<<y<<endl;
    swap_by_value(x,y);
    cout<<"After exchange value:"<<x<<" "<<y<<endl;
    swap_by_pointer(&x,&y);
    cout<<"Afer exchange pointer"<<x<<" "<<y<<endl;
    swap_by_ref(x,y);
    cout<<"Afer exchange ref:"<<x<<" "<<y<<endl;
    return 0;
}