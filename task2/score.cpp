#include <vector>
#include <iostream>
#include <algorithm>
#include <iomanip>
using namespace std;
int main()
{
    int n;
    cout<<"Please push scores:"<<endl;
    cin>>n;
    vector<double> scores;
    for (int i=0;i<n;i++)
    {
        double s;
        cin>>s;
        scores.push_back(s);

    }
\
    double sum=0;
    for (int i=0;i<scores.size();i++)
    {
        cout<<"The"<<i+1<<"score is"<<scores[i]<< endl;
        sum+=scores[i];

    }

    sort(scores.begin(), scores.end(), greater<double>());
    cout<<"The max_score is"<<scores[0]<<endl;
    cout<<"The average of these scores is"<<sum/scores.size()<<endl;
    cout<<"The min_score is"<<scores[scores.size()-1]<<endl;
    for (int i=0;i<scores.size();i++)
    {
        cout<<scores[i]<<"";

    }
    cout<<endl;

    return 0;
}