#include <iostream>
#include <iomanip>
#include <vector>

using namespace std;

void print(vector<int> flist)
{
   vector <int> :: iterator iter;
   for (iter = flist.begin(); iter != flist.end(); ++iter)
      cout<<*iter<<' ';
   cout<<endl;
}

int main()
{
   vector<int> nums;
   int n;
   nums.assign({1,2,3,4,5,6,7,8,9});

   cout<<"vector contents"<<endl;
   print(nums);
   for (int i = 0; i < 3; i++)
   {
      nums.push_back({i});
      n = nums.size();
      cout << nums[n-1] << endl;
   }

   cout<<"After push new element in the list"<<endl;
   print(nums);
   return 0;
}