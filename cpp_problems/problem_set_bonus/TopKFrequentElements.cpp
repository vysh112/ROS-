bool compare(pair<int, int> p1, pair<int, int> p2)
{if (p1.second == p2.second){
return (p1.first > p2.first);
}
return (p1.second > p2.second);
}
class Solution {
public:
    vector<int> topKFrequent(vector<int>& nums, int k) {
        vector<int> result;
        unordered_map<int,int> mp;
        for(int i=0;i<nums.size();i++){
            mp[nums[i]]++;
        }

        vector<pair<int,int>> frequency(mp.begin(),mp.end());

        sort(frequency.begin(),frequency.end(),compare);

        for(int i=0;i<k;i++){
            result.push_back(frequency[i].first);
        }
return result;}};