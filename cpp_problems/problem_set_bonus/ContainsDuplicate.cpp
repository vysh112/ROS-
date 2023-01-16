class Solution {
public:
    bool containsDuplicate(vector<int>& nums) {

        int i,j;
        std::sort(nums.begin(),nums.end());
        int temp;

        for( i=0;i<nums.size();i++){

            if(i>0){
            if(nums[i]==temp){
                return true;
            }}
            temp=nums[i];
        }
        return false;
    }
};
