class Solution {
public:
    vector<int> productExceptSelf(vector<int>& nums){
    vector<int> answer(nums.size(),1);
    vector<int> numleft(nums.size(),1);
    vector<int> numright(nums.size(),1);
    for(int i=1;i<nums.size();i++){
        numleft[i]=numleft[i-1]*nums[i-1];
    }
    for(int i=nums.size()-1;i>=1;i--){
        numright[i-1]=numright[i]*nums[i];
    }    
    for(int i=0;i<nums.size();i++){
        answer[i]=numleft[i]*numright[i];
    }    
    return answer;
    }
};