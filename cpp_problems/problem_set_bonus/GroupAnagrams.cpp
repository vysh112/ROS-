class Solution {
public:
    vector<vector<string>> groupAnagrams(vector<string>& strs) {
        vector<vector<string>> result;
        unordered_map<string,vector<string>> unorderedmap;
        string word;
        int i;
        int n= strs.size();
        for(i=0;i<n;i++){
            word = strs[i];
            sort(strs[i].begin(),strs[i].end());
            unorderedmap[strs[i]].push_back(word);
        }
        for(auto itr=unorderedmap.begin();itr!=unorderedmap.end();++itr)
            result.push_back(itr->second);
        
        return result;

        
        
    }
};