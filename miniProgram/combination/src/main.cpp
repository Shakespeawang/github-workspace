#include <iostream>
#include "../include/MakeRank.h"

using namespace std;

int main() {

    int N;
    string str;
    while(cin>>N){
        vector<int>vec(N);
        for (int k = 0; k < N; ++k) {
            cin>>vec[k];
            vec[k]--;
        }
        vector<int>inversions = makeInversion(vec);
        for (int i = 0; i < inversions.size(); ++i) {
            cout << inversions[i] << ",";
        }
        cout << endl;

        vector<int>ranks = makeRankFromInversions(inversions);
        for (int i = 0; i < inversions.size(); ++i) {
            cout << ranks[i]+1 << ",";
        }
        cout << endl;
    }
    return 0;

}