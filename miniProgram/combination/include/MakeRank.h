//
// Created by 97412 on 2018/11/4.
//

#ifndef MINIPROGRAM_MAKERANK_H
#define MINIPROGRAM_MAKERANK_H

#include <string>
#include <vector>

using namespace std;

vector<int> makeRankFromInversions(vector<int> inversions);

vector<int> makeInversion(vector<int> vec);


#define LEFT 1
#define RIGHT 2

typedef struct RankNode{
    int direction;
    int index;
};

vector<RankNode> makeRankNode(int N);

vector<vector<RankNode>> makeRank(vector<RankNode> rankNodes);

vector<vector<RankNode>> makeRank(int N);



#endif //MINIPROGRAM_MAKERANK_H
