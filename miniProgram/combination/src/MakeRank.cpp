//
// Created by 97412 on 2018/11/4.
//

#include "../include/MakeRank.h"

/*
 * @ 从逆序列构建排列
 */
vector<int> makeRankFromInversions(vector<int> inversions){
    vector<int> ranks(inversions.size(),-1);
    for (int i = 0; i < inversions.size(); ++i) {
        int cnt=0;
        for (int j = 0; j < ranks.size(); ++j) {
            if(-1 == ranks[j]){
                cnt++;
            }
            if(cnt == inversions[i]+1){
                ranks[j] = i;
                break;
            }
        }
    }
    return ranks;
}

/*
 * @ 生成逆序列
 */
vector<int> makeInversion(vector<int> ranks){
    vector<int> inversions(ranks.size(),-1);
    for (int i = 0; i < ranks.size(); ++i) {
        int cnt=0;
        for (int j = 0; j < i; ++j) {
            if(ranks[j] > ranks[i]){
                cnt++;
            }
        }
        inversions[ranks[i]]=cnt;
    }
    return inversions;
}

/*
 * 1.生成排列索引
 * 2.执行排列
 *  2.1 求出最大可移动数
 *  2.2 交换箭头所指向的相邻的数
 *  2.3 交换p > m的数的箭头方向
 */

vector<RankNode> makeRankNode(int N){
    vector<RankNode> rankNodes;
    for (int i = 0; i < N; ++i) {
        RankNode rankNode;
        rankNode.direction=LEFT;
        rankNode.index=i;
        rankNodes.push_back(rankNode);
    }
    return rankNodes;
}

int getMaxMove(vector<RankNode> rankNodes,int& maxValue){
    int maxIndex = -1;
    maxValue = 0;
    for (int i = 0; i < rankNodes.size(); ++i) {
        if(0 != i && LEFT == rankNodes[i].direction && rankNodes[i].index > rankNodes[i-1].index){
            maxIndex = rankNodes[i].index > maxValue ? i : maxIndex;
            maxValue = rankNodes[i].index > maxValue ? rankNodes[i].index : maxValue;
        }

        if(rankNodes.size() - 1 != i && RIGHT == rankNodes[i].direction && rankNodes[i].index > rankNodes[i+1].index){
            maxIndex = rankNodes[i].index > maxValue ? i : maxIndex;
            maxValue = rankNodes[i].index > maxValue ? rankNodes[i].index : maxValue;
        }
    }
    return maxIndex;
}

void switchLocation(vector<RankNode>& rankNodes,int maxIndex){
    if(LEFT == rankNodes[maxIndex].direction){
        RankNode rankNode = rankNodes[maxIndex];
        rankNodes[maxIndex] = rankNodes[maxIndex-1];
        rankNodes[maxIndex-1] = rankNode;
    }
    else if(RIGHT == rankNodes[maxIndex].direction){
        RankNode rankNode = rankNodes[maxIndex];
        rankNodes[maxIndex] = rankNodes[maxIndex+1];
        rankNodes[maxIndex+1] = rankNode;
    }
}

void switchDirection(vector<RankNode>& rankNodes,int maxValue){
    for (int i = 0; i < rankNodes.size(); ++i) {
        if(rankNodes[i].index > maxValue){
            rankNodes[i].direction = rankNodes[i].direction == LEFT ? RIGHT : LEFT;
        }
    }
}

vector<vector<RankNode>> makeRank(vector<RankNode> rankNodes){
    vector<vector<RankNode>> ranks;
    while (true){
        ranks.push_back(rankNodes);
        int maxValue = 0;
        int maxIndex = getMaxMove(rankNodes,maxValue);
        if(-1 == maxIndex) break;
        switchLocation(rankNodes,maxIndex);
        switchDirection(rankNodes,maxValue);
    }
    return ranks;
}

vector<vector<RankNode>> makeRank(int N){
    return makeRank(makeRankNode(N));
}
