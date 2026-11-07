#include "../include/score_utils.h"
#include <iostream>
#include <fstream>
#include <algorithm>

// 从文件读分数
std::vector<double> read_scores_from_file(const std::string& filename) {
    std::vector<double> scores;
    std::ifstream fin(filename);
    double s;
    while (fin >> s) {
        scores.push_back(s);
    }
    return scores;
}

// 找最大值
double find_max(const std::vector<double>& scores) {
    double max = scores[0];
    for (int i=1; i<scores.size(); i++) {
        if (scores[i] > max) max = scores[i];
    }
    return max;
}

// 找最小值
double find_min(const std::vector<double>& scores) {
    double min = scores[0];
    for (int i=1; i<scores.size(); i++) {
        if (scores[i] < min) min = scores[i];
    }
    return min;
}

// 算平均值
double find_avg(const std::vector<double>& scores) {
    double sum = 0;
    for (int i=0; i<scores.size(); i++) {
        sum += scores[i];
    }
    return sum / scores.size();
}

// 从高到低排序
void sort_scores(std::vector<double>& scores) {
    std::sort(scores.begin(), scores.end(), std::greater<double>());
}
