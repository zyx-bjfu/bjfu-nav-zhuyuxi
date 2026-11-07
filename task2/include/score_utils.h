#ifndef SCORE_UTILS_H
#define SCORE_UTILS_H

#include <vector>
#include <string>

// 函数声明
// 从文件读分数，返回分数列表
std::vector<double> read_scores_from_file(const std::string& filename);

// 找最大值
double find_max(const std::vector<double>& scores);

// 找最小值
double find_min(const std::vector<double>& scores);

// 算平均值
double find_avg(const std::vector<double>& scores);

// 从高到低排序
void sort_scores(std::vector<double>& scores);
#endif
