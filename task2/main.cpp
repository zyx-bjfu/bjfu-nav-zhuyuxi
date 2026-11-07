#include <iostream>
#include "include/score_utils.h"


int main() {
    // 从文件读分数
    auto scores = read_scores_from_file("scores.txt");
    
    // 输出
    std::cout << "最高分: " << find_max(scores) << std::endl;
    std::cout << "最低分: " << find_min(scores) << std::endl;
    std::cout << "平均分: " << find_avg(scores) << std::endl;
    
    // 排序输出
    sort_scores(scores);
    std::cout << "从高到低: ";
    for (int i=0; i<scores.size(); i++) {
        std::cout << scores[i] << " ";
    }
    std::cout << std::endl;
    
    return 0;
}

