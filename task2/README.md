# task2 Hello C++

## 编译运行方法

```bash
g++ hello.cpp -o hello && ./hello
g++ score.cpp -o score && ./score
g++ swap.cpp -o swap && ./swap
```
为什么传值交换不了？
传值时函数拿到的是变量的副本，函数里改的是副本，原件不动。传指针和传引用直接操作原件，所以能交换。

## CMake 多文件工程

```bash
cd task2
mkdir -p build
cd build
cmake ..
make
./score_app
