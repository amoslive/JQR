

#ifndef LIHEBIKE_TRACK_STAND_ILQRMPC_20221122__UTILS_H_
#define LIHEBIKE_TRACK_STAND_ILQRMPC_20221122__UTILS_H_

#include <cmath>
#include <ctime>
#include <chrono>
#include <iostream>

class TicToc {
 public:
  TicToc() {
    tic();
  }

  void tic() {
    start = std::chrono::steady_clock::now();
  }

  double toc() {
    end = std::chrono::steady_clock::now();
//    std::chrono::duration<double> elapsed_seconds = end - start;
    double elapsed_seconds = (end-start).count()/1e9;
    std::cout << "elapsed time: " << elapsed_seconds << " s." << std::endl;
    return elapsed_seconds;
  }

 private:
  std::chrono::time_point<std::chrono::steady_clock> start, end;
};

class CyclicCounter {
 public:
  CyclicCounter(const int & _period = 1) {
    setPeriod(_period);
    init();
  }
  void init() {
    cnt_abs = 0;
    cnt_rel = 0;
    cnt_cycle = 0;
    is_new_cycle = true;
  }
  void incrementPost() {
    ++cnt_abs;
    if (cnt_rel == period1) {
      cnt_rel = 0;
      is_new_cycle = true;
      ++cnt_cycle;
    } else {
      ++cnt_rel;
      is_new_cycle = false;
    }
  }
  void setPeriod(const int &_period) {
    if(_period < 1)
      throw std::invalid_argument("period must be greater than zero.");
    this->period = _period;
    this->period1 = _period-1;
  }
  int getPeriod() const { return period; }
  int getCntAbs() const { return cnt_abs; }
  int getCntRel() const { return cnt_rel; }
  int getCntCycle() const { return cnt_cycle; }
  bool isNewCycle() const { return is_new_cycle; }
 private:
  // parameters
  int period = 1; 
  int period1 = 0;
  int cnt_abs = 0; // ，
  int cnt_rel = 0; // ，
  int cnt_cycle = 0; // update
  bool is_new_cycle = true; // 
};

#endif 
