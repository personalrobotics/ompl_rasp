#ifndef EXPERIMENT_INFORMATION_H_
#define EXPERIMENT_INFORMATION_H_

#include <ctime>
#include <vector>

template <class Vertex, class Edge>
class ExperimentInformation
{
public:
    ExperimentInformation() {}

    size_t start() {
        clock_t startTime = clock();
        startTimes.push_back(startTime);
        endTimes.push_back(startTime);
        return startTimes.size() - 1;
    }
    void stop(size_t i)  {endTimes[i] = clock();}
    double runtime(size_t i) { return (double) (endTimes[i] - startTimes[i]) / CLOCKS_PER_SEC; }
    double runtime() { return runtime(0); }

    std::vector<clock_t> startTimes;
    std::vector<clock_t> endTimes;
};

#endif // EXPERIMENT_INFORMATION_H_
