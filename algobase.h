#if !defined(__ALGO_BASE__)
#define __ALGO_BASE__

class AlgoBase
{
public:
    virtual ~AlgoBase() {};
    virtual void Configure() = 0;
};



#endif //(__ALGO_BASE__)
