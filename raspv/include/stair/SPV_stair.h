#include "SPV/SPV.h"
#include "stair/stair_classes.h"


class SPV_stair : public SPV
{
public:

  SPV_stair() : SPV()
  {}

  SPV_stair(ros::NodeHandle &nh) : SPV(nh)
  {}

  ~SPV_stair(){}


  void computeStairFlagSegments(Stair stair, std::vector<int> &phospheneFlag, int intensity);



};
