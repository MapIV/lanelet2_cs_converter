#ifndef LANELET2_HPP
#define LANELET2_HPP

#include <vector>
#include <iostream>

struct LL2Point
{
  unsigned int id = 0;
  double lat = 0, lon = 0;
  double local_x = 0, local_y = 0;
  double elevation = 0;
  bool valid = false;
  
  bool operator<(const LL2Point& right) const
  {
    return id < right.id;
  }

  void print() const
  {
    std::cout << "ID        : " << id << std::endl;
    std::cout << "lat       : " << lat << std::endl;
    std::cout << "lon       : " << lon << std::endl;
    std::cout << "local_x   : " << local_x << std::endl;
    std::cout << "local_y   : " << local_y << std::endl;
    std::cout << "elevation : " << elevation << std::endl;
    std::cout << "valid     : " << valid << std::endl;
  }
};

class LL2Points
{
public:
  LL2Points()
  {
  }

  void addNewPoint(const LL2Point& point)
  {
    tmp_vec_.push_back(point);
  }

  void makeVec()
  {
    int max_id = tmp_vec_[0].id;
    for (int i = 1; i < tmp_vec_.size(); i++)
    {
      if (max_id < tmp_vec_[i].id) max_id = tmp_vec_[i].id;
    }
    std::cout << "max id: " << max_id << std::endl;

    point_vec_.resize(max_id);

    for (int i = 0; i < tmp_vec_.size(); i++)
    {
      if (point_vec_[tmp_vec_[i].id].valid)
      {
        std::cerr << "\033[31;1mError: Point ID is not identical\033[m" << std::endl;
        exit(1);
      }
      point_vec_[tmp_vec_[i].id] = tmp_vec_[i];
    }
  }

  void printVec()
  {
    for (int i = 0; i < point_vec_.size(); i++)
    {
      point_vec_[i].print();
      std::cout << "===================================" << std::endl;
    }
  }

  void printInfo()
  {
    std::cout << "Vector size: " << point_vec_.size() << std::endl;
  }

private:
  std::vector<LL2Point> point_vec_;
  std::vector<LL2Point> tmp_vec_;
};

#endif
