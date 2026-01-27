#ifndef __OGM_GRIDMAP_MANUAL_SYNC__
#define __OGM_GRIDMAP_MANUAL_SYNC__

#include "rclcpp/rclcpp.hpp"

template <typename T>
class ManualSyncEntry
{
  public:
    double stamp;
    std::shared_ptr<T> val;
    
    ManualSyncEntry(const T& _val, double _stamp)
    {
      stamp = _stamp;
      val = std::make_shared<T>(_val);
    }
};

template <typename T1, typename T2>
class ManualSync
{
  private:
    
    float slack;
    float window;

    std::list<ManualSyncEntry<T1>> q1;
    std::list<ManualSyncEntry<T2>> q2;

    template <typename T>
    static void prune(std::list<ManualSyncEntry<T>>& q, double stamp)
    {
      while (q.begin() != q.end() &&
          q.begin()->stamp < stamp)
        q.pop_front();
    } 

    template <typename T>
    static auto retrieve(std::list<T>& q, double stamp)
    {
      auto it = q.begin();
      auto best_it = it;
      while (it != q.end())
      {
        double e = std::abs(it->stamp - stamp); 
        double best_e = std::abs(best_it->stamp - stamp);
        if (e < best_e) best_it = it;
        ++it;
      }
      return best_it;
    } 

  public:
    ManualSync(float _slack, float _window)
    {
      slack = _slack;
      window = _window;
    }

    auto pop(double stamp)
    {
      auto cand1 = retrieve(q1, stamp); 
      auto cand2 = retrieve(q2, stamp);

      std::shared_ptr<T1> v1;
      std::shared_ptr<T2> v2;

      if (cand1 != q1.end()
          && cand2 != q2.end()
          && std::abs(cand1->stamp - stamp) < slack
          && std::abs(cand2->stamp - stamp) < slack)
      {
        v1 = cand1->val; 
        v2 = cand2->val; 
        prune(q1, cand1->stamp + 1e9);
        prune(q2, cand2->stamp + 1e9);
      } 

      return std::make_pair(v1,v2);
    }

    void push_t1(const T1& v, double stamp)
    {
      prune(q1, stamp - window);
      ManualSyncEntry<T1> e(v, stamp);
      q1.push_back(e);
    }

    void push_t2(const T2& v, double stamp)
    {
      prune(q2, stamp - window);
      ManualSyncEntry<T2> e(v, stamp);
      q2.push_back(e);
    }

    std::pair<double, double> range_t1()
    {
      double left = -1, right = -1;    
      if (q1.size() > 0)
      {
        left = q1.begin()->stamp;
        right = q1.rbegin()->stamp;
      }  
      return std::make_pair(left, right);  
    }

    std::pair<double, double> range_t2()
    {
      double left = -1, right = -1;    
      if (q2.size() > 0)
      {
        left = q2.begin()->stamp;
        right = q2.rbegin()->stamp;
      }  
      return std::make_pair(left, right);  
    }


    std::pair<size_t, size_t> size()
    {
      size_t s1 = q1.size();  
      size_t s2 = q2.size();
      return std::make_pair(s1,s2);  
    }
};


#endif
