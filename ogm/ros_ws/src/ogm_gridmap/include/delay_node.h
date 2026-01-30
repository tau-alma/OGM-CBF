#ifndef __OGM_GRIDMAP_DELAY_NODE__
#define __OGM_GRIDMAP_DELAY_NODE__

#include "rclcpp/rclcpp.hpp"

#include <chrono>   
using namespace std::chrono_literals;

template <typename T>
class DelayBufferEntry
{
  public:
    double stamp;
    std::shared_ptr<T> val;
    
    DelayBufferEntry(const T& _val, double _stamp)
    {
      stamp = _stamp;
      val = std::make_shared<T>(_val);
    }
};

template <typename T>
class DelayBuffer
{
  private:
    
    float delay;
    std::list<DelayBufferEntry<T>> q;

  public:

    void push(const T& v, double stamp)
    {
      DelayBufferEntry<T> e(v, stamp);
      q.push_back(e);
    }

    std::shared_ptr<T> query(double stamp)
    {
      std::shared_ptr<T> ret;

      for (auto it = q.begin(); it != q.end();)
      {
        if (stamp > it->stamp + delay)
        {
          ret = it->val;
          q.erase(it);
        }
        break;
      }
      return ret;
    }


    DelayBuffer(float _delay)
    {
      delay = _delay;
    }

};

template <typename M>
class DelayNode  : public rclcpp::Node
{
  private:
    
    typename rclcpp::Publisher<M>::SharedPtr pub;
    typename rclcpp::Subscription<M>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer;

    float delay;

    std::shared_ptr<DelayBuffer<std::shared_ptr<M>>> buffer;
    
    void callback(
        const std::shared_ptr<M> msg
        )
    {
      rclcpp::Time now = this->now();
      buffer->push(msg, now.seconds());
    }

    void tick()
    {
      rclcpp::Time ts = this->now();

      std::shared_ptr<std::shared_ptr<M>> msg; 
      while ((msg = buffer->query(ts.seconds())) != nullptr)
      {
        pub->publish(*(*msg));
      }
    }
  
  public:
    DelayNode() : Node("delayer")
    {
      delay = this->declare_parameter("delay", 5.);
      RCLCPP_INFO(this->get_logger(), "delay: %f", delay);

      buffer = std::make_shared<DelayBuffer<std::shared_ptr<M>>>(
          DelayBuffer<std::shared_ptr<M>>(delay));

      pub = this->create_publisher<M>(
		      "out",
          rclcpp::QoS(rclcpp::SensorDataQoS())
		      );
 
      sub = this->create_subscription<M>(
        "in",
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&DelayNode::callback, this, std::placeholders::_1)
        );
      
      timer = this->create_wall_timer(
		      100ms,
		      std::bind(&DelayNode::tick, this)
		      );
    }
};


template <typename M>
int init_delay_node(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<DelayNode<M>>());
  rclcpp::shutdown();
  return 0;
}

#endif
