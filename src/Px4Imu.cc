#include "Px4Imu.h"
#include "Px4Vio.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <thread>
namespace ORB_SLAM3 { namespace px4 {
struct ImuReceiver::Impl {
    mutable std::mutex mutex;
    std::condition_variable wake;
    ImuBuffer buffer;
    std::string fault="waiting_imu";
    std::shared_ptr<rclcpp::Context> context;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
    std::atomic<bool> stop{false};
    std::thread worker;
    bool calibrated=false;
    uint8_t accelCalibration=0,gyroCalibration=0;
    Impl(int domain,const std::string& prefix) {
        context=std::make_shared<rclcpp::Context>();
        rclcpp::InitOptions init;init.set_domain_id(domain);init.shutdown_on_signal=false;
        context->init(0,nullptr,init);
        rclcpp::NodeOptions options;options.context(context).use_global_arguments(false);
        node=std::make_shared<rclcpp::Node>("decxin_px4_raw_imu",options);
        sub=node->create_subscription<px4_msgs::msg::SensorCombined>(
            TopicName(prefix,"/fmu/out/sensor_combined",0),rclcpp::SensorDataQoS().keep_last(100),
            [this](px4_msgs::msg::SensorCombined::ConstSharedPtr m) {
                std::lock_guard<std::mutex> lock(mutex);
                const auto now=SystemMicros();
                const double age=(double(now)-double(m->timestamp))*1e-6;
                const bool invalid=m->accelerometer_timestamp_relative==m->RELATIVE_TIMESTAMP_INVALID ||
                    std::abs(int64_t(m->accelerometer_timestamp_relative))>200 ||
                    !m->gyro_integral_dt || !m->accelerometer_integral_dt ||
                    m->gyro_integral_dt>30000 || m->accelerometer_integral_dt>30000 ||
                    m->accelerometer_clipping || m->gyro_clipping || age<-.005 || age>.25;
                if(invalid) {buffer.Reset();++buffer.rejected;fault="imu_time_or_measurement_invalid";wake.notify_all();return;}
                if(calibrated && (accelCalibration!=m->accel_calibration_count || gyroCalibration!=m->gyro_calibration_count)) {
                    buffer.Reset();fault="imu_calibration_changed";
                }
                calibrated=true;accelCalibration=m->accel_calibration_count;gyroCalibration=m->gyro_calibration_count;
                ImuReading r;r.time=m->timestamp*1e-6;r.accel=m->accelerometer_m_s2;r.gyro=m->gyro_rad;
                buffer.Add(r);fault="ok";wake.notify_all();
            });
        rclcpp::ExecutorOptions eo;eo.context=context;
        executor=std::make_unique<rclcpp::executors::SingleThreadedExecutor>(eo);executor->add_node(node);
        worker=std::thread([this] {
            try {while(!stop) {executor->spin_some(std::chrono::milliseconds(2));std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
            catch(const std::exception& e) {std::lock_guard<std::mutex> lock(mutex);buffer.Reset();fault=e.what();stop=true;wake.notify_all();}
        });
    }
    ~Impl() {stop=true;wake.notify_all();if(worker.joinable())worker.join();executor.reset();node.reset();context->shutdown("IMU receiver stopped");}
};
ImuReceiver::ImuReceiver(int domain,const std::string& prefix):impl_(std::make_unique<Impl>(domain,prefix)) {}
ImuReceiver::~ImuReceiver()=default;
bool ImuReceiver::Interval(double start,double end,std::vector<ImuReading>& out,uint64_t& epoch,std::string& reason) {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    impl_->wake.wait_for(lock,std::chrono::milliseconds(35),[&] {
        return impl_->stop || (!impl_->buffer.data.empty() && impl_->buffer.data.back().time>=end);
    });
    epoch=impl_->buffer.epoch;
    return impl_->buffer.Slice(start,end,out,reason);
}
std::string ImuReceiver::Status() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    std::ostringstream s;s<<"received="<<impl_->buffer.received<<" rejected="<<impl_->buffer.rejected
        <<" epoch="<<impl_->buffer.epoch<<" buffered="<<impl_->buffer.data.size()<<" state="<<impl_->fault;
    return s.str();
}
}}
