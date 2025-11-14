#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <optional>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono;

// ==================== Estruturas ===================
struct HovercraftState {
    double x = 0, y = 0, z = 0;
    double vx = 0, vy = 0, vz = 0;
    double ax = 0, ay = 0, az = 0;
    double yaw = 0, pitch = 0, roll = 0;
    bool valid = false;
    uint64_t t_ms = 0;
};

struct TrajectoryRef {
    double x_ref=0, y_ref=0, vx_ref=0, vy_ref=0, ax_ref=0, ay_ref=0;
    uint64_t t_ms = 0;
};

// ==================== MQTT Wrapper ==================
class MQTTClient : public mosqpp::mosquittopp {
public:
    HovercraftState lastIMU{};
    std::mutex imu_mtx;
    bool new_imu = false;

    MQTTClient(const std::string& id, const std::string& host, int port)
        : mosqpp::mosquittopp(id.c_str())
    {
        connect_async(host.c_str(), port, 60);
        loop_start();
        subscribe(nullptr, "hovercraft/imu");
    }
    void on_message(const struct mosquitto_message *msg) override {
        if (msg && std::string(msg->topic) == "hovercraft/imu") {
            try {
                json j = json::parse(static_cast<const char*>(msg->payload));
                HovercraftState imu;
                imu.ax = j.value("ax", 0.0);
                imu.ay = j.value("ay", 0.0);
                imu.az = j.value("az", 0.0); 
                imu.yaw = j.value("yaw", 0.0);
                imu.pitch = j.value("pitch", 0.0);
                imu.roll = j.value("roll", 0.0);
                imu.t_ms = j.value("t_ms", 0);
                std::lock_guard<std::mutex> lk(imu_mtx);
                lastIMU = imu; new_imu = true;
            } catch(...) { }
        }
    }
    // Publica estimativa (opcional para monitoramento)
    void publish_pose(const HovercraftState& state) {
        json j = {{"x", state.x}, {"y", state.y}, {"z", state.z},
                  {"vx", state.vx}, {"vy", state.vy}, {"vz", state.vz},
                  {"ax", state.ax}, {"ay", state.ay}, {"az", state.az},
                  {"yaw", state.yaw},{"pitch", state.pitch},{"roll",state.roll},
                  {"t_ms", state.t_ms}};
        std::string payload = j.dump();
        publish(nullptr, "hovercraft/ekfstate", payload.size(), payload.c_str());
    }
    // Publica comando de motor/servo
    void publish_cmd(uint32_t motor_us, uint32_t servo_us) {
        json j = {{"motor_us", motor_us}, {"servo_us", servo_us}};
        std::string payload = j.dump();
        publish(nullptr, "hovercraft/cmd", payload.size(), payload.c_str());
    }
};

// ============== Processamento de Imagem ============
std::optional<HovercraftState> detect_hovercraft_blob(
    const cv::Mat& frame, cv::Scalar hsv_low, cv::Scalar hsv_high,
    double fx, double fy, double cx, double cy, double obj_real_mm)
{
    HovercraftState state;
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::inRange(hsv, hsv_low, hsv_high, mask);
    cv::erode(mask, mask, {}, {}, 1);
    cv::dilate(mask, mask, {}, {}, 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double best_area = 0; int best = -1;
    for (size_t i=0;i<contours.size();i++) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) { best_area=area; best=i;}
    }
    if (best<0 || best_area<20.0) return std::nullopt;
    cv::Moments mu = cv::moments(contours[best]);
    if (mu.m00 == 0.0) return std::nullopt;
    cv::Point2f c(mu.m10/mu.m00, mu.m01/mu.m00);
    double diam_img = 2.0 * sqrt(best_area/CV_PI);
    double z_mm = (obj_real_mm*fx)/diam_img;
    double x_mm = (c.x-cx)*z_mm/fx, y_mm = (c.y-cy)*z_mm/fy;
    state.x = x_mm; state.y = y_mm; state.z = z_mm;
    state.t_ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    state.valid = true;
    return state;
}

// =================== Fusão/EKF "stub" ====================
HovercraftState run_ekf(const HovercraftState& last,
                        const std::optional<HovercraftState>& cam_obs,
                        const HovercraftState& lastIMU)
{
    HovercraftState est = last;
    // Propaga estado usando aceleração da IMU/integrador Euler
    double dt = 0.02;
    est.ax = lastIMU.ax; est.ay = lastIMU.ay; est.az = lastIMU.az;
    est.vx += est.ax*dt; est.vy += est.ay*dt; est.vz += est.az*dt;
    est.x += est.vx*dt; est.y += est.vy*dt; est.z += est.vz*dt;
    if (cam_obs && cam_obs->valid) {
        est.x = 0.8*est.x + 0.2*cam_obs->x;
        est.y = 0.8*est.y + 0.2*cam_obs->y;
        est.z = 0.8*est.z + 0.2*cam_obs->z;
    }
    est.t_ms = lastIMU.t_ms;
    return est;
}

// ============== Controle (PI) ================
void run_control(const HovercraftState& est, const TrajectoryRef& ref,
                MQTTClient& mqtt)
{
    // Erro de posição, integra (para gerar erro de velocidade)
    static double int_ex = 0, int_ey = 0;
    double ex = ref.x_ref - est.x, ey = ref.y_ref - est.y;
    int_ex += ex*0.02, int_ey += ey*0.02;
    // Gera aceleração desejada (PI)
    double ax_des = 2.0*ex + 0.2*int_ex;
    double ay_des = 2.0*ey + 0.2*int_ey;
    // Erro entre aceleração desejada x medida (IMU)
    double e_ax = ax_des - est.ax;
    double e_ay = ay_des - est.ay;
    // Controle simplificado (direção e intensidade)
    double acc_cmd = sqrt(e_ax*e_ax + e_ay*e_ay);
    double dir_rad = atan2(e_ay, e_ax);
    // Mapear para comando PWM
    uint32_t motor_us = std::clamp(1200u + uint32_t(acc_cmd*250), 1000u, 2000u);
    uint32_t servo_us = std::clamp(1500u + int(400.0*dir_rad/3.14), 1000, 2000);
    // Comando
    mqtt.publish_cmd(motor_us, servo_us);
}

// =============== MAIN =======================
int main() {
    // Camera intrinsics/config
    const int W = 640, H = 480;  constexpr double FOVX_DEG = 62.0;
    double fovx_rad = FOVX_DEG * CV_PI / 180.0;
    double fx = (W*0.5) / std::tan(fovx_rad*0.5), fy = fx, cx = W*0.5, cy = H*0.5;
    constexpr double OBJ_REAL_MM = 100.0;
    cv::Scalar HSV_LOW(100, 90, 80), HSV_HIGH(130, 255, 255);

    // MQTT
    mosqpp::lib_init();
    MQTTClient mqtt("hc_host", "broker.hivemq.com", 1883);

    // Camera
    cv::VideoCapture cap(0); cap.set(cv::CAP_PROP_FRAME_WIDTH, W); cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);

    HovercraftState est_state, last_imu;
    TrajectoryRef traj_ref{0, 0, 0, 0, 0, 0}; // Trajetória fixa, mude conforme precisar

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) continue;
        auto cam_obs = detect_hovercraft_blob(frame, HSV_LOW, HSV_HIGH, fx, fy, cx, cy, OBJ_REAL_MM);

        // Coleta último IMU recebido (mutex protegido)
        {
            std::lock_guard<std::mutex> lk(mqtt.imu_mtx);
            last_imu = mqtt.lastIMU;
        }
        est_state = run_ekf(est_state, cam_obs, last_imu);

        // Controle com base na referência
        run_control(est_state, traj_ref, mqtt);

        // display opcional
        if (cam_obs && cam_obs->valid)
            cv::circle(frame, cv::Point2f(W/2 + cam_obs->x*4,W/2 + cam_obs->y*4), 8, cv::Scalar(255,0,0), 2);
        cv::imshow("Hovercraft", frame); if (cv::waitKey(1) == 27) break;
    }
    mosqpp::lib_cleanup();
    return 0;
}
