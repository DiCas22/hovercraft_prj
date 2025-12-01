#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <queue>
#include <optional>
#include <vector>
#include <deque>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <fstream>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>

#include <termios.h>
#include <unistd.h>

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
    double x_ref = 0, y_ref = 0;
    double vx_ref = 0, vy_ref = 0;
    double ax_ref = 0, ay_ref = 0;
    uint64_t t_ms = 0;
};

enum class Mode {
    AUTONOMOUS = 1,
    TELEOP     = 2
};

enum class SystemState {
    SISTEMA_LIGADO,
    TRAJETORIA_RECEBIDA,
    EM_TRAJETO,
    TELEOP_ATIVADO,
    AUTONOMO_ATIVADO,
    FALHA_SENSORES
};

// ==================== Terminal raw mode guard ==================
struct TerminalRawGuard {
    termios orig{};
    bool ok = false;

    TerminalRawGuard() {
        if (tcgetattr(STDIN_FILENO, &orig) == 0) {
            termios raw = orig;
            raw.c_lflag &= ~(ICANON | ECHO); // sem modo canônico e sem eco
            raw.c_cc[VMIN]  = 1;
            raw.c_cc[VTIME] = 0;
            tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            ok = true;
        }
    }

    ~TerminalRawGuard() {
        if (ok) {
            tcsetattr(STDIN_FILENO, TCSANOW, &orig);
        }
    }
};

// ==================== Logger de estados -> CSV ==================
std::string state_to_string(SystemState s) {
    switch (s) {
        case SystemState::SISTEMA_LIGADO:      return "Sistema ligado";
        case SystemState::TRAJETORIA_RECEBIDA: return "Trajetoria recebida";
        case SystemState::EM_TRAJETO:          return "Em trajeto";
        case SystemState::TELEOP_ATIVADO:      return "Teleop ativado";
        case SystemState::AUTONOMO_ATIVADO:    return "Autonomo ativado";
        case SystemState::FALHA_SENSORES:      return "Falha nos sensores";
    }
    return "Desconhecido";
}

void log_state_change(SystemState new_state, const std::string& reason = "") {
    static std::mutex log_mtx;
    static SystemState last_state = SystemState::SISTEMA_LIGADO;
    static bool first = true;

    std::lock_guard<std::mutex> lk(log_mtx);

    // Evita registrar o mesmo estado repetidas vezes
    if (!first && new_state == last_state) {
        return;
    }
    first = false;
    last_state = new_state;

    static std::ofstream log_file("hovercraft_log.csv", std::ios::app);
    if (!log_file) {
        std::cerr << "[LOG] Falha ao abrir hovercraft_log.csv\n";
        return;
    }

    // Cabeçalho se arquivo recém-criado
    if (log_file.tellp() == 0) {
        log_file << "timestamp_ms,datetime,state,reason\n";
    }

    auto now = system_clock::now();
    auto t_ms = duration_cast<milliseconds>(now.time_since_epoch()).count();
    std::time_t t_c = system_clock::to_time_t(now);
    std::tm tm = *std::localtime(&t_c);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);

    log_file << t_ms << ",\"" << buf << "\",\""
             << state_to_string(new_state) << "\"";
    if (!reason.empty()) {
        log_file << ",\"" << reason << "\"";
    }
    log_file << "\n";
    log_file.flush();

    std::cout << "[STATE] " << state_to_string(new_state);
    if (!reason.empty()) std::cout << " (" << reason << ")";
    std::cout << "\n";
}

// ==================== MQTT Wrapper ==================
class MQTTClient : public mosqpp::mosquittopp {
public:
    HovercraftState lastIMU{};
    std::mutex imu_mtx;
    bool new_imu = false;

    // Trajectory queue (FIFO)
    std::mutex traj_mtx;
    std::deque<TrajectoryRef> traj_queue;

    MQTTClient(const std::string& id, const std::string& host, int port)
        : mosqpp::mosquittopp(id.c_str())
    {
        connect_async(host.c_str(), port, 60);
        loop_start();
    }

    void on_connect(int rc) override {
        if (rc == 0) {
            subscribe(nullptr, "hovercraft/imu");
            subscribe(nullptr, "hovercraft/traj");
            std::cout << "[MQTT] Connected, subscribed to hovercraft/imu + hovercraft/traj\n";
        } else {
            std::cerr << "[MQTT] Connect failed, rc=" << rc << "\n";
        }
    }

    void on_message(const struct mosquitto_message *msg) override {
        if (!msg) return;

        std::string topic = msg->topic ? std::string(msg->topic) : std::string();
        std::string payload(
            static_cast<const char*>(msg->payload),
            static_cast<size_t>(msg->payloadlen)
        );

        try {
            json j = json::parse(payload);

            if (topic == "hovercraft/imu") {
                HovercraftState imu;
                imu.ax = j.value("ax", 0.0);
                imu.ay = j.value("ay", 0.0);
                imu.az = j.value("az", 0.0);
                imu.yaw = j.value("yaw", 0.0);
                imu.pitch = j.value("pitch", 0.0);
                imu.roll = j.value("roll", 0.0);
                imu.t_ms = j.value("t_ms", 0ull);

                std::lock_guard<std::mutex> lk(imu_mtx);
                lastIMU = imu;
                new_imu = true;
            }
            else if (topic == "hovercraft/traj") {
                handle_trajectory_message(j);
            }
        } catch (const std::exception& e) {
            std::cerr << "[MQTT] JSON parse error on topic " << topic
                      << ": " << e.what() << "\n";
        }
    }

    // Publica estimativa (monitoramento / plot externo)
    void publish_pose(const HovercraftState& state) {
        json j = {
            {"x", state.x}, {"y", state.y}, {"z", state.z},
            {"vx", state.vx}, {"vy", state.vy}, {"vz", state.vz},
            {"ax", state.ax}, {"ay", state.ay}, {"az", state.az},
            {"yaw", state.yaw}, {"pitch", state.pitch}, {"roll", state.roll},
            {"t_ms", state.t_ms}
        };
        std::string payload = j.dump();
        publish(nullptr, "hovercraft/ekfstate",
                static_cast<int>(payload.size()), payload.c_str());
    }

    // Publica comando de motor/servo
    void publish_cmd(uint32_t motor_us, uint32_t servo_us) {
        json j = {{"motor_us", motor_us}, {"servo_us", servo_us}};
        std::string payload = j.dump();
        publish(nullptr, "hovercraft/cmd",
                static_cast<int>(payload.size()), payload.c_str());
    }

    // ======= Trajectory helpers =======
    void handle_trajectory_message(const json& j) {
        auto make_ref = [](const json& jj) -> TrajectoryRef {
            TrajectoryRef r;
            r.x_ref  = jj.value("x_ref",  0.0);
            r.y_ref  = jj.value("y_ref",  0.0);
            r.vx_ref = jj.value("vx_ref", 0.0);
            r.vy_ref = jj.value("vy_ref", 0.0);
            r.ax_ref = jj.value("ax_ref", 0.0);
            r.ay_ref = jj.value("ay_ref", 0.0);
            r.t_ms   = jj.value("t_ms",   0ull);
            return r;
        };

        std::lock_guard<std::mutex> lk(traj_mtx);

        if (j.is_array()) {
            std::cout << "[TRAJ] Received array of " << j.size() << " trajectories:\n";
            for (const auto& elem : j) {
                if (!elem.is_object()) continue;
                TrajectoryRef r = make_ref(elem);
                traj_queue.push_back(r);
                std::cout << "  - x_ref=" << r.x_ref
                          << ", y_ref=" << r.y_ref << "\n";
            }
        } else if (j.is_object()) {
            TrajectoryRef r = make_ref(j);
            traj_queue.push_back(r);
            std::cout << "[TRAJ] Received trajectory: x_ref="
                      << r.x_ref << ", y_ref=" << r.y_ref << "\n";
        } else {
            std::cerr << "[TRAJ] Invalid trajectory JSON (not object/array)\n";
        }
    }

    bool get_next_trajectory(TrajectoryRef& out) {
        std::lock_guard<std::mutex> lk(traj_mtx);
        if (traj_queue.empty()) return false;
        out = traj_queue.front();
        traj_queue.pop_front();
        return true;
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
    double best_area = 0;
    int best = -1;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > best_area) { best_area = area; best = static_cast<int>(i); }
    }
    if (best < 0 || best_area < 20.0) return std::nullopt;

    cv::Moments mu = cv::moments(contours[best]);
    if (mu.m00 == 0.0) return std::nullopt;

    cv::Point2f c(static_cast<float>(mu.m10 / mu.m00),
                  static_cast<float>(mu.m01 / mu.m00));

    double diam_img = 2.0 * std::sqrt(best_area / CV_PI);
    double z_mm = (obj_real_mm * fx) / diam_img;
    double x_mm = (c.x - cx) * z_mm / fx;
    double y_mm = (c.y - cy) * z_mm / fy;

    state.x = x_mm;
    state.y = y_mm;
    state.z = z_mm;
    state.t_ms = duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
    state.valid = true;
    return state;
}

// =================== Fusão/EKF "stub" ====================
HovercraftState run_ekf(const HovercraftState& last,
                        const std::optional<HovercraftState>& cam_obs,
                        const HovercraftState& lastIMU)
{
    HovercraftState est = last;

    // Loop do filtro a 50 Hz
    double dt = 0.02;

    // Propaga estado usando aceleração da IMU (Euler)
    est.ax = lastIMU.ax;
    est.ay = lastIMU.ay;
    est.az = lastIMU.az;

    est.vx += est.ax * dt;
    est.vy += est.ay * dt;
    est.vz += est.az * dt;

    est.x += est.vx * dt;
    est.y += est.vy * dt;
    est.z += est.vz * dt;

    // Copia ângulos da IMU
    est.yaw   = lastIMU.yaw;
    est.pitch = lastIMU.pitch;
    est.roll  = lastIMU.roll;

    // Correção simples com câmera
    if (cam_obs && cam_obs->valid) {
        est.x = 0.8 * est.x + 0.2 * cam_obs->x;
        est.y = 0.8 * est.y + 0.2 * cam_obs->y;
        est.z = 0.8 * est.z + 0.2 * cam_obs->z;
    }

    est.t_ms = duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
    return est;
}

// ============== Controle (PI) ================
void run_control(const HovercraftState& est, const TrajectoryRef& ref,
                 MQTTClient& mqtt)
{
    // Erro de posição, integra (para gerar erro de velocidade)
    static double int_ex = 0, int_ey = 0;
    constexpr double dt = 0.02; // deve bater com a frequência real do loop

    double ex = ref.x_ref - est.x;
    double ey = ref.y_ref - est.y;
    int_ex += ex * dt;
    int_ey += ey * dt;

    // Gera aceleração desejada (PI)
    double ax_des = 2.0 * ex + 0.2 * int_ex;
    double ay_des = 2.0 * ey + 0.2 * int_ey;

    // Erro entre aceleração desejada x medida (IMU)
    double e_ax = ax_des - est.ax;
    double e_ay = ay_des - est.ay;

    // Controle simplificado (direção e intensidade)
    double acc_cmd = std::sqrt(e_ax * e_ax + e_ay * e_ay);
    double dir_rad = std::atan2(e_ay, e_ax);

    // Mapear para comando PWM
    uint32_t motor_us = std::clamp(
        1200u + static_cast<uint32_t>(acc_cmd * 250.0),
        1000u, 2000u
    );

    int servo_raw = 1500 + static_cast<int>(400.0 * dir_rad / 3.14);
    uint32_t servo_us = static_cast<uint32_t>(
        std::clamp(servo_raw, 1000, 2000)
    );

    // Comando
    mqtt.publish_cmd(motor_us, servo_us);
}

// ================== Trajectory completion ==================
bool trajectory_finished(const HovercraftState& est, const TrajectoryRef& ref) {
    double ex = ref.x_ref - est.x;
    double ey = ref.y_ref - est.y;
    double dist = std::sqrt(ex * ex + ey * ey);
    // Tolerância de 20 mm (ajuste conforme necessário)
    return dist < 20.0;
}

// =============== MAIN (pipeline em threads) =======================
int main() {
    // MQTT client
    mosqpp::lib_init();
    MQTTClient mqtt("hc_host", "broker.hivemq.com", 1883);

    // Flags e estados compartilhados
    std::atomic<bool> running{true};
    std::atomic<Mode> g_mode{Mode::AUTONOMOUS};
    std::atomic<bool> sensor_failure{false};

    // Teleop state (PWM pulses)
    std::atomic<uint32_t> teleop_motor_us{1000}; // ESC off
    std::atomic<uint32_t> teleop_servo_us{1500}; // center

    // Camera obs compartilhada
    std::mutex cam_mtx;
    std::optional<HovercraftState> last_cam_obs;
    bool new_cam_obs = false;

    // Estado estimado compartilhado
    std::mutex est_mtx;
    HovercraftState est_state{};

    // Camera intrinsics/config
    const int W = 640, H = 480;
    constexpr double FOVX_DEG = 62.0;
    double fovx_rad = FOVX_DEG * CV_PI / 180.0;
    double fx = (W * 0.5) / std::tan(fovx_rad * 0.5);
    double fy = fx;
    double cx = W * 0.5;
    double cy = H * 0.5;
    constexpr double OBJ_REAL_MM = 100.0;
    cv::Scalar HSV_LOW(100, 90, 80), HSV_HIGH(130, 255, 255);

    // ---------- UI inicial (modo) ----------
    std::cout << "==================== HOVERCRAFT PIPELINE ====================\n";
    std::cout << "Comandos em tempo real (depois de escolher o modo):\n";
    std::cout << "  1 = modo AUTONOMOUS\n";
    std::cout << "  2 = modo TELEOP\n";
    std::cout << "  w/s = aumenta/diminui motor (PWM)\n";
    std::cout << "  q/e = vira esquerda/direita (servo)\n";
    std::cout << "  x   = sair do TELEOP (voltar para AUTONOMOUS)\n";
    std::cout << "  Ctrl+C = encerrar programa\n";
    std::cout << "=============================================================\n";

    log_state_change(SystemState::SISTEMA_LIGADO, "Programa iniciado");

    // Pergunta modo inicial (linha-bufferizado, precisa ENTER)
    std::cout << "Selecione modo inicial (1=AUTONOMOUS, 2=TELEOP): ";
    int mode_sel = 1;
    if (std::cin >> mode_sel && mode_sel == 2) {
        g_mode.store(Mode::TELEOP);
        std::cout << "[MODE] TELEOP selecionado.\n";
        log_state_change(SystemState::TELEOP_ATIVADO, "Modo inicial");
    } else {
        g_mode.store(Mode::AUTONOMOUS);
        std::cout << "[MODE] AUTONOMOUS selecionado.\n";
        log_state_change(SystemState::AUTONOMO_ATIVADO, "Modo inicial");
    }

    // ---------- THREAD: Image recognition ----------
    std::thread image_thread([&](){
        cv::VideoCapture cap(0);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, W);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);

        if (!cap.isOpened()) {
            std::cerr << "[IMG] Failed to open camera 0\n";
            return;
        }

        while (running.load()) {
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            auto obs = detect_hovercraft_blob(
                frame, HSV_LOW, HSV_HIGH, fx, fy, cx, cy, OBJ_REAL_MM
            );

            if (obs && obs->valid) {
                std::lock_guard<std::mutex> lk(cam_mtx);
                last_cam_obs = *obs;
                new_cam_obs = true;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        cap.release();
    });

    // ---------- THREAD: EKF / filter + detecção de falha ----------
    std::thread filter_thread([&](){
        HovercraftState local_est{};
        const auto period = std::chrono::milliseconds(20); // 50 Hz
        auto last_imu_time = steady_clock::now();
        const auto sensor_timeout = std::chrono::milliseconds(1000); // 1 s sem IMU => falha

        while (running.load()) {
            HovercraftState imu{};
            bool has_imu = false;
            {
                std::lock_guard<std::mutex> lk(mqtt.imu_mtx);
                if (mqtt.new_imu) {
                    imu = mqtt.lastIMU;
                    mqtt.new_imu = false;
                    has_imu = true;
                }
            }

            if (has_imu) {
                last_imu_time = steady_clock::now();

                std::optional<HovercraftState> cam_obs_copy;
                {
                    std::lock_guard<std::mutex> lk(cam_mtx);
                    if (new_cam_obs) {
                        cam_obs_copy = last_cam_obs;
                        new_cam_obs = false;
                    }
                }

                local_est = run_ekf(local_est, cam_obs_copy, imu);

                {
                    std::lock_guard<std::mutex> lk(est_mtx);
                    est_state = local_est;
                }

                mqtt.publish_pose(local_est);
            }

            // Verifica timeout de sensor
            auto now = steady_clock::now();
            if (!sensor_failure.load() &&
                now - last_imu_time > sensor_timeout) {

                sensor_failure.store(true);
                mqtt.publish_cmd(1000, 1500); // desliga motor e centraliza servo
                log_state_change(SystemState::FALHA_SENSORES,
                                 "Sem dados de IMU por > 1s");
            }

            std::this_thread::sleep_for(period);
        }
    });

    // ---------- THREAD: Control (autonomous / teleop) ----------
    std::thread control_thread([&](){
        TrajectoryRef active_ref{};
        bool has_traj = false;
        const auto period = std::chrono::milliseconds(50); // 20 Hz

        while (running.load()) {
            // Se falha de sensor, garante motores zerados e não faz nada
            if (sensor_failure.load()) {
                mqtt.publish_cmd(1000, 1500);
                std::this_thread::sleep_for(period);
                continue;
            }

            Mode mode = g_mode.load();

            // -------- TELEOP --------
            if (mode == Mode::TELEOP) {
                uint32_t m = teleop_motor_us.load();
                uint32_t s = teleop_servo_us.load();
                mqtt.publish_cmd(m, s);
                std::this_thread::sleep_for(period);
                continue;
            }

            // -------- AUTONOMOUS --------
            HovercraftState est{};
            {
                std::lock_guard<std::mutex> lk(est_mtx);
                est = est_state;
            }

            if (!has_traj) {
                TrajectoryRef next{};
                if (mqtt.get_next_trajectory(next)) {
                    active_ref = next;
                    has_traj = true;
                    log_state_change(SystemState::TRAJETORIA_RECEBIDA,
                                     "x_ref=" + std::to_string(active_ref.x_ref) +
                                     ", y_ref=" + std::to_string(active_ref.y_ref));
                    log_state_change(SystemState::EM_TRAJETO,
                                     "Iniciando seguimento de trajetoria");
                    std::cout << "[CTRL] Starting trajectory: x_ref="
                              << active_ref.x_ref << ", y_ref=" << active_ref.y_ref << "\n";
                }
            }

            if (has_traj) {
                run_control(est, active_ref, mqtt);

                if (trajectory_finished(est, active_ref)) {
                    std::cout << "[CTRL] Trajectory finished.\n";
                    has_traj = false;
                    log_state_change(SystemState::AUTONOMO_ATIVADO,
                                     "Trajetoria finalizada, mantendo autonomo parado");
                }
            } else {
                // Sem trajetória: poderia manter motores parados, se quiser
                // mqtt.publish_cmd(1000, 1500);
            }

            std::this_thread::sleep_for(period);
        }
    });

    // ---------- THREAD: Keyboard / teleop (sem ENTER) ----------
    std::thread keyboard_thread([&](){
        TerminalRawGuard guard;  // deixa terminal em modo raw enquanto a thread existir

        std::cout << "[KEY] Controle em tempo real ativo. 'x' sai do TELEOP.\n";

        while (running.load()) {
            int ch = getchar();   // bloqueia só nesta thread
            if (ch == EOF) {
                break;
            }

            char c = static_cast<char>(ch);

            // troca de modo instantânea
            if (c == '1') {
                g_mode.store(Mode::AUTONOMOUS);
                log_state_change(SystemState::AUTONOMO_ATIVADO,
                                 "Comando do teclado (1)");
                std::cout << "\n[MODE] AUTONOMOUS selecionado.\n";
            } else if (c == '2') {
                g_mode.store(Mode::TELEOP);
                log_state_change(SystemState::TELEOP_ATIVADO,
                                 "Comando do teclado (2)");
                std::cout << "\n[MODE] TELEOP selecionado.\n";
            }

            Mode mode = g_mode.load();

            // 'x' sai APENAS do TELEOP (volta pra AUTONOMOUS e zera comandos)
            if ((c == 'x' || c == 'X') && mode == Mode::TELEOP) {
                teleop_motor_us.store(1000);
                teleop_servo_us.store(1500);
                mqtt.publish_cmd(1000, 1500); // garante motor parado / servo central
                g_mode.store(Mode::AUTONOMOUS);
                log_state_change(SystemState::AUTONOMO_ATIVADO,
                                 "Saida do teleop (x)");
                std::cout << "\n[MODE] Saindo do TELEOP -> AUTONOMOUS.\n";
                continue;
            }

            // TELOP: comandos só se estiver em modo TELEOP e sem falha de sensor
            if (mode == Mode::TELEOP && !sensor_failure.load()) {
                if (c == 'w' || c == 'W') {
                    uint32_t m = teleop_motor_us.load();
                    m = std::min(m + 10u, 2000u);
                    teleop_motor_us.store(m);
                } else if (c == 's' || c == 'S') {
                    uint32_t m = teleop_motor_us.load();
                    if (m > 1000u) m -= 10u;
                    teleop_motor_us.store(m);
                } else if (c == 'q' || c == 'Q') {
                    uint32_t s = teleop_servo_us.load();
                    if (s > 1000u) s -= 10u;
                    teleop_servo_us.store(s);
                } else if (c == 'e' || c == 'E') {
                    uint32_t s = teleop_servo_us.load();
                    s = std::min(s + 10u, 2000u);
                    teleop_servo_us.store(s);
                }

                std::cout << "\r[TELEOP] motor_us=" << teleop_motor_us.load()
                          << " servo_us=" << teleop_servo_us.load()
                          << "           " << std::flush;
            }
        }
    });

    if (keyboard_thread.joinable()) keyboard_thread.join();

    // Finaliza threads
    running.store(false);
    if (image_thread.joinable())   image_thread.join();
    if (filter_thread.joinable())  filter_thread.join();
    if (control_thread.joinable()) control_thread.join();

    mosqpp::lib_cleanup();
    return 0;
}
