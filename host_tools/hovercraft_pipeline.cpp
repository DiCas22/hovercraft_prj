#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <optional>
#include <vector>
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

#include "msg_types.h"
#include "bufferCircular.h"

bufferCircular g_buffer(256);

// ==================== Modos de operação ===================
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

// ==================== Terminal raw mode ==================
struct TerminalRawGuard {
    termios orig{};
    bool ok = false;

    TerminalRawGuard() {
        if (tcgetattr(STDIN_FILENO, &orig) == 0) {
            termios raw = orig;
            raw.c_lflag &= ~(ICANON | ECHO);
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

// Registra mudança de estado do sistema em CSV + stdout
void log_state_change(SystemState new_state, const std::string& reason = "") {
    static std::mutex log_mtx;
    static SystemState last_state = SystemState::SISTEMA_LIGADO;
    static bool first = true;

    std::lock_guard<std::mutex> lk(log_mtx);

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
// Encapsula MQTT: recebe IMU/trajetória e publica estado e comandos
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

    // Publica estimativa (para monitoramento externo)
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

    // Publica comando motor/servo
    void publish_cmd(uint32_t motor_us, uint32_t servo_us) {
        json j = {{"motor_us", motor_us}, {"servo_us", servo_us}};
        std::string payload = j.dump();
        publish(nullptr, "hovercraft/cmd",
                static_cast<int>(payload.size()), payload.c_str());
    }

    // Converte JSON de trajetória para mensagens no buffer
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

        auto push_traj = [](const TrajectoryRef& r) {
            BufferMessage msg;
            msg.type     = MsgType::TRAJECTORY;
            msg.priority = 5;
            msg.payload  = r;
            g_buffer.push(msg);
        };

        if (j.is_array()) {
            for (const auto& elem : j) {
                if (!elem.is_object()) continue;
                TrajectoryRef r = make_ref(elem);
                push_traj(r);
            }
        } else if (j.is_object()) {
            TrajectoryRef r = make_ref(j);
            push_traj(r);
        }
    }
};

// ============== Processamento de Imagem ============
// Detecta blob colorido e estima posição 3D aproximada
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

// =================== Fusão/EKF simples ====================
// Atualiza estado estimado usando IMU e correção da câmera
HovercraftState run_ekf(const HovercraftState& last,
    const std::optional<HovercraftState>& cam_obs,
    const HovercraftState& imu,
    double dt)
{
HovercraftState est = last;

// Protege contra dt maluco
if (dt < 0.0) dt = 0.0;
if (dt > 0.1) dt = 0.1; // máx 100 ms

// --------- 1) Propagação (modelo constante velocidade) ----------
est.x += est.vx * dt;
est.y += est.vy * dt;
est.z += est.vz * dt;

// Mantém acelerações e orientação vindas da IMU
est.ax = imu.ax;
est.ay = imu.ay;
est.az = imu.az;

est.yaw   = imu.yaw;
est.pitch = imu.pitch;
est.roll  = imu.roll;

// --------- 2) Correção por câmera (se tiver medição nova) -------
if (cam_obs && cam_obs->valid) {
// Peso da medição da câmera (entre 0 e 1)
constexpr double alpha = 0.25; // 25% câmera, 75% previsão

// Low-pass entre previsão e medição
est.x = (1.0 - alpha) * est.x + alpha * cam_obs->x;
est.y = (1.0 - alpha) * est.y + alpha * cam_obs->y;
est.z = (1.0 - alpha) * est.z + alpha * cam_obs->z;

// Atualiza velocidades a partir do deslocamento medido
if (dt > 1e-3) {
est.vx = (cam_obs->x - last.x) / dt;
est.vy = (cam_obs->y - last.y) / dt;
est.vz = (cam_obs->z - last.z) / dt;
}
}

est.t_ms = duration_cast<milliseconds>(
steady_clock::now().time_since_epoch()).count();

est.valid = true;
return est;
}

// ============== Controle (PI plano XY) ================
// Gera comandos de motor/servo a partir do estado e referência
void run_control(const HovercraftState& est, const TrajectoryRef& ref,
    MQTTClient& mqtt)
{
static double int_ex = 0.0, int_ey = 0.0;
constexpr double dt = 0.02;

// Erro de posição
double ex = ref.x_ref - est.x;
double ey = ref.y_ref - est.y;

// Integradores
int_ex += ex * dt;
int_ey += ey * dt;

// Ganhos (ajusta depois na prática)
constexpr double Kp = 2.0;
constexpr double Ki = 0.2;

double ax_des = Kp * ex + Ki * int_ex;
double ay_des = Kp * ey + Ki * int_ey;

// Erro de "aceleração"
double e_ax = ax_des - est.ax;
double e_ay = ay_des - est.ay;

double acc_cmd = std::sqrt(e_ax * e_ax + e_ay * e_ay);

// Limita módulo de "aceleração" desejada
constexpr double ACC_MAX = 1.0; // ajusta na mão depois
if (acc_cmd > ACC_MAX) {
double scale = ACC_MAX / acc_cmd;
e_ax *= scale;
e_ay *= scale;
acc_cmd = ACC_MAX;
}

// Direção da força no plano
double dir_rad = std::atan2(e_ay, e_ax);

// =============== Mapeamento para motor (ESC) =================
// 1200us + ganho proporcional à "aceleração"
uint32_t motor_us = 1200u + static_cast<uint32_t>(acc_cmd * 500.0);
motor_us = std::clamp(motor_us, 1000u, 2000u);

// =============== Mapeamento para servo =======================
// Queremos ±45° em relação ao neutro (1500us)
constexpr double SERVO_MAX_DEG  = 45.0;
constexpr double SERVO_MAX_RAD  = SERVO_MAX_DEG * M_PI / 180.0;

// Limita a direção a ±45° para não ficar louco
double dir_clamped = std::clamp(dir_rad, -SERVO_MAX_RAD, SERVO_MAX_RAD);

// 1000–2000us ≈ ±90°. Então ±45° ≈ ±250us
constexpr double SERVO_US_PER_45DEG = 250.0;

int servo_raw = 1500 + static_cast<int>(
SERVO_US_PER_45DEG * (dir_clamped / SERVO_MAX_RAD)
);

uint32_t servo_us = static_cast<uint32_t>(
std::clamp(servo_raw, 1100, 1900) // deixa uma folga das extremidades
);

mqtt.publish_cmd(motor_us, servo_us);
}

// ================== Critério de término da trajetória ==================
bool trajectory_finished(const HovercraftState& est, const TrajectoryRef& ref) {
    double ex = ref.x_ref - est.x;
    double ey = ref.y_ref - est.y;
    double dist = std::sqrt(ex * ex + ey * ey);
    return dist < 20.0; // tolerância em mm
}

// =============== MAIN (pipeline multi-thread) =======================
int main() {
    mosqpp::lib_init();
    MQTTClient mqtt("hc_host", "broker.hivemq.com", 1883);

    std::atomic<bool> running{true};
    std::atomic<Mode> g_mode{Mode::AUTONOMOUS};
    std::atomic<bool> sensor_failure{false};

    std::atomic<uint32_t> teleop_motor_us{1000};
    std::atomic<uint32_t> teleop_servo_us{1500};

    std::mutex cam_mtx;
    std::optional<HovercraftState> last_cam_obs;
    bool new_cam_obs = false;

    std::mutex est_mtx;
    HovercraftState est_state{};

    const int W = 640, H = 480;
    constexpr double FOVX_DEG = 62.0;
    double fovx_rad = FOVX_DEG * CV_PI / 180.0;
    double fx = (W * 0.5) / std::tan(fovx_rad * 0.5);
    double fy = fx;
    double cx = W * 0.5;
    double cy = H * 0.5;
    constexpr double OBJ_REAL_MM = 100.0;
    cv::Scalar HSV_LOW(10, 150, 150), HSV_HIGH(25, 255, 255);

    std::cout << "==================== HOVERCRAFT PIPELINE ====================\n";
    std::cout << "  1 = AUTONOMOUS\n";
    std::cout << "  2 = TELEOP\n";
    std::cout << "  w/s = motor +/-\n";
    std::cout << "  q/e = servo esquerda/direita\n";
    std::cout << "  x   = sair do TELEOP -> AUTONOMOUS\n";
    std::cout << "  Ctrl+C = encerrar programa\n";
    std::cout << "=============================================================\n";

    log_state_change(SystemState::SISTEMA_LIGADO, "Programa iniciado");

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

    // Thread de visão
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

    // Thread de filtro / EKF + detecção de falha de sensor
    std::thread filter_thread([&](){
        HovercraftState local_est{};
        const auto period = std::chrono::milliseconds(20);
    
        auto last_imu_time = steady_clock::now();
        auto last_cam_time = steady_clock::now();
    
        const auto sensor_timeout = std::chrono::milliseconds(1000);
        const auto cam_soft_timeout  = std::chrono::milliseconds(500);  // fase de recuperação
        const auto cam_hard_timeout  = std::chrono::milliseconds(3000); // desistir de vez
    
        bool vision_failed = false;
    
        // Para calcular dt do EKF
        auto last_step_time = steady_clock::now();
    
        while (running.load()) {
            auto now = steady_clock::now();
    
            // dt para o EKF
            double dt = std::chrono::duration_cast<std::chrono::duration<double>>(
                            now - last_step_time).count();
            last_step_time = now;
    
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
    
            std::optional<HovercraftState> cam_obs_copy;
            {
                std::lock_guard<std::mutex> lk(cam_mtx);
                if (new_cam_obs) {
                    cam_obs_copy = last_cam_obs;
                    new_cam_obs = false;
                    last_cam_time = now;
    
                    // Se a visão estava em falha, marca recuperação
                    if (vision_failed) {
                        vision_failed = false;
    
                        StatusEvent st{ "Visao recuperada" };
                        BufferMessage msg;
                        msg.type     = MsgType::STATUS_EVENT;
                        msg.priority = 7;
                        msg.payload  = st;
                        g_buffer.push(msg);
    
                        log_state_change(SystemState::EM_TRAJETO,
                                         "Visao recuperada");
                    }
                }
            }
    
            if (has_imu) {
                last_imu_time = now;
    
                // Se a IMU falhou antes e agora voltou, você pode
                // limpar o sensor_failure aqui se quiser:
                if (sensor_failure.load()) {
                    sensor_failure.store(false);
                    StatusEvent st{ "IMU recuperada" };
                    BufferMessage msg;
                    msg.type     = MsgType::STATUS_EVENT;
                    msg.priority = 8;
                    msg.payload  = st;
                    g_buffer.push(msg);
    
                    log_state_change(SystemState::EM_TRAJETO,
                                     "IMU recuperada");
                }
    
                // EKF com dt real
                local_est = run_ekf(local_est, cam_obs_copy, imu, dt);
    
                {
                    std::lock_guard<std::mutex> lk(est_mtx);
                    est_state = local_est;
                }
    
                mqtt.publish_pose(local_est);
    
                BufferMessage msg;
                msg.type     = MsgType::EKF_STATE;
                msg.priority = 4;
                msg.payload  = local_est;
                g_buffer.push(msg);
            }
    
            // --------- Falha de IMU (mantém sua lógica) ---------------
            if (!sensor_failure.load() &&
                now - last_imu_time > sensor_timeout) {
    
                sensor_failure.store(true);
                mqtt.publish_cmd(1000, 1500);
                log_state_change(SystemState::FALHA_SENSORES,
                                 "Sem dados de IMU por > 1s");
    
                StatusEvent st{ "Falha nos sensores (IMU)" };
                BufferMessage msg;
                msg.type     = MsgType::STATUS_EVENT;
                msg.priority = 8;
                msg.payload  = st;
                g_buffer.push(msg);
            }
    
            // --------- Rotina de RECUPERAÇÃO da câmera ----------------
            auto dt_cam = now - last_cam_time;
    
            // Fase "soft": sem medição recente, mas ainda tentando seguir
            // (deixa o EKF rodar com modelo de velocidade constante)
            if (!vision_failed && dt_cam > cam_soft_timeout &&
                dt_cam <= cam_hard_timeout) {
    
                StatusEvent st{ "Visao temporariamente perdida, tentando recuperar" };
                BufferMessage msg;
                msg.type     = MsgType::STATUS_EVENT;
                msg.priority = 5;
                msg.payload  = st;
                g_buffer.push(msg);
    
                // Aqui você pode também diminuir motor para evitar fugir do FOV:
                mqtt.publish_cmd(1100, 1500); // aceleração bem baixa, servo central
            }
    
            // Fase "hard": perdeu por muito tempo
            if (!vision_failed && dt_cam > cam_hard_timeout) {
                vision_failed = true;
    
                mqtt.publish_cmd(1000, 1500); // para o barco
                StatusEvent st{ "Falha na camera (timeout)" };
                BufferMessage msg;
                msg.type     = MsgType::STATUS_EVENT;
                msg.priority = 8;
                msg.payload  = st;
                g_buffer.push(msg);
    
                log_state_change(SystemState::FALHA_SENSORES,
                                 "Camera perdeu o alvo por muito tempo");
            }
    
            std::this_thread::sleep_for(period);
        }
    });

    // Thread de controle (consumidor do buffer)
    std::thread control_thread([&](){
        TrajectoryRef active_ref{};
        bool has_traj = false;
        HovercraftState last_est{};
        TeleopCmd last_teleop{};

        while (running.load()) {
            BufferMessage msg = g_buffer.pop();

            switch (msg.type) {
            case MsgType::STATUS_EVENT: {
                const auto& st = std::get<StatusEvent>(msg.payload);
                std::cout << "[STATE] " << st.text << "\n";

                if (st.text.find("Falha nos sensores") != std::string::npos) {
                    mqtt.publish_cmd(1000, 1500);
                }
                break;
            }

            case MsgType::EKF_STATE: {
                last_est = std::get<HovercraftState>(msg.payload);
                if (g_mode.load() == Mode::AUTONOMOUS && has_traj) {
                    run_control(last_est, active_ref, mqtt);
                    if (trajectory_finished(last_est, active_ref)) {
                        has_traj = false;
                        StatusEvent st{ "Trajetória concluída" };
                        BufferMessage done_msg;
                        done_msg.type     = MsgType::STATUS_EVENT;
                        done_msg.priority = 7;
                        done_msg.payload  = st;
                        g_buffer.push(done_msg);
                    }
                }
                break;
            }

            case MsgType::TRAJECTORY: {
                active_ref = std::get<TrajectoryRef>(msg.payload);
                has_traj   = true;
                std::cout << "[CTRL] Nova trajetória: x_ref="
                          << active_ref.x_ref << ", y_ref=" << active_ref.y_ref << "\n";

                StatusEvent st{ "Trajetória recebida" };
                BufferMessage st_msg;
                st_msg.type     = MsgType::STATUS_EVENT;
                st_msg.priority = 7;
                st_msg.payload  = st;
                g_buffer.push(st_msg);

                log_state_change(SystemState::TRAJETORIA_RECEBIDA);
                break;
            }

            case MsgType::TELEOP_CMD: {
                last_teleop = std::get<TeleopCmd>(msg.payload);
                if (g_mode.load() == Mode::TELEOP) {
                    mqtt.publish_cmd(last_teleop.motor_us, last_teleop.servo_us);
                }
                break;
            }
            }
        }
    });

        // Thread de teclado / teleop (modo raw, sem ENTER)
        std::thread keyboard_thread([&](){
            TerminalRawGuard guard;
    
            std::cout << "[KEY] Controle em tempo real ativo. 'x' sai do TELEOP.\n";
    
            while (running.load()) {
                int ch = getchar();
                if (ch == EOF) break;
    
                char c = static_cast<char>(ch);
    
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
    
                if ((c == 'x' || c == 'X') && mode == Mode::TELEOP) {
                    teleop_motor_us.store(1000);
                    teleop_servo_us.store(1500);
                    mqtt.publish_cmd(1000, 1500);
                    g_mode.store(Mode::AUTONOMOUS);
                    log_state_change(SystemState::AUTONOMO_ATIVADO,
                                     "Saida do teleop (x)");
                    std::cout << "\n[MODE] Saindo do TELEOP -> AUTONOMOUS.\n";
                    continue;
                }
    
                // IMPORTANTE: teleop continua funcionando mesmo com falha de IMU
                if (mode == Mode::TELEOP) {
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
    
                    TeleopCmd cmd;
                    cmd.motor_us = teleop_motor_us.load();
                    cmd.servo_us = teleop_servo_us.load();
    
                    BufferMessage msg;
                    msg.type     = MsgType::TELEOP_CMD;
                    msg.priority = 6;
                    msg.payload  = cmd;
                    g_buffer.push(msg);
    
                    std::cout << "\r[TELEOP] motor_us=" << teleop_motor_us.load()
                              << " servo_us=" << teleop_servo_us.load()
                              << "           " << std::flush;
                }
            }
        });
    

    if (keyboard_thread.joinable()) keyboard_thread.join();

    running.store(false);
    if (image_thread.joinable())   image_thread.join();
    if (filter_thread.joinable())  filter_thread.join();
    if (control_thread.joinable()) control_thread.join();

    mosqpp::lib_cleanup();
    return 0;
}

