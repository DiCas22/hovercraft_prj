#include <iostream>
#include <mutex>
#include <deque>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// ==================== Estruturas ===================
struct Pose2D {
    double x_mm = 0.0;  // posição em mm (mesmo sistema da trajetória)
    double y_mm = 0.0;
    uint64_t t_ms = 0;
    bool valid = false;
};

// ==================== MQTT Client ==================
class TrajPlotMQTTClient : public mosqpp::mosquittopp {
public:
    // Trajetória de referência (lista de pontos x_ref, y_ref)
    std::mutex traj_mtx;
    std::vector<cv::Point2f> traj_points_mm;   // pontos em milímetros (x_ref, y_ref)

    // Histórico de pose estimada
    std::mutex pose_mtx;
    std::deque<cv::Point2f> pose_hist_mm;      // (x_mm, y_mm)
    Pose2D last_pose{};
    bool has_pose = false;

    // Tamanho máximo do histórico
    const size_t max_pose_hist = 5000;

    TrajPlotMQTTClient(const std::string& id,
                       const std::string& host,
                       int port)
        : mosqpp::mosquittopp(id.c_str())
    {
        connect_async(host.c_str(), port, 60);
        loop_start();
    }

    ~TrajPlotMQTTClient() override {
        try {
            loop_stop(true);
        } catch (...) {
            // ignora qualquer exceção na destruição
        }
    }

    void on_connect(int rc) override {
        if (rc == 0) {
            std::cout << "[MQTT] Connected (traj_viewer). Subscribing...\n";
            subscribe(nullptr, "hovercraft/traj");
            subscribe(nullptr, "hovercraft/ekfstate"); // pose estimada publicada pelo pipeline
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

            if (topic == "hovercraft/traj") {
                handle_traj_message(j);
            } else if (topic == "hovercraft/ekfstate") {
                handle_pose_message(j);
            }
        } catch (const std::exception& e) {
            std::cerr << "[MQTT] JSON parse error on topic " << topic
                      << ": " << e.what() << "\n";
        }
    }

private:
    void handle_traj_message(const json& j) {
        auto add_point = [](const json& jj) -> cv::Point2f {
            double xr = jj.value("x_ref", 0.0);
            double yr = jj.value("y_ref", 0.0);
            return cv::Point2f(static_cast<float>(xr),
                               static_cast<float>(yr));
        };

        std::lock_guard<std::mutex> lk(traj_mtx);

        // Estratégia: se vier um array, consideramos que é uma nova
        // trajetória completa → limpamos e substituímos.
        // Se vier objeto único, apenas adicionamos ao final.
        if (j.is_array()) {
            traj_points_mm.clear();
            for (const auto& elem : j) {
                if (!elem.is_object()) continue;
                traj_points_mm.push_back(add_point(elem));
            }
            std::cout << "[TRAJ_VIEW] Nova trajetória recebida: "
                      << traj_points_mm.size() << " pontos.\n";
        } else if (j.is_object()) {
            traj_points_mm.push_back(add_point(j));
            std::cout << "[TRAJ_VIEW] Ponto de trajetória adicionado. Total: "
                      << traj_points_mm.size() << "\n";
        } else {
            std::cerr << "[TRAJ_VIEW] JSON inválido em hovercraft/traj (nem array, nem objeto).\n";
        }
    }

    void handle_pose_message(const json& j) {
        Pose2D pose;
        pose.x_mm = j.value("x", 0.0);
        pose.y_mm = j.value("y", 0.0);
        pose.t_ms = j.value("t_ms", 0ull);
        pose.valid = true;

        std::lock_guard<std::mutex> lk(pose_mtx);

        last_pose = pose;
        has_pose = true;

        pose_hist_mm.emplace_back(
            static_cast<float>(pose.x_mm),
            static_cast<float>(pose.y_mm)
        );
        if (pose_hist_mm.size() > max_pose_hist) {
            pose_hist_mm.pop_front();
        }
    }
};

// ==================== MAIN =========================
int main() {
    // Inicializa biblioteca MQTT
    mosqpp::lib_init();

    // Ajuste host/porta conforme usado no pipeline
    TrajPlotMQTTClient mqtt("hc_traj_viewer", "broker.hivemq.com", 1883);

    const int win_size = 800;
    cv::namedWindow("Hovercraft Trajectory", cv::WINDOW_AUTOSIZE);

    // Escala: px por mm (ajuste conforme área típica de operação)
    const float scale = 0.1f; // 0.1 px / mm => 10 px por 100 mm
    const cv::Point2f center(win_size / 2.0f, win_size / 2.0f);

    std::cout << "==============================================\n";
    std::cout << "   Hovercraft Trajectory Viewer (host_tools)\n";
    std::cout << " - Assina hovercraft/traj e hovercraft/ekfstate\n";
    std::cout << " - ESC para sair\n";
    std::cout << "==============================================\n";

    while (true) {
        // Copia dados sob mutex para não travar threads do MQTT
        std::vector<cv::Point2f> traj_copy;
        std::deque<cv::Point2f> pose_hist_copy;
        Pose2D last_pose_copy;
        bool has_pose_copy = false;

        {
            std::lock_guard<std::mutex> lk(mqtt.traj_mtx);
            traj_copy = mqtt.traj_points_mm;
        }

        {
            std::lock_guard<std::mutex> lk(mqtt.pose_mtx);
            pose_hist_copy = mqtt.pose_hist_mm;
            last_pose_copy = mqtt.last_pose;
            has_pose_copy = mqtt.has_pose;
        }

        // Cria fundo
        cv::Mat img(win_size, win_size, CV_8UC3, cv::Scalar(30, 30, 30));

        // Eixos
        cv::line(img,
                 cv::Point(0, (int)center.y),
                 cv::Point(win_size, (int)center.y),
                 cv::Scalar(80, 80, 80), 1);
        cv::line(img,
                 cv::Point((int)center.x, 0),
                 cv::Point((int)center.x, win_size),
                 cv::Scalar(80, 80, 80), 1);

        // Desenha trajetória planejada (azul claro)
        if (traj_copy.size() >= 2) {
            for (size_t i = 1; i < traj_copy.size(); ++i) {
                cv::Point p1(
                    static_cast<int>(center.x + traj_copy[i - 1].x * scale),
                    static_cast<int>(center.y - traj_copy[i - 1].y * scale)
                );
                cv::Point p2(
                    static_cast<int>(center.x + traj_copy[i].x * scale),
                    static_cast<int>(center.y - traj_copy[i].y * scale)
                );
                cv::line(img, p1, p2, cv::Scalar(255, 255, 0), 2);
            }

            // Marca início e fim da trajetória
            cv::Point p_start(
                static_cast<int>(center.x + traj_copy.front().x * scale),
                static_cast<int>(center.y - traj_copy.front().y * scale)
            );
            cv::Point p_end(
                static_cast<int>(center.x + traj_copy.back().x * scale),
                static_cast<int>(center.y - traj_copy.back().y * scale)
            );
            cv::circle(img, p_start, 6, cv::Scalar(0, 255, 255), -1); // início
            cv::circle(img, p_end, 6, cv::Scalar(255, 0, 255), -1);   // fim
        }

        // Desenha trilha da pose (amarelo)
        if (pose_hist_copy.size() >= 2) {
            for (size_t i = 1; i < pose_hist_copy.size(); ++i) {
                cv::Point p1(
                    static_cast<int>(center.x + pose_hist_copy[i - 1].x * scale),
                    static_cast<int>(center.y - pose_hist_copy[i - 1].y * scale)
                );
                cv::Point p2(
                    static_cast<int>(center.x + pose_hist_copy[i].x * scale),
                    static_cast<int>(center.y - pose_hist_copy[i].y * scale)
                );
                cv::line(img, p1, p2, cv::Scalar(0, 255, 255), 2);
            }
        }

        // Posição atual (vermelho)
        if (has_pose_copy && last_pose_copy.valid) {
            cv::Point p_cur(
                static_cast<int>(center.x + last_pose_copy.x_mm * scale),
                static_cast<int>(center.y - last_pose_copy.y_mm * scale)
            );
            cv::circle(img, p_cur, 7, cv::Scalar(0, 0, 255), -1);

            // Pequeno texto com coordenadas
            char buf[128];
            std::snprintf(buf, sizeof(buf), "x=%.1f mm, y=%.1f mm",
                          last_pose_copy.x_mm, last_pose_copy.y_mm);
            cv::putText(img, buf, cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
        } else {
            cv::putText(img, "Aguardando pose em hovercraft/ekfstate...",
                        cv::Point(10, 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(100, 100, 100), 1, cv::LINE_AA);
        }

        // Info de ajuda
        cv::putText(img, "Trajeto: hovercraft/traj | Pose: hovercraft/ekfstate",
                    cv::Point(10, win_size - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(150, 150, 150), 1, cv::LINE_AA);

        cv::imshow("Hovercraft Trajectory", img);
        int key = cv::waitKey(30);
        if (key == 27) { // ESC
            std::cout << "[TRAJ_VIEW] ESC detectado. Encerrando...\n";
            break;
        }
    }

    mqtt.disconnect();
    mosqpp::lib_cleanup();
    return 0;
}
