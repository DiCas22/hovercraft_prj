// host_tools/hovercraft_stop_test.cpp
//
// Teste simples: controlar SOMENTE o motor (ESC) via MQTT,
// usando visão para frear o hovercraft a ~5 cm de uma caixa.
//
// - Barquinho: blob laranja (ajustar HSV_BOAT_*)
// - Caixa: coloque um papel de cor diferente (ex: azul) e ajuste HSV_BOX_*
// - Controle: PI em cima da distância aproximada em cm
// - motor_us varia entre 1500 (neutro/flutuando) e 1800 (correndo)

#include <iostream>
#include <cmath>
#include <chrono>
#include <string>

#include <opencv2/opencv.hpp>
#include <mosquittopp.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono;

// ====================== Config de MQTT ======================
static const char* MQTT_HOST = "broker.hivemq.com";
static const int   MQTT_PORT = 1883;
static const char* MQTT_TOPIC_CMD = "hovercraft/cmd";

// ====================== MQTT simples =======================
class SimpleMQTT : public mosqpp::mosquittopp {
public:
    SimpleMQTT(const std::string& id,
               const std::string& host,
               int port)
        : mosqpp::mosquittopp(id.c_str())
    {
        int keepalive = 60;
        connect_async(host.c_str(), port, keepalive);
        loop_start();
    }

    ~SimpleMQTT() {
        loop_stop();
        disconnect();
    }

    void on_connect(int rc) override {
        std::cout << "[MQTT] Conectado, rc=" << rc << "\n";
    }

    void send_cmd(uint32_t motor_us, uint32_t servo_us = 1500) {
        json j = {{"motor_us", motor_us}, {"servo_us", servo_us}};
        std::string payload = j.dump();

        int ret = publish(nullptr,
                          MQTT_TOPIC_CMD,
                          static_cast<int>(payload.size()),
                          payload.c_str(),
                          0,  // QoS
                          false); // retain

        if (ret != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] Erro ao publicar (ret=" << ret << ")\n";
        }
    }
};

// ====================== Detecção de blob =====================
struct BlobInfo {
    cv::Point2f center;
    double area;
    double radius;
};

bool detect_blob(const cv::Mat& frame_bgr,
                 const cv::Scalar& hsv_low,
                 const cv::Scalar& hsv_high,
                 BlobInfo& out)
{
    cv::Mat hsv;
    cv::cvtColor(frame_bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, hsv_low, hsv_high, mask);

    // Limpeza básica
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    int best_idx = -1;
    for (int i = 0; i < static_cast<int>(contours.size()); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > best_area) {
            best_area = a;
            best_idx = i;
        }
    }

    if (best_idx < 0 || best_area < 50.0) {
        return false; // nada significativo
    }

    cv::Moments mu = cv::moments(contours[best_idx]);
    if (std::abs(mu.m00) < 1e-5) return false;

    out.center = cv::Point2f(static_cast<float>(mu.m10 / mu.m00),
                             static_cast<float>(mu.m01 / mu.m00));
    out.area   = best_area;
    out.radius = std::sqrt(best_area / CV_PI);

    return true;
}

// ====================== Main ================================
int main() {
    // ---- Inicializa MQTT ----
    mosqpp::lib_init();
    SimpleMQTT mqtt("hc_stop_test", MQTT_HOST, MQTT_PORT);

    // ---- Inicializa câmera ----
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "[CAM] Nao foi possivel abrir a camera 0\n";
        mosqpp::lib_cleanup();
        return 1;
    }

    const int W = 1280;
    const int H = 920;
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  W);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, H);

    cv::namedWindow("hovercraft_stop_test", cv::WINDOW_AUTOSIZE);

    // ---- HSV: ajuste conforme suas cores reais ----
    // Barquinho laranja (valores aproximados, ajuste no teste)
    cv::Scalar HSV_BOAT_LOW(5, 120, 120);
    cv::Scalar HSV_BOAT_HIGH(25, 255, 255);

    // Verde escuro metálico
    cv::Scalar HSV_BOX_LOW (50,  60,  40);   // H, S, V mínimos
    cv::Scalar HSV_BOX_HIGH(80, 255, 200);   // H, S, V máximos

    // ---- Controle PI em cima da distancia em "cm" aproximados ----
    const double TARGET_DIST_CM = 7.0;     // alvo ~5 cm
    const double PIXELS_PER_CM  = 10.0;    // chute inicial, ajuste na mão
    const double target_dist_px = TARGET_DIST_CM * PIXELS_PER_CM;

    const double Kp = 20.0;  // ajusta ganhando experiência
    const double Ki = 2.0;

    double integ = 0.0;
    double motor_us_cmd = 1800.0; // começa "correndo"

    auto last_time = steady_clock::now();

    std::cout << "================= HOVERCRAFT STOP TEST =================\n";
    std::cout << " - ESC controlado de 1500 a 1800 us\n";
    std::cout << " - Barco: blob laranja (ajuste HSV_BOAT)\n";
    std::cout << " - Caixa: coloque algo azul (ajuste HSV_BOX)\n";
    std::cout << " - Pressione 'q' ou ESC para sair\n";
    std::cout << "========================================================\n";

    while (true) {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "[CAM] Frame vazio\n";
            break;
        }

        BlobInfo boat{}, box{};
        bool boat_ok = detect_blob(frame, HSV_BOAT_LOW, HSV_BOAT_HIGH, boat);
        bool box_ok  = detect_blob(frame, HSV_BOX_LOW,  HSV_BOX_HIGH,  box);

        auto now = steady_clock::now();
        double dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - last_time).count();
        last_time = now;
        if (dt < 0.0)   dt = 0.0;
        if (dt > 0.1)   dt = 0.1; // limite de 100 ms pra nao explodir integral

        if (boat_ok) {
            cv::circle(frame, boat.center, static_cast<int>(boat.radius),
                       cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, "BOAT",
                        boat.center + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 0), 2);
        }

        if (box_ok) {
            cv::circle(frame, box.center, static_cast<int>(box.radius),
                       cv::Scalar(255, 0, 0), 2);
            cv::putText(frame, "BOX",
                        box.center + cv::Point2f(5, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(255, 0, 0), 2);
        }

        if (boat_ok && box_ok) {
            // Distancia em pixels e "cm" aproximados
            double dist_px = cv::norm(boat.center - box.center);
            double dist_cm = dist_px / PIXELS_PER_CM;

            // Erro em cm (positivo = longe demais, negativo = perto demais)
            double error_cm = dist_cm - TARGET_DIST_CM;

            // PI discreto
            integ += error_cm * dt;
            // anti-windup simples
            const double INTEG_MAX = 100.0;
            if (integ >  INTEG_MAX) integ =  INTEG_MAX;
            if (integ < -INTEG_MAX) integ = -INTEG_MAX;

            double u = Kp * error_cm + Ki * integ;

            // Comando em torno de 1500 us
            motor_us_cmd = 1500.0 + u;

            // Saturar entre neutro e "correndo"
            if (motor_us_cmd > 1800.0) motor_us_cmd = 1800.0;
            if (motor_us_cmd < 1500.0) motor_us_cmd = 1500.0;

            // Desenho da linha e texto
            cv::line(frame, boat.center, box.center,
                     cv::Scalar(0, 255, 255), 2);

            char txt[128];
            std::snprintf(txt, sizeof(txt),
                          "d=%.1f cm  motor=%.0f us", dist_cm, motor_us_cmd);
            cv::putText(frame, txt, cv::Point(20, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(0, 255, 255), 2);

            // Print no terminal
            std::cout << "dist_cm=" << dist_cm
                      << "  error_cm=" << error_cm
                      << "  motor_us=" << motor_us_cmd
                      << std::endl;
        } else {
            // Se perder barco ou caixa -> segura em 1500 e zera integral
            integ = 0.0;
            motor_us_cmd = 1500.0;
        }

        // Envia comando de motor para a ESP
        uint32_t motor_cmd_u32 = static_cast<uint32_t>(std::lround(motor_us_cmd));
        mqtt.send_cmd(motor_cmd_u32, 1500);

        // Mostra janela
        cv::imshow("hovercraft_stop_test", frame);
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }
    }

    // Para o motor no fim
    mqtt.send_cmd(1500, 1500);

    cap.release();
    cv::destroyAllWindows();
    mosqpp::lib_cleanup();
    return 0;
}
