#ifndef MSG_TYPES_H
#define MSG_TYPES_H

#include <cstdint>
#include <string>
#include <variant>

// Estado completo do hovercraft (EKF)
struct HovercraftState {
    double x = 0, y = 0, z = 0;
    double vx = 0, vy = 0, vz = 0;
    double ax = 0, ay = 0, az = 0;
    double yaw = 0, pitch = 0, roll = 0;
    bool   valid = false;
    std::uint64_t t_ms = 0;
};

// Referência de trajetória no plano
struct TrajectoryRef {
    double x_ref = 0, y_ref = 0;
    double vx_ref = 0, vy_ref = 0;
    double ax_ref = 0, ay_ref = 0;
    std::uint64_t t_ms = 0;
};

// Evento de texto (logs, status)
struct StatusEvent {
    std::string text;
};

// Comando de teleop (PWM)
struct TeleopCmd {
    std::uint32_t motor_us = 1000;
    std::uint32_t servo_us = 1500;
};

// Tipos de mensagem do buffer
enum class MsgType {
    STATUS_EVENT,
    EKF_STATE,
    TRAJECTORY,
    TELEOP_CMD
};

// Payload genérico do buffer
using MessagePayload = std::variant<
    StatusEvent,
    HovercraftState,
    TrajectoryRef,
    TeleopCmd
>;

// Mensagem que trafega no buffer
struct BufferMessage {
    MsgType type;
    int priority = 0;
    MessagePayload payload;
};

#endif // MSG_TYPES_H
