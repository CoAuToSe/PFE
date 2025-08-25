#include <cmath>
#include <limits>
#include <initializer_list>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <tello_msgs/msg/flight_data.hpp>
#include <tello_msgs/msg/tello_position.hpp>

// ======================= Helpers génériques =======================

inline double qNaN() { return std::numeric_limits<double>::quiet_NaN(); }

// NaN si non fini
inline double nan_if_not_finite(double v) {
  return std::isfinite(v) ? v : qNaN();
}

// NaN si hors bornes [lo, hi] (et aussi si non fini)
inline double bounded_or_nan(double v, double lo, double hi) {
  if (!std::isfinite(v)) return qNaN();
  return (v < lo || v > hi) ? qNaN() : v;
}

// NaN si |v| > max_abs
inline double nan_if_abs_gt(double v, double max_abs) {
  if (!std::isfinite(v)) return qNaN();
  return (std::fabs(v) > max_abs) ? qNaN() : v;
}

// Moyenne pondérée qui ignore les NaN
struct Meas { double v; double w; };
inline double wavg_ignore_nan(std::initializer_list<Meas> ms) {
  double num = 0.0, den = 0.0;
  for (const auto& m : ms) {
    if (std::isfinite(m.v) && m.w > 0.0) { num += m.w * m.v; den += m.w; }
  }
  return den > 0.0 ? num / den : qNaN();
}

// Kalman 1D (état = vitesse), skip update si z = NaN
struct KF1D {
  double x  = 0.0;  // vitesse estimée (m/s)
  double P  = 1.0;  // variance d'état
  double R  = 1.0;  // variance de mesure (bruit vitesse)
  double qa = 0.1;  // variance du bruit d'accélération (process)

  void predict(double a, double dt){
    // modèle : v_{k|k-1} = v_{k-1|k-1} + a*dt ; P += qa*dt^2
    if (std::isfinite(a)) x += a * dt;
    P += qa * dt * dt;
  }
  void update(double z){
    if (!std::isfinite(z)) return;  // pas de mesure
    const double S = P + R;
    const double K = P / S;
    x += K * (z - x);
    P  = (1.0 - K) * P;
  }
};

// ======================= Node =======================

class TelloPosition : public rclcpp::Node {
 public:
  explicit TelloPosition(const rclcpp::NodeOptions& options)
  : Node("tello_position", options),
    x_(0.0), y_(0.0), z_(0.0),
    last_time_(this->now()),
    initial_barometer_(0.0),
    initial_barometer_set_(false)
  {
    subscription_ = this->create_subscription<tello_msgs::msg::FlightData>(
      "flight_data", 10,
      std::bind(&TelloPosition::flight_data_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<tello_msgs::msg::TelloPosition>(
      "tello_position", 10);

    this->declare_parameter("initial_x", 0.0);
    this->declare_parameter("initial_y", 0.0);
    this->declare_parameter("initial_z", 0.0);
    this->get_parameter("initial_x", x_);
    this->get_parameter("initial_y", y_);
    this->get_parameter("initial_z", z_);

    RCLCPP_INFO(this->get_logger(), "Initial position: x=%.2f y=%.2f z=%.2f", x_, y_, z_);

    std::cout << "\n" << "tello_position\n"
              << "-----------------------------------------\n"
              << "Node name: " << this->get_name() << "\n"
              << "Node namespace: " << this->get_namespace() << "\n"
              << "-----------------------------------------\n\n";
  }

 private:
  void flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg) {
    // ====== Δt sécurisé ======
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0.0) dt = 1e-3;

    // ====== Baro: initialisation & hauteur relative ======
    if (!initial_barometer_set_) {
      if (msg->baro != 0.0 && std::isfinite(msg->baro)) {
        initial_barometer_ = msg->baro;
        initial_barometer_set_ = true;
      }
    }

    // ====== Angles en degrés → sanitation → radians ======
    const double yaw_deg   = bounded_or_nan(msg->yaw,   -180.0, 180.0);
    const double pitch_deg = bounded_or_nan(msg->pitch,  -90.0,   90.0);
    const double roll_deg  = bounded_or_nan(msg->roll,   -90.0,   90.0);

    static double yaw = 0.0, pitch = 0.0, roll = 0.0; // garde la dernière valeur valide
    if (std::isfinite(yaw_deg))   yaw   = yaw_deg   * M_PI / 180.0;
    if (std::isfinite(pitch_deg)) pitch = pitch_deg * M_PI / 180.0;
    if (std::isfinite(roll_deg))  roll  = roll_deg  * M_PI / 180.0;

    // ====== Vitesses corps (cm/s) bornées ======
    const double vgx_cm = bounded_or_nan(msg->vgx, -1000.0, 1000.0);
    const double vgy_cm = bounded_or_nan(msg->vgy, -1000.0, 1000.0);
    const double vgz_cm = bounded_or_nan(msg->vgz, -1000.0, 1000.0);

    // ====== Accélérations corps (cm/s²) bornées (≈ ±16 g) ======
    const double agx_cm = bounded_or_nan(msg->agx, -16000.0, 16000.0);
    const double agy_cm = bounded_or_nan(msg->agy, -16000.0, 16000.0);
    const double agz_cm = bounded_or_nan(msg->agz, -16000.0, 16000.0);

    // shortcuts pour gérer NaN dans produits/sommes
    auto mul = [](double a, double b){ return (std::isfinite(a) && std::isfinite(b)) ? a*b : qNaN(); };
    auto add3 = [](double a,double b,double c){
      const bool A=std::isfinite(a), B=std::isfinite(b), C=std::isfinite(c);
      if (!A && !B && !C) return qNaN();
      return (A?a:0.0) + (B?b:0.0) + (C?c:0.0);
    };

    // ====== Vitesses monde (cm/s) ======
    double vx_natural_cm = add3(
      mul(vgx_cm, (std::cos(yaw) * std::cos(pitch))),
      mul(vgy_cm, (-std::sin(yaw) * std::cos(roll) + std::cos(yaw) * std::sin(pitch) * std::sin(roll))),
      mul(vgz_cm, ( std::sin(yaw) * std::sin(roll) + std::cos(yaw) * std::sin(pitch) * std::cos(roll)))
    );
    double vy_natural_cm = add3(
      mul(vgx_cm, (std::sin(yaw) * std::cos(pitch))),
      mul(vgy_cm, (std::cos(yaw) * std::cos(roll) + std::sin(yaw) * std::sin(pitch) * std::sin(roll))),
      mul(vgz_cm, (-std::cos(yaw) * std::sin(roll) + std::sin(yaw) * std::sin(pitch) * std::cos(roll)))
    );
    double vz_natural_cm = add3(
      mul(vgx_cm, (-std::sin(pitch))),
      mul(vgy_cm, ( std::cos(pitch) * std::sin(roll))),
      mul(vgz_cm, ( std::cos(pitch) * std::cos(roll)))
    );

    // ====== Accélérations monde (cm/s²) ======
    double ax_natural_cm = add3(
      mul(agx_cm, (std::cos(yaw) * std::cos(pitch))),
      mul(agy_cm, (-std::sin(yaw) * std::cos(roll) + std::cos(yaw) * std::sin(pitch) * std::sin(roll))),
      mul(agz_cm, ( std::sin(yaw) * std::sin(roll) + std::cos(yaw) * std::sin(pitch) * std::cos(roll)))
    );
    double ay_natural_cm = add3(
      mul(agx_cm, (std::sin(yaw) * std::cos(pitch))),
      mul(agy_cm, (std::cos(yaw) * std::cos(roll) + std::sin(yaw) * std::sin(pitch) * std::sin(roll))),
      mul(agz_cm, (-std::cos(yaw) * std::sin(roll) + std::sin(yaw) * std::sin(pitch) * std::cos(roll)))
    );
    double az_natural_cm = add3(
      mul(agx_cm, (-std::sin(pitch))),
      mul(agy_cm, ( std::cos(pitch) * std::sin(roll))),
      mul(agz_cm, ( std::cos(pitch) * std::cos(roll)))
    );

    // ====== Passage en SI ======
    const double vx_meas = std::isfinite(vx_natural_cm) ? vx_natural_cm / 100.0 : qNaN(); // m/s
    const double vy_meas = std::isfinite(vy_natural_cm) ? vy_natural_cm / 100.0 : qNaN(); // m/s
    const double vz_meas = std::isfinite(vz_natural_cm) ? vz_natural_cm / 100.0 : qNaN(); // m/s

    const double ax_si = std::isfinite(ax_natural_cm) ? ax_natural_cm / 100.0 : qNaN(); // m/s²
    const double ay_si = std::isfinite(ay_natural_cm) ? ay_natural_cm / 100.0 : qNaN();
    const double az_si = std::isfinite(az_natural_cm) ? az_natural_cm / 100.0 : qNaN();

    // NOTE gravité : si tes ag* incluent 1g, soustrais 9.80665 après rotation
    // ex: double az_si_corr = std::isfinite(az_si) ? (az_si - 9.80665) : qNaN();
    // ici on garde az_si tel quel (à ajuster selon ton capteur)

    // ====== Kalman 1D sur vitesses (skip NaN) ======
    kf_vx_.predict(std::isfinite(ax_si) ? ax_si : 0.0, dt);
    kf_vx_.update(vx_meas);

    kf_vy_.predict(std::isfinite(ay_si) ? ay_si : 0.0, dt);
    kf_vy_.update(vy_meas);

    kf_vz_.predict(std::isfinite(az_si) ? az_si : 0.0, dt);
    kf_vz_.update(vz_meas);

    // ====== Intégration position (prior), corrigée ensuite par l'altitude ======
    x_ += kf_vx_.x * dt;
    y_ += kf_vy_.x * dt;
    z_ += kf_vz_.x * dt;

    // ====== Altitude : TOF / h / baro → mix robuste ======
    // TOF (cm → m) ; 6553 => invalide
    double height_tof = qNaN();
    if (msg->tof != 6553 && std::isfinite(static_cast<double>(msg->tof))) {
      const double tof_m = msg->tof / 100.0;
      if (std::isfinite(tof_m))
        height_tof = tof_m * std::cos(pitch) * std::cos(roll);
    }

    // Altitude relative h (cm → m), bornée [0, 100] m
    const double height_rel = bounded_or_nan(msg->h / 100.0, 0.0, 100.0);

    // Baro (m) si initialisé
    double height_baro = qNaN();
    if (initial_barometer_set_ && std::isfinite(msg->baro)) {
      height_baro = msg->baro - initial_barometer_;
    }

    // Mix (pondérations à ajuster)
    double z_mix = wavg_ignore_nan({
      {height_tof, 0.4},
      {height_rel, 0.3},
      {z_,         0.2}, // prior
      {height_baro,0.1}
    });
    if (std::isfinite(z_mix)) z_ = z_mix;

    // ====== Logs concis (facultatifs) ======
    std::cout << "v_meas [m/s]  vx " << vx_meas << " vy " << vy_meas << " vz " << vz_meas << "\n";
    std::cout << "a_si   [m/s2] ax " << ax_si   << " ay " << ay_si   << " az " << az_si   << "\n";
    std::cout << "kf v   [m/s]  vx " << kf_vx_.x<< " vy " << kf_vy_.x<< " vz " << kf_vz_.x<< "\n";
    std::cout << "pos [m] x " << x_ << " y " << y_ << " z " << z_
              << " | h_tof " << height_tof << " h_rel " << height_rel << " h_baro " << height_baro << "\n";

    // ====== Publication ======
    auto position_msg = tello_msgs::msg::TelloPosition();
    position_msg.x = x_;
    position_msg.y = y_;
    position_msg.z = z_;
    position_msg.pitch = std::isfinite(pitch_deg) ? pitch_deg : 0.0;
    position_msg.roll  = std::isfinite(roll_deg)  ? roll_deg  : 0.0;
    position_msg.yaw   = std::isfinite(yaw_deg)   ? yaw_deg   : 0.0;
    publisher_->publish(position_msg);
  }

  // --- members ---
  rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr   subscription_;
  rclcpp::Publisher<tello_msgs::msg::TelloPosition>::SharedPtr   publisher_;
  double x_, y_, z_;
  rclcpp::Time last_time_;

  double initial_barometer_;
  bool   initial_barometer_set_;

  KF1D kf_vx_, kf_vy_, kf_vz_;
};

// ======================= main =======================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<TelloPosition>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
