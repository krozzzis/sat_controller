#include <climits>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <complex>

const size_t MAX_ITERATIONS = INT_MAX;

const double earth_m = 6e24;
const double g = 6.6743e-11;
const double u = 3.986e14;

using complex = std::complex<double>;

double normalize_angle(double angle) {
    angle = fmod(angle, M_PI * 2);
    if (angle < 0)
        angle += M_PI * 2;
    return angle;
}

class CheckPoint {
public:
    size_t time;
    double force;
    bool finish;

    CheckPoint(size_t _time, double _force, bool _finish) {
        time = _time;
        force = _force;
        finish = _finish;
    }
};

class Orbit {
public:
    double e; // Эксцентриситет
    double a; // Большая полуось, м
    double tilt; // Наклон, град.
};

class SatelliteState {
public:
    size_t time; // Прошедшее время с начала моделирования до текущего момента, с
    double sx, sy; // Текущее положение спутника, м
    double vx, vy; // Текущий вектор скорости спутника, м/с
    double fuel; // Текущий запас топлива спутника

    size_t t_count; // Количество спутников-целей
    double tx, ty; // Текущее положение спутника-цели
    double tvx, tvy; // Текущий вектор скорости спутника-цели

    Orbit target; // Параметры целевой орбиты

    std::vector<CheckPoint> points; // Ключевые точки маршрута

    bool simulation(double x, double y, double vvx, double vvy, size_t t, std::vector<CheckPoint> &points, double target_angle, size_t target_time, double &new_x, double &new_y, double &new_vx, double &new_vy, size_t &new_t) {
        while (t < 30000) {
            double r = sqrt(x*x + y*y);

            double f = g * earth_m / r / r;
            double fx = -f * x / r;
            double fy = -f * y / r;

            double dvx = 0.0;
            double dvy = 0.0;

            for (int i = 0; i < points.size(); ++i) {
                if (points[i].time == t) {
                    double vvv = sqrt(vvx*vvx + vvy*vvy);
                    dvx = points[i].force * vvx / vvv;
                    dvy = points[i].force * vvy / vvv;
                }
            }

            double ax = dvx + fx;
            double ay = dvy + fy;

            double sx_new = x + vvx + 0.5 * ax;
            double sy_new = y + vvy + 0.5 * ay;

            double r_new = sqrt(sx_new * sx_new + sy_new * sy_new);
            double f_new = g * earth_m / r_new / r_new;
            double fx_new = -f_new * sx_new / r_new;
            double fy_new = -f_new * sy_new / r_new;

            double vx_new = vvx + (dvx + (fx + fx_new) / 2) * 1.0;
            double vy_new = vvy + (dvy + (fy + fy_new) / 2) * 1.0;

            x = sx_new;
            y = sy_new;
            vvx = vx_new;
            vvy = vy_new;

            t += 1;

            double angle = atan2(y, x);
            if ((angle < target_angle && target_time == -1) || (t == target_time)) {
                new_x = x;
                new_y = y;
                new_vx = vvx;
                new_vy = vvy;
                new_t = t;
                return true;
            }
        }

        return false;
    }

    // Рассчет ключевых точек маршрута
    void calculate_script() {
        double time_gap = 100000;
        double new_x, new_y, new_vx, new_vy;

        complex target_sputnik_angle(tx, ty);
        complex sputnik_angle(sx, sy);

        target.a = sqrt(tx*tx + ty*ty);
        target.e = 0.0;
        target.tilt = 0.0;

        double initial_orbit_radius = sqrt(sx*sx + sy*sy);
        double initial_orbit_speed = sqrt(g * earth_m * (2 / initial_orbit_radius - 1 / initial_orbit_radius));

        double target_orbit_radius = target.a + target.e * target.a;
        double target_orbit_speed = sqrt(g * earth_m * (2 / target_orbit_radius - 1 / target_orbit_radius));

        double vp = sqrt((initial_orbit_speed*initial_orbit_speed + target_orbit_speed*target_orbit_speed) / 2);

        double impulse_speed1 = initial_orbit_speed * (initial_orbit_speed / vp - 1);
        double impulse_speed2 = target_orbit_speed * (1 - target_orbit_speed / vp);

        // Моделирование движение основного спутника для рассчета длительности гомановского перехода
        size_t impulse2_time = 0;
        std::vector<CheckPoint> pps;
        pps.push_back(CheckPoint(0, impulse_speed1, false));
        simulation(sx, sy, vx, vy, 0, pps, -M_PI/2, -1, new_x, new_y, new_vx, new_vy, impulse2_time);

        std::vector<CheckPoint> noll;
        size_t pi_time1 = 0;
        size_t pi_time2 = 0;
        simulation(sx, sy, vx, vy, 0, noll, -M_PI/2, -1, new_x, new_y, new_vx, new_vy, pi_time1);
        simulation(0.0, target_orbit_radius, target_orbit_speed, 0.0, 0, noll, -M_PI/2, -1, new_x, new_y, new_vx, new_vy, pi_time2);

        // Угловые скорости орбит
        double v1 = initial_orbit_speed / initial_orbit_radius;
        double v2 = target_orbit_speed / target_orbit_radius;

        double vv1 = M_PI / pi_time1;
        double vv2 = M_PI / pi_time2;


        // Угловое положение спутников
        double a1 = (M_PI/2 - std::arg(sputnik_angle));
        double a2 = (M_PI/2 - std::arg(target_sputnik_angle));

        // Максимальная угловая погрешность, ~10км
        double b = 10000 / target_orbit_radius;

        // Время гомановского перехода по формуле, немного отличается от полученного через симулятор
        double x = M_PI * sqrt(pow(initial_orbit_radius + target_orbit_radius, 3) / (8 * u));

        // Моделирование необходимой задержки перед переходом
        for (int t = 0; t < 70000; t++) {
            double aa1 = normalize_angle(a1 + vv1 * t + M_PI);
            double aa2 = normalize_angle(a2 + vv2 * t + impulse2_time*vv2);
            double gg = fabs((aa1) - (aa2));
            if (gg > M_PI) {
                gg = 2 * M_PI - gg;
            }
            if (gg < b) {
                if (t < time_gap)
                    time_gap = t;
                else
                    break;
            }
        }

        size_t t = time_gap;
        size_t rb;
        for (int i = time_gap-10; i <= time_gap+10; i++) {
            std::vector<CheckPoint> r;
            r.push_back(CheckPoint(i, impulse_speed1, false));
            r.push_back(CheckPoint(impulse2_time + i, impulse_speed2, false));
            simulation(sx, sy, vx, vy, 0, r, 0, impulse2_time + i + 50, new_x, new_y, new_vx, new_vy, rb);
            double xx1 = new_x, yy1 = new_y;
            simulation(tx, ty, tvx, tvy, 0, noll, 0, impulse2_time + i + 50, new_x, new_y, new_vx, new_vy, rb);
            double xx2 = new_x, yy2 = new_y;

            double dist = sqrt((xx2 - xx1)*(xx2 - xx1) + (yy2 - yy1)*(yy2 - yy1));
            if (dist < 20000) {
                t = i;
            }
        }
        time_gap = t;

        points.push_back(CheckPoint(time_gap, impulse_speed1, false));
        points.push_back(CheckPoint(impulse2_time + time_gap, impulse_speed2, false));
        points.push_back(CheckPoint(impulse2_time + time_gap + 50, 0.0, true));
    }

    bool check_point() {
        for (int i = 0; i < points.size(); ++i) {
            CheckPoint point = points[i];

            // Если мы находимся в контрольной точке, то делаем импульс
            if (point.time == time) {
                made_impulse(point.force);

                // Если контрольная точка финишная, то даем сигнал завершить программу
                return point.finish;
            }
        }

        made_impulse(0.0);

        // Если не находимся ни в одной контрольной точке, то ничего не делаем
        return false;
    }

    void made_impulse(double force) {
        double ddd = sqrt(vx*vx + vy*vy);
        double dx = vx / ddd;
        double dy = vy / ddd;
        std::cout << dx * force << std::endl;
        std::cout << dy * force << std::endl;
    }
};


int main(int argc, char **argv) {
    SatelliteState state;
    std::cin >> state.fuel;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        std::cin >> state.time;
        std::cin >> state.sx >> state.sy;
        std::cin >> state.vx >> state.vy;
        std::cin >> state.fuel;
        std::cin >> state.t_count;
        std::cin >> state.tx >> state.ty;
        std::cin >> state.tvx >> state.tvy;

        if (i == 0) {
            state.calculate_script();
        }

        if (state.check_point()) {
            std::cout << "1" << std::endl;
            break;
        } else {
            std::cout << "0" << std::endl;
        }
    }

    return 0;
}
