#include <climits>
#include <iostream>
#include <fstream>
#include <vector>

#define DEBUG

const size_t MAX_ITERATIONS = INT_MAX;

const double earth_m = 6e24;
const double earth_r = 6.357e6;

void log(size_t time, std::string message) {
    #ifdef DEBUG
    std::ofstream file;
    file.open("log.txt", std::ios::app);
    file << "[" << time << "] ";
    file << message << std::endl;
    file.close();
    #endif // DEBUG
}

class CheckPoint {
public:
    size_t time;
    double vx, vy;
    bool finish;

    CheckPoint(size_t _time, double _vx, double _vy, bool _finish) {
        time = _time;
        vx = _vx;
        vy = _vy;
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
    Orbit target; // Параметры целевой орбиты

    std::vector<CheckPoint> points;

    // Рассчет ключевых точек маршрута
    void calculate_script() {
        log(time, "Calculating script");
        points.push_back(CheckPoint(1, 0.0, 0.0, true));
    }

    bool check_point() {
        double res_x = 0.0, res_y = 0.0;

        log(time, "Checking current position");
        for (int i = 0; i < points.size(); ++i) {
            CheckPoint point = points[i];

            // Если мы находимся в контрольной точке, то делаем импульс
            if (point.time == time) {
                res_x = point.vx;
                res_y = point.vy;

                // Если контрольная точка финишная, то даем сигнал завершить программу
                return point.finish;
            }
        }

        made_impulse(res_x, res_y);

        // Если не находимся ни в одной контрольной точке, то ничего не делаем
        return false;
    }

    void made_impulse(double vx, double vy) {
        log(time, "Made impulse");
        std::cout << vx << std::endl;
        std::cout << vy << std::endl;
    }
};


int main(int argc, char **argv) {
    #ifdef DEBUG
    // Clear log file
    std::ofstream file("log.txt", std::ios::trunc);
    file.close();
    #endif // DEBUG

    // Ввод параметров целевой орбиты
    Orbit target;
    std::cin >> target.a;
    std::cin >> target.e;
    std::cin >> target.tilt;

    SatelliteState state;
    state.target = target;
    std::cin >> state.fuel;
    log(0, "Initialized state");

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        std::cin >> state.time;
        std::cin >> state.sx >> state.sy;
        std::cin >> state.vx >> state.vy;
        std::cin >> state.fuel;

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
