#ifndef PARTICLES_HPP
#define PARTICLES_HPP

#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include "bigbang/types/rosparams.h"
#include "bigbang/utils/measurement_utils.hpp"
#include "sensor_msgs/PointCloud.h"
#include "bigbang_eurobot/LaserBeacons.h"
#include <bigbang/types/coord.hpp>
#include <bigbang/types/position.hpp>
#include "pcg/pcg_random.hpp"
#include <QtGlobal>
#include <random>
#include <vector>

struct ParticlesSettings : RosParams {
    Q_GADGET
    IS_SERIALIZABLE
    SERIAL_FIELD(int, count, 2000)
    SERIAL_FIELD(float, move_noise, 0.05)
    SERIAL_FIELD(float, turn_noise, 0.04)
    SERIAL_FIELD(float, move_stddev, 0.03)
    SERIAL_FIELD(float, theta_stddev, 0.08)
    SERIAL_FIELD(float, min_x, 0)
    SERIAL_FIELD(float, min_y, 0)
    SERIAL_FIELD(float, max_x, 2)
    SERIAL_FIELD(float, max_y, 3)
    SERIAL_FIELD(float, resample_move, 0.1)
    SERIAL_FIELD(int, noises_count, 5000)
    SERIAL_FIELD(int, recalc_beta_each, 100)
    SERIAL_FIELD(float, resample_turn, 0.3)
    SERIAL_FIELD(float, max_disperce, 0.55)
    SERIAL_FIELD(float, resample_min_divider, 0.3)
    SERIAL_FIELD(float, weight_compensation, 0.02)
    SERIAL_FIELD(float, start_rotation, 0)
};

struct Particle : public PositionF {
    using PositionF::PositionF;
    float weight{0};
};

struct Particles {
    Particles(const ParticlesSettings& settings) :
        m_settings(settings),
        m_currentDisperce(settings.max_disperce),
        engine(std::random_device()()),
        normal_move(0, settings.move_noise),
        normal_turn(0, settings.turn_noise),
        uniform_sample(0, settings.noises_count),
        uniform_x(settings.min_x, settings.max_x),
        uniform_y(settings.min_y, settings.max_y),
        uniform_th{toRadians(-180 + settings.start_rotation), toRadians(180 + settings.start_rotation)}
    {
        m_particles.resize(settings.count);
        m_particles2.resize(settings.count);
        m_move_noises.reserve(settings.noises_count);
        for (int i = 0; i < settings.noises_count; ++i) {
            m_move_noises.push_back(PositionF{normal_move(engine),
                                               normal_move(engine),
                                               normal_turn(engine)});
        }
        randomizeParticles();
    }
    void setPosition(const PositionF& pos) noexcept {
        m_currentMedian  = pos;
        m_currentDisperce = 0;
        for (auto &subPos : m_particles) {
            subPos = pos;
        }
    }
    void applyMove(const PositionF &deltaPos) noexcept {
        if (!deltaPos.norm()) return;
        m_currentDisperce = 0;
        auto lastMedian = m_currentMedian;
        m_currentMedian = {};
        for (auto &pos : m_particles) {
            pos.apply_delta(deltaPos + noiseMove(deltaPos));
            m_currentMedian += pos;
            m_currentDisperce += disperceFrom(lastMedian, pos);
        }
        m_currentMedian/=static_cast<float>(m_particles.size());
        m_currentDisperce/=static_cast<float>(m_particles.size());
    }
    void applyMeasure(const bigbang_eurobot::LaserBeaconsConstPtr &landmarks) noexcept
    {
        // if landmarks are empty resample() is called still (uncertainty grows)
        m_currentWeightSum = 0;
        auto wasDensity = density();
        float noiseAmp = 1.f / std::max(wasDensity, m_settings.resample_min_divider);
        for (auto &pos : m_particles) {
            pos.weight = 1;
            for (quint32 i = 0; i < landmarks->beacons.size(); ++i) {
                auto expDist = landmarks->beacons[i].dist;
                auto landmark = CoordF(landmarks->beacons[i].x, landmarks->beacons[i].y);
                auto dist = pos.dist_to(landmark);
                auto probFromDist = 
                    gaussianProbability(dist, expDist, m_settings.move_stddev * noiseAmp);
                pos.weight *= probFromDist;
                pos.weight += m_settings.weight_compensation;

                auto expAngle = normalized_theta(landmarks->beacons[i].relative_theta);
                auto relAngle = pos.look_angle_to(landmark);
                auto probFromTheta = 
                    gaussianProbability(relAngle, expAngle, m_settings.theta_stddev * noiseAmp);
                pos.weight *= probFromTheta;
                pos.weight += m_settings.weight_compensation;
            }
            m_currentWeightSum += pos.weight;
        }
        resample();
    }
    void fillPointCloud(sensor_msgs::PointCloud &cloud) noexcept {
        cloud.points.resize(m_particles.size());
        for (quint32 i = 0; i < m_particles.size(); ++i) {
            cloud.points[i].x = m_particles[i].getX();
            cloud.points[i].y = m_particles[i].getY();
        }
    }
    void discardBelief() {
        m_currentDisperce = m_settings.max_disperce;
        m_currentMedian = {};
        randomizeParticles();
    }
    inline const PositionF &currentMedian() const noexcept {
        return m_currentMedian;
    }
    inline float density() const noexcept {
        return (m_settings.max_disperce -
                std::min(m_currentDisperce, m_settings.max_disperce)) /
                    m_settings.max_disperce;
    }
protected:
    inline static float disperceFrom(const PositionF &median, const PositionF& pos) noexcept {
        return median.dist_to(pos) + abs(normalized_theta(median.getTheta() - pos.getTheta()));
    }
    inline const PositionF &prepareNoise() noexcept {
        return m_move_noises[uniform_sample(fast_engine)];
    }
    inline PositionF noiseMove(const PositionF &from) noexcept {
        auto coeff = from.norm();
        auto noise = prepareNoise();
        return {noise.getX() * coeff, noise.getY() * coeff, noise.getTheta() * from.getTheta()};
    }
    inline PositionF resampleFrom(const PositionF &was, float noiseCoeff = 1.f) noexcept {
        return was + noiseMove(PositionF(m_settings.resample_move,
                                         m_settings.resample_move,
                                         m_settings.resample_turn) * noiseCoeff);
    }
    inline static float gaussianProbability(float value, float mean, float stddev) noexcept
    {
        float exponent = -0.5 * pow((value - mean) / stddev, 2);
        float factor = 1.0 / (stddev * 2.5066282746310002/*sqrt(pi*2)*/);
        float probability = factor * exp(exponent);
        return probability;
    }
    void resample() noexcept {
        auto lastMedian = m_currentMedian;
        m_currentDisperce = 0;
        m_currentMedian = {};
        auto pushed = 0;
        auto wasDensity = density();
        float noiseAmp = 1.f / std::max(wasDensity, m_settings.resample_min_divider);
        auto betaUsedTimes = 0;
        while (pushed < m_settings.count) {
            auto beta = uniform_01(fast_engine) * m_currentWeightSum;
            for (int i = 0; i < m_settings.count && pushed < m_settings.count; ++i) {
                if (betaUsedTimes++ > m_settings.recalc_beta_each) {
                    beta = uniform_01(fast_engine) * m_currentWeightSum;
                    betaUsedTimes = 0;
                }
                if (beta <= m_particles[i].weight) {
                    m_particles2[pushed++] = resampleFrom(m_particles[i], noiseAmp);
                    m_currentDisperce += disperceFrom(lastMedian, m_particles2[pushed-1]);
                    m_currentMedian += m_particles2[pushed-1];
                }
            }
        }
        m_currentMedian/=static_cast<float>(m_settings.count);
        m_currentDisperce/=static_cast<float>(m_settings.count);
        m_particles.swap(m_particles2);
    }
    void randomizeParticles() noexcept {
        engine.seed(std::random_device()());
        m_currentMedian = {};
        for (auto &pos : m_particles) {
            pos = PositionF(uniform_x(engine), uniform_y(engine), uniform_th(engine));
            m_currentMedian += pos;
        }
        m_currentMedian /= m_settings.count;
    }
private:
    ParticlesSettings m_settings;
    std::vector<Particle> m_particles;
    std::vector<Particle> m_particles2;
    std::vector<PositionF> m_move_noises;
    PositionF m_currentMedian;
    float m_currentWeightSum{0};
    float m_currentDisperce;
    std::mt19937 engine;
    pcg32_fast fast_engine;
    std::normal_distribution<float> normal_move;
    std::normal_distribution<float> normal_turn;
    std::uniform_int_distribution<> uniform_sample;
    std::uniform_real_distribution<float> uniform_x;
    std::uniform_real_distribution<float> uniform_y;
    std::uniform_real_distribution<float> uniform_01{0, 1};
    std::uniform_real_distribution<float> uniform_th;
};

#endif
