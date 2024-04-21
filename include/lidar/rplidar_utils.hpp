#ifndef RPLIDAR_UTILS_H
#define RPLIDAR_UTILS_H
#include "common/position.hpp"
#include "ros/console.h"
#include "sl_lidar.h"
#include <QObject>
#include "bigbang_eurobot/LaserBeacon.h"
#include <QString>
#include <QMetaEnum>
#include <unordered_map>
#include <unordered_set>
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

inline bool isOk(sl_result result) {
    return SL_IS_OK(result);
}

inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node) {
    return node.angle_z_q14 * 90.f / 16384.f;
}

class Helper {
    Q_GADGET
public:
    enum Errors {
        ErrorACCES = EACCES,
        ErrorEXIST = EEXIST,
        ErrorINTR = EINTR,
        ErrorINVAL = EINVAL,
        ErrorIO = EIO,
        ErrorISDIR = EISDIR,
        ErrorLOOP = ELOOP,
        ErrorMFILE = EMFILE,
        ErrorNAMETOOLONG = ENAMETOOLONG,
        ErrorNFILE = ENFILE,
        ErrorNOENT = ENOENT,
        ErrorNOSR = ENOSR,
        ErrorNOSPC = ENOSPC,
        ErrorNOTDIR = ENOTDIR,
        ErrorNXIO = ENXIO,
        ErrorOVERFLOW = EOVERFLOW,
        ErrorROFS = EROFS,
        ErrorAGAIN = EAGAIN,
        ErrorNOMEM = ENOMEM,
        ErrorTXTBSY = ETXTBSY,
        ErrorBadFileDesctiptor = EBADF
    };
    Q_ENUM(Errors)
    static QString printErrno() {
        return printErrno(errno);
    }
    static QString printErrno(int error) {
        auto errs = QMetaEnum::fromType<Errors>();
        auto result = errs.valueToKey(error);
        if (result) {
            return result;
        } else {
            return "NoError";
        }
    }
};

struct ParsedNode {
    float range{};
    Coord_<float> position{};
    float intensity{};
    float relTheta{};
};

struct LaserObject {
    LaserObject(Coord_<float> position, float size, float startTtl, float distTo) :
        position(position),
        size(size),
        ttl(startTtl),
        distTo(distTo)
    {}
    bool posValid(const Coord_<float> &min, const Coord_<float> &max) const {
        return min.getX() < position.getX() && position.getX() < max.getX() &&
               min.getY() < position.getY() && position.getY() < max.getY();
    }
    LaserObject &operator=(const LaserObject&) = default;
    Coord_<float> position;
    float size;
    float ttl;
    float distTo;
    float totalLifetime{0};
    int id{-1};
    float shimmering_hz{0};
    float notFoundFor{100};
    bool wasFoundLastTime{false};
};

struct ObjectDetection {
    int min_points = 3;
    int split_each = 60;
    float max_dist = 3.5;
    float max_dist_between_dots = 0.05;
    float start_ttl = 800;
    float map_ttl_coeff = 0.8;
    float max_deviation = 0.02; // m
    float max_x = 5; // m
    float min_x = -3; // m
    float max_y = 6; // m
    float min_y = -3; // m
};
DESCRIBE(ObjectDetection,
    &_::min_points, &_::split_each, &_::max_dist, &_::max_dist_between_dots, 
    &_::start_ttl, &_::map_ttl_coeff, &_::max_deviation,
    &_::max_x, &_::min_x, &_::max_y, &_::min_y
)

struct LaserBeacon {
    LaserBeacon(float targetHz, const CoordF &targetPos, int id) :
        targetHz(targetHz),
        targetPos(targetPos),
        id(id)
    {}
    float targetHz;
    CoordF targetPos;
    int id;
    void setLost(float passedTimeMs) {
        lostFor+=passedTimeMs;
        isValid = false;
    }
    void setFound(const LaserObject &obj) {
        setFound(obj.position, obj.shimmering_hz, obj.size);
    }
    bigbang_eurobot::LaserBeacon toMsg(const PositionF &from) const {
        bigbang_eurobot::LaserBeacon result;
        result.x = targetPos.getX();
        result.y = targetPos.getY();
        result.dist = from.dist_to(realPos);
        result.relative_theta = from.look_angle_to(realPos);
        return result;
    }
    void setFound(const CoordF &pos, float hz, float size) {
        lostFor = 0;
        realHz = hz;
        realPos = pos;
        this->size = size;
        isValid = true;
    }
    float size{};
    float realHz{0};
    CoordF realPos{};
    float lostFor{0};
    bool isValid{false};
};

struct BeaconPair {
    BeaconPair(const LaserBeacon &first, const LaserBeacon &second) :
        first(first),
        second(second)
    {}
    LaserBeacon first;
    LaserBeacon second;
    bool operator==(const BeaconPair &other) const {
        return first.id == other.first.id && second.id == other.second.id;
    }
    float expectedDist() const {
        return first.targetPos.dist_to(second.targetPos);
    }
    bool isValid() const {
        return first.isValid && second.isValid;
    }
};

struct ObjectsMap {
    ObjectsMap() {
        //m_objs.reserve(50);
        m_idsToRemove.reserve(50);
        m_foundIds.reserve(50);
    }
    void insertFound(LaserObject &obj, const ObjectDetection& params) {
        for (auto &iter: m_objs) {
            if (obj.position.dist_to(iter.position) < params.max_deviation) {
                iter.ttl = obj.ttl;
                m_foundIds.insert(iter.id);
                return;
            }
        }
        obj.id = m_lastId++;
        m_foundIds.insert(obj.id);
        m_objs.insert(obj.id, obj);
    }
    void update(float passedTimeMs) {
        for (auto &obj : m_objs) {
            obj.totalLifetime += passedTimeMs;
            obj.ttl -= passedTimeMs;
            if (obj.ttl <= 0) {
                m_idsToRemove.insert(obj.id);
                continue;
            }
            auto wasHz = obj.shimmering_hz;
            auto wasNotFoundFor = obj.notFoundFor + passedTimeMs;
            auto wasFound = obj.wasFoundLastTime;
            auto isFoundNow = m_foundIds.find(obj.id) != m_foundIds.end();
            if (isFoundNow) {
                obj.wasFoundLastTime = true;
                obj.notFoundFor = 0;
            } else {
                obj.wasFoundLastTime = false;
                obj.notFoundFor += passedTimeMs;
            }
            if (!wasFound && isFoundNow) {
                obj.shimmering_hz = (1000. / wasNotFoundFor + wasHz) / 2.;
            }
        }
        for (const auto id: m_idsToRemove) {
            m_objs.remove(id);
        }
        m_idsToRemove.clear();
        m_foundIds.clear();
        ROS_INFO_STREAM_THROTTLE(3, "[Lidar] Found Objects Count: " << m_objs.size());
    }
    quint32 size() const {
        return m_objs.size();
    }
    QHash<int, LaserObject>::iterator begin() {
        return m_objs.begin();
    }
    QHash<int, LaserObject>::iterator end() {
        return m_objs.end();
    }
    QHash<int, LaserObject>::const_iterator begin() const {
        return m_objs.cbegin();
    }
    QHash<int, LaserObject>::const_iterator end() const {
        return m_objs.cend();
    }
    QHash<int, LaserObject>::const_iterator cbegin() const {
        return m_objs.cbegin();
    }
    QHash<int, LaserObject>::const_iterator cend() const {
        return m_objs.cend();
    }
    QList<const LaserObject*> allWithinDist(const CoordF &target, float dist, float margin) const noexcept {
        QList<const LaserObject*> result;
        for (const auto &obj : *this) {
            if (qAbs(obj.position.dist_to(target) - dist) < margin) {
                result.append(&obj);
            }
        }
        return result;
    }
    QList<const LaserObject*> closestByHz(float targetHz, float margin) const noexcept {
        QList<const LaserObject*> result;
        for (const auto &obj : *this) {
            if (obj.shimmering_hz - targetHz <= margin) {
                result.append(&obj);
            }
        }
        std::sort(result.begin(), result.end(), [targetHz](const LaserObject* f, const LaserObject* sec) {
            return f->shimmering_hz - targetHz < sec->shimmering_hz - targetHz;
        });
        return result;
    }
    const LaserObject* closestByPos(const Coord_<float> &coord, float margin) const noexcept {
        const LaserObject* result{nullptr};
        auto minDist = std::numeric_limits<float>::max();
        for (const auto &obj : *this) {
            auto dist = coord.dist_to(obj.position);
            if (dist < minDist) {
                minDist = dist;
                result = &obj;
            }
        }
        return minDist < margin ? result : nullptr;
    }
    QHash<int, LaserObject> m_objs{};
    std::unordered_set<int> m_foundIds{};
    std::unordered_set<int> m_idsToRemove{};
    quint32 m_lastId{0};
};

#endif
