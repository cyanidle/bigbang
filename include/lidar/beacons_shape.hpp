#pragma once

#include "rplidar_utils.hpp"
#include "describe/describe.hpp"

struct ShapeDetection {
    float dist_margin = 0.05; //0.05
    int min_count = 3;
};
DESCRIBE(ShapeDetection, &_::dist_margin, &_::min_count)

struct SimpleDetection {
    float dist_margin = 0.15;
};
DESCRIBE(SimpleDetection, &_::dist_margin)

struct BeaconsParams {
    float time_before_global = 1;
    float time_for_global = 3;
    float min_size = 0.035;
    float max_size = 0.1;
    std::vector<float> all_shimmering_hz = {0.5f, 1.f, 1.5f};
    std::vector<float> all_poses_x = {0, 2, 2};
    std::vector<float> all_poses_y = {1.5, 3, 0};
    SimpleDetection simple;
    ShapeDetection shape;
};
DESCRIBE(BeaconsParams, 
    &_::time_before_global, &_::time_for_global, &_::min_size,
    &_::max_size, &_::all_shimmering_hz, &_::all_poses_x,
    &_::all_poses_y, &_::simple, &_::shape)

struct BeaconsShape {
    BeaconsShape() = default;
    BeaconsShape(const std::vector<LaserBeacon> &beacons, const BeaconsParams& params) :
        m_params(params)
    {
        m_offsets.reserve(beacons.size());
        m_results.resize(beacons.size());
        if (beacons.size() < 2) throw std::runtime_error("Minimum 2 beacons required!");
        const auto &first = *beacons.begin();
        m_offsets.emplace_back();
        for (int i = 1; i < beacons.size(); ++i) {
            m_offsets.push_back(beacons[i].targetPos - first.targetPos);
        }
    }
    const std::vector<const LaserObject *> &findIn(const ObjectsMap &map)
    {
        m_results.assign(m_results.size(), nullptr);
        auto maxFound = 0;
        auto minDeviation = std::numeric_limits<float>::max();
        for (const auto &obj: map) {
            if (!checkForSize(obj)) continue;
            for (int iAmOffsetNumber = 0; iAmOffsetNumber < m_offsets.size(); ++iAmOffsetNumber) { // i am this point now 0,0 or 1,1 e.t.c
                for (int iAmSecondOffsetN = 0; iAmSecondOffsetN < m_offsets.size(); ++iAmSecondOffsetN) { // choose existing pair from
                    if (iAmOffsetNumber == iAmSecondOffsetN) continue;
                    const auto &offset1 = m_offsets[iAmOffsetNumber];
                    const auto &offset2 = m_offsets[iAmSecondOffsetN];
                    auto offsetDiff = CoordF(offset2 - offset1);
                    auto candidatesForSecond = map.allWithinDist(obj.position, offsetDiff.norm(), m_params.shape.dist_margin);
                    if (candidatesForSecond.isEmpty()) {
                        ROS_INFO_THROTTLE(1.2, "Cannot find any candidates for second point!");
                    }
                    auto baseRotation = offsetDiff.angle_from_x_axis();
                    for (const auto &secondPoint : qAsConst(candidatesForSecond)) { //
                        if (!checkForSize(*secondPoint)) continue;
                        auto rotation = obj.position.angle_from_x_axis_to(secondPoint->position) - baseRotation;
                        auto found = countWithRot(obj, iAmOffsetNumber, map, rotation);
                        if (found.count > maxFound ||
                            (found.count == maxFound && found.sumDeviation < minDeviation))
                        {
                            maxFound = found.count;
                            minDeviation = found.sumDeviation;
                            for (int beaconNum = 0; beaconNum < m_results.size(); ++beaconNum) {
                                if (beaconNum == iAmOffsetNumber) {
                                    m_results[beaconNum] = &obj;
                                } else {
                                    m_results[beaconNum] =
        map.closestByPos(obj.position + CoordF(m_offsets[beaconNum] - offset1).rotated_by(rotation), m_params.shape.dist_margin);
                                }
                            }
                        }
                    }
                }
            }
        }
        if (maxFound == 1 || maxFound < m_params.shape.min_count) { // one beacon does not count
            m_results.assign(m_results.size(), nullptr);
        }
        return m_results;
    }
protected:
    struct CountResult {
        int count;
        float sumDeviation;
    };
    bool checkForSize(const LaserObject &obj) const {
        return obj.size >= m_params.min_size && obj.size <= m_params.max_size;
    }
    CountResult countWithRot(const LaserObject &base, int isOffset, const ObjectsMap &map, float rot) {
        auto count = 1;
        float sumDeviation = 0;
        const auto &baseOffset = m_offsets[isOffset];
        for (int i =0; i < m_offsets.size(); ++i) {
            if (i == isOffset) continue;
            const auto &offset = CoordF(m_offsets[i] - baseOffset);
            auto toCheck = offset.rotated_by(rot) + base.position;
            if (auto found = map.closestByPos(toCheck, m_params.shape.dist_margin)) {
                if (!checkForSize(*found)) continue;
                count++;
                sumDeviation += found->position.dist_to(toCheck);
            }
        }
        return {count, sumDeviation};
    }
private:
    BeaconsParams m_params;
    std::vector<const LaserObject *> m_results;
    std::vector<CoordF> m_offsets;
};
